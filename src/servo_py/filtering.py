"""Low-pass filtering of motion commands before they reach Servo.step.

The filter conditions the *request*. It never edits the generated reference,
so the core's velocity, acceleration, position-envelope and stop guarantees
are unchanged. It reads no clock and owns no thread.
"""
from __future__ import annotations

from dataclasses import replace
import math

import numpy as np

from . import _core
from .api import JointJogCommand, JointPositionCommand, PoseCommand, Servo, TwistCommand


def _skew(w):
    return np.array([[0., -w[2], w[1]], [w[2], 0., -w[0]], [-w[1], w[0], 0.]])


def _exp(w):
    """Rotation matrix exp([w]x), with series coefficients near zero."""
    theta = float(np.linalg.norm(w))
    k = _skew(w)
    if theta < 1e-6:
        a, b = 1. - theta * theta / 6., .5 - theta * theta / 24.
    else:
        a, b = math.sin(theta) / theta, (1. - math.cos(theta)) / (theta * theta)
    return np.eye(3) + a * k + b * (k @ k)


def _orthonormal(r):
    u, _, vt = np.linalg.svd(r)
    return u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt


class ButterworthCommandFilter:
    """First-order Butterworth low-pass for commands, applied once per cycle.

    Call ``filter(command, dt=dt, now_ns=now_ns)`` with the latest command and
    the same ``dt``/``now_ns`` you then pass to ``Servo.step``. The bilinear
    coefficient follows the actual period, ``C = cot(pi * cutoff_hz * dt)``, so
    the cutoff stays fixed when the period varies. Each output is a weighted
    mean of the current input, previous input and previous output with
    non-negative weights; it never overshoots the inputs.

    Velocity commands (JointJog, Twist) start from rest after a reset; target
    commands (JointPosition, Pose) start at the first target. JointPosition
    targets are filtered with the model's joint difference (shortest angle on
    continuous joints) and Pose orientation on SO(3). Named commands are
    returned in model order. The latest source timestamp is preserved.

    Stop, stale, future-stamped, malformed or unsupported commands are returned
    unchanged and reset the filter, so braking is never delayed and the Servo
    still reports the cause. A change of command type or Twist expression
    frame also resets it. One filter serves one command stream and one Servo.
    """

    def __init__(self, servo: Servo, cutoff_hz: float):
        cutoff = float(cutoff_hz)
        max_dt = servo.config.max_dt
        if not math.isfinite(cutoff) or cutoff <= 0:
            raise ValueError("cutoff_hz must be positive and finite")
        if cutoff * max_dt > .25:
            raise ValueError(f"cutoff_hz must not exceed 1 / (4 * max_dt) = {.25 / max_dt:g} Hz; "
                             "reduce the cutoff or ServoConfig.max_dt")
        self.cutoff_hz = cutoff
        self.model = servo.model
        limits = servo.limits
        margin = np.asarray(limits.margin, dtype=float)
        self._lower = np.asarray(limits.lower, dtype=float) + margin
        self._upper = np.asarray(limits.upper, dtype=float) - margin
        self._max_dt = max_dt
        self._timeout_ns = servo.config.command_timeout * 1e9
        self._names = tuple(self.model.joint_names())
        self._indices = {name: i for i, name in enumerate(self._names)}
        self._base = self.model.base_frame()
        self._tip = self.model.tip_frame()
        self.reset()

    def reset(self):
        """Forget history; the next accepted command starts a new sequence."""
        self._key = None
        self._inputs = None
        self._outputs = None

    def filter(self, command, *, dt: float, now_ns: int):
        try:
            prepared = self._prepare(command, dt, now_ns)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
            prepared = None
        if prepared is None:
            self.reset()
            return command
        key, inputs = prepared
        if key != self._key:
            self._key = key
            rest = key[0] in ("jog", "twist")
            start = tuple(np.zeros_like(x) if rest else x for x in inputs)
            self._inputs, self._outputs = start, start
        c = max(1., 1. / math.tan(math.pi * self.cutoff_hz * dt))
        outputs = []
        for kind, x, x1, y1 in zip(self._kinds(key), inputs, self._inputs, self._outputs):
            if kind == "vector":
                y = (x + x1 + (c - 1.) * y1) / (1. + c)
            elif kind == "joint":
                step = (np.asarray(self.model.difference(x, y1)) +
                        np.asarray(self.model.difference(x1, y1))) / (1. + c)
                # Inputs lie inside the effective limits, so the clip only
                # removes rounding that could otherwise invalidate the target.
                y = np.clip(np.asarray(self.model.integrate(y1, step), dtype=float), self._lower, self._upper)
            else:
                w = (_core.rotation_log(x @ y1.T) + _core.rotation_log(x1 @ y1.T)) / (1. + c)
                y = _orthonormal(_exp(w) @ y1)
            outputs.append(y)
        self._inputs, self._outputs = inputs, tuple(outputs)
        return self._build(command, outputs)

    @staticmethod
    def _kinds(key):
        return {"jog": ("vector",), "twist": ("vector", "vector"),
                "position": ("joint",), "pose": ("vector", "rotation")}[key[0]]

    def _vector(self, values, size):
        array = np.array(values, dtype=float)
        if array.shape != (size,) or not np.all(np.isfinite(array)):
            raise ValueError("invalid command vector")
        return array

    def _prepare(self, command, dt, now_ns):
        if not isinstance(command, (JointJogCommand, JointPositionCommand, TwistCommand, PoseCommand)):
            return None
        stamp = command.stamp_ns
        if (isinstance(stamp, bool) or isinstance(now_ns, bool) or
                not isinstance(stamp, (int, np.integer)) or not isinstance(now_ns, (int, np.integer))):
            return None
        if not math.isfinite(dt) or not 0 < dt <= self._max_dt:
            return None
        if stamp < 0 or stamp > now_ns or now_ns - stamp > self._timeout_ns:
            return None
        n = len(self._names)
        if isinstance(command, JointJogCommand):
            if command.names is None:
                return ("jog",), (self._vector(command.velocities, n),)
            values = self._vector(command.velocities, len(command.names))
            if len(set(command.names)) != len(command.names):
                return None
            mapped = np.zeros(n)
            for name, value in zip(command.names, values):
                mapped[self._indices[name]] = value
            return ("jog",), (mapped,)
        if isinstance(command, JointPositionCommand):
            values = self._vector(command.positions, n)
            if command.names is not None:
                if len(command.names) != n or set(command.names) != set(self._names):
                    return None
                positions = dict(zip(command.names, values))
                values = np.array([positions[name] for name in self._names])
            if np.any(values < self._lower) or np.any(values > self._upper):
                return None
            return ("position",), (values,)
        if isinstance(command, TwistCommand):
            if command.reference_point not in ("tool", self._tip):
                return None
            if command.expressed_in in ("base", self._base):
                frame = "base"
            elif command.expressed_in in ("tool", self._tip):
                frame = "tool"
            else:
                return None
            return ("twist", frame), (self._vector(command.linear, 3), self._vector(command.angular, 3))
        pose = np.array(command.pose, dtype=float)
        if pose.shape != (4, 4) or command.expressed_in not in ("base", self._base) or not _core.valid_pose(pose):
            return None
        return ("pose",), (pose[:3, 3].copy(), _orthonormal(pose[:3, :3]))

    @staticmethod
    def _build(command, outputs):
        if isinstance(command, JointJogCommand):
            return replace(command, velocities=outputs[0], names=None)
        if isinstance(command, JointPositionCommand):
            return replace(command, positions=outputs[0], names=None)
        if isinstance(command, TwistCommand):
            return replace(command, linear=outputs[0], angular=outputs[1])
        pose = np.eye(4)
        pose[:3, 3], pose[:3, :3] = outputs[0], outputs[1]
        return replace(command, pose=pose)
