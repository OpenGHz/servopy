"""Typed Python entry point. All motion math and reference generation live in C++."""
from __future__ import annotations

from dataclasses import dataclass, fields
from enum import IntFlag
from typing import Sequence
import numpy as np
from numpy.typing import ArrayLike
from . import _core

Action = _core.Action
Kinematics = _core.Kinematics
SerialChainModel = _core.SerialChainModel


class SafetyFlag(IntFlag):
    NONE = _core.NONE
    INVALID_COMMAND = _core.INVALID_COMMAND
    INVALID_STATE = _core.INVALID_STATE
    INVALID_TIMING = _core.INVALID_TIMING
    STALE_COMMAND = _core.STALE_COMMAND
    STALE_STATE = _core.STALE_STATE
    FUTURE_TIMESTAMP = _core.FUTURE_TIMESTAMP
    TRACKING_ERROR = _core.TRACKING_ERROR
    VELOCITY_LIMIT = _core.VELOCITY_LIMIT
    ACCELERATION_LIMIT = _core.ACCELERATION_LIMIT
    POSITION_LIMIT = _core.POSITION_LIMIT
    SINGULARITY_DECELERATION = _core.SINGULARITY_DECELERATION
    SINGULARITY_HALT = _core.SINGULARITY_HALT
    LEAVING_SINGULARITY = _core.LEAVING_SINGULARITY
    COLLISION_DISABLED = _core.COLLISION_DISABLED
    COLLISION_MISSING = _core.COLLISION_MISSING
    COLLISION_STALE = _core.COLLISION_STALE
    COLLISION_DECELERATION = _core.COLLISION_DECELERATION
    COLLISION_HALT = _core.COLLISION_HALT
    INFEASIBLE = _core.INFEASIBLE
    FAULT_LATCHED = _core.FAULT_LATCHED
    GOAL_REACHED = _core.GOAL_REACHED
    MODE_SWITCH = _core.MODE_SWITCH
    MODEL_ERROR = _core.MODEL_ERROR


@dataclass(frozen=True)
class ServoConfig:
    command_timeout: float = 0.1
    state_timeout: float = 0.1
    collision_timeout: float = 0.1
    max_dt: float = 0.05
    timing_tolerance: float = 0.5
    max_tracking_error: float = 0.2
    position_gain: float = 2.0
    orientation_gain: float = 2.0
    max_linear_speed: float = 0.2
    max_angular_speed: float = 0.5
    position_tolerance: float = 1e-4
    orientation_tolerance: float = 1e-3
    min_damping: float = 1e-4
    max_damping: float = 0.1
    damping_threshold: float = 0.1
    singularity_soft: float = 0.05
    singularity_hard: float = 0.001
    singularity_probe_step: float = 0.01
    singularity_escape_epsilon: float = 1e-6
    collision_required: bool = False
    task_axes: Sequence[int] = (0, 1, 2, 3, 4, 5)
    task_weights: ArrayLike = (1., 1., 1., 1., 1., 1.)

    def _native(self):
        config = _core.Config()
        for field in fields(self):
            setattr(config, field.name, getattr(self, field.name))
        return config


@dataclass(frozen=True)
class JointLimits:
    lower: ArrayLike
    upper: ArrayLike
    velocity: ArrayLike
    acceleration: ArrayLike
    margin: ArrayLike = 0.01

    def _native(self):
        limits = _core.Limits()
        for name in ("lower", "upper", "velocity", "acceleration"):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.ndim != 1:
                raise ValueError(f"{name} limits must be a 1D array")
            setattr(limits, name, values)
        margin = np.asarray(self.margin, dtype=float)
        if margin.ndim == 0:
            margin = np.full(len(limits.lower), margin)
        if margin.ndim != 1:
            raise ValueError("margin must be a scalar or a 1D array")
        limits.margin = margin
        limits.validate(len(limits.lower))
        return limits


@dataclass(frozen=True)
class JointState:
    q: ArrayLike
    dq: ArrayLike
    stamp_ns: int

    def _native(self):
        return _core.State(self.q, self.dq, self.stamp_ns)


@dataclass(frozen=True)
class JointJogCommand:
    velocities: ArrayLike
    stamp_ns: int
    names: Sequence[str] | None = None


@dataclass(frozen=True)
class TwistCommand:
    linear: ArrayLike
    angular: ArrayLike
    stamp_ns: int
    expressed_in: str = "base"
    reference_point: str = "tool"


@dataclass(frozen=True)
class PoseCommand:
    pose: ArrayLike
    stamp_ns: int
    expressed_in: str = "base"


@dataclass(frozen=True)
class StopCommand:
    """Request an acceleration-limited stop; no timestamp is needed."""


@dataclass(frozen=True)
class CollisionSample:
    velocity_scale: float
    stamp_ns: int
    state_stamp_ns: int

    def _native(self):
        return _core.CollisionSample(self.velocity_scale, self.stamp_ns, self.state_stamp_ns)


@dataclass(frozen=True)
class StepResult:
    action: Action
    reference: _core.Reference | None
    flags: SafetyFlag
    diagnostics: _core.Diagnostics
    message: str


class Servo:
    """Single-consumer, stateful servo. REJECT latches until explicit reset()."""

    def __init__(self, model: Kinematics, config: ServoConfig | None = None,
                 limits: JointLimits | None = None):
        self.model = model
        self.config = config or ServoConfig()
        native_limits = limits._native() if limits is not None else model.limits
        self._core = _core.ServoCore(model, native_limits, self.config._native())
        self._names = tuple(model.joint_names())
        self._indices = {name: i for i, name in enumerate(self._names)}
        self._base = model.base_frame()
        self._tip = model.tip_frame()

    def reset(self, state: JointState, *, now_ns: int):
        self._core.reset(state._native(), now_ns)

    def _command(self, command, now_ns):
        result = _core.Command()
        result.stamp_ns = now_ns
        try:
            if isinstance(command, StopCommand):
                return result
            result.stamp_ns = command.stamp_ns
            if isinstance(command, JointJogCommand):
                result.type = _core.CommandType.JOINT_JOG
                values = np.asarray(command.velocities, dtype=float)
                if values.ndim != 1:
                    raise ValueError("joint velocities must be a vector")
                if command.names is not None:
                    if len(set(command.names)) != len(command.names) or len(command.names) != len(values):
                        raise ValueError("joint names must be unique and match velocities")
                    mapped = np.zeros(len(self._names))
                    for name, value in zip(command.names, values):
                        mapped[self._indices[name]] = value
                    values = mapped
                result.joint_velocity = values
            elif isinstance(command, TwistCommand):
                result.type = _core.CommandType.TWIST
                result.linear = command.linear
                result.angular = command.angular
                if command.expressed_in in ("base", self._base):
                    result.frame = _core.Frame.BASE
                elif command.expressed_in in ("tool", self._tip):
                    result.frame = _core.Frame.TOOL
                else:
                    raise ValueError("unknown expression frame")
                if command.reference_point not in ("tool", self._tip):
                    raise ValueError("this version accepts twist velocity at the TCP only")
            elif isinstance(command, PoseCommand):
                result.type = _core.CommandType.POSE
                result.pose = command.pose
                if command.expressed_in not in ("base", self._base):
                    raise ValueError("pose targets must be expressed in base")
            else:
                raise ValueError("unknown command type")
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
            result.valid = False
        return result

    def step(self, state: JointState, command, *, dt: float, now_ns: int,
             collision: CollisionSample | None = None) -> StepResult:
        result = self._core.step(state._native(), self._command(command, now_ns), dt, now_ns,
                                 None if collision is None else collision._native())
        return StepResult(result.action, result.reference, SafetyFlag(result.flags),
                          result.diagnostics, result.message)
