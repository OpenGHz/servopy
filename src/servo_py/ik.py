"""Checked adapters for an application-owned position IK solver.

This module reads no clock, starts no thread and owns no motion reference.
Call Servo.step with fresh feedback and time after preparing each command.
"""
from __future__ import annotations

from dataclasses import dataclass
import operator
from typing import Callable

import numpy as np
from numpy.typing import ArrayLike

from . import _core
from .api import JointPositionCommand, PoseCommand, Servo, StopCommand


@dataclass(frozen=True)
class PositionIKResult:
    command: JointPositionCommand | StopCommand
    message: str
    position_error: float | None = None
    orientation_error: float | None = None

    @property
    def success(self) -> bool:
        return isinstance(self.command, JointPositionCommand)


class PositionIKAdapter:
    """Prepare joint position commands using ``solver(base_T_tip, q_seed)``.

    ``servo`` supplies effective limits, frames, task axes and timeouts.
    The solver returns a joint vector or None. Invalid/failed results become
    StopCommand; the caller must still run Servo.step and execute braking.
    The original pose timestamp is preserved. A blocking Python solver cannot
    be interrupted by this adapter.
    """

    def __init__(
        self,
        servo: Servo,
        solver: Callable[[np.ndarray, np.ndarray], ArrayLike | None],
        *,
        max_joint_step: ArrayLike | None = None,
        position_tolerance: float | None = None,
        orientation_tolerance: float | None = None,
    ):
        if not callable(solver):
            raise TypeError("solver must be callable")
        self.solver = solver
        self.model = servo.model
        self._limits = servo.limits
        self._dof = self.model.dof()
        self._base = self.model.base_frame()
        self._axes = tuple(servo.config.task_axes)
        self._timeout_ns = servo.config.command_timeout * 1e9
        self.position_tolerance = servo.config.position_tolerance if position_tolerance is None else position_tolerance
        self.orientation_tolerance = servo.config.orientation_tolerance if orientation_tolerance is None else orientation_tolerance
        for tolerance in (self.position_tolerance, self.orientation_tolerance):
            if not np.isfinite(tolerance) or tolerance <= 0:
                raise ValueError("IK pose tolerances must be finite and positive")
        self._max_step = None
        if max_joint_step is not None:
            bound = np.asarray(max_joint_step, dtype=float)
            if bound.ndim == 0:
                bound = np.full(self._dof, bound)
            if bound.shape != (self._dof,) or not np.isfinite(bound).all() or np.any(bound <= 0):
                raise ValueError("max_joint_step must be positive and finite, scalar or one value per joint")
            self._max_step = bound.copy()

    def solve(self, command: PoseCommand, q_seed: ArrayLike, *, now_ns: int) -> PositionIKResult:
        """Return a checked joint target or StopCommand, never an old solution."""
        def failed(message, position=None, orientation=None):
            return PositionIKResult(StopCommand(), message, position, orientation)

        try:
            if not isinstance(command, PoseCommand):
                return failed("expected a PoseCommand")
            stamp, now = operator.index(command.stamp_ns), operator.index(now_ns)
            if not 0 <= stamp <= now < 2**63:
                return failed("invalid or future target timestamp")
            if now - stamp > self._timeout_ns:
                return failed("pose target expired before IK")
            if command.expressed_in not in ("base", self._base):
                return failed("IK pose target must be expressed in the model base frame")
            target = np.asarray(command.pose, dtype=float).copy()
            if target.shape != (4, 4) or not _core.valid_pose(target):
                return failed("IK target must be a finite rigid 4x4 transform")
            seed = np.asarray(q_seed, dtype=float).copy()
            if seed.shape != (self._dof,) or not np.isfinite(seed).all():
                return failed("IK seed must be a finite vector matching the model DOF")
        except (TypeError, ValueError, OverflowError) as exc:
            return failed(f"invalid IK input: {exc}")

        try:
            solution = self.solver(target.copy(), seed.copy())
        except Exception as exc:
            return failed(f"IK solver raised {type(exc).__name__}: {exc}")
        if solution is None:
            return failed("IK did not find a solution")
        try:
            q = np.asarray(solution, dtype=float).copy()
            if q.shape != (self._dof,) or not np.isfinite(q).all():
                return failed("IK solution must be a finite vector matching the model DOF")
            if (np.any(q < self._limits.lower + self._limits.margin)
                    or np.any(q > self._limits.upper - self._limits.margin)):
                return failed("IK solution is outside the effective Servo position limits")
            delta = np.asarray(self.model.difference(q, seed), dtype=float)
            if delta.shape != (self._dof,) or not np.isfinite(delta).all():
                return failed("model returned an invalid joint difference")
            if self._max_step is not None and np.any(np.abs(delta) > self._max_step):
                return failed("IK solution exceeds max_joint_step from the reference seed")
            actual = np.asarray(self.model.fk(q), dtype=float)
            if actual.shape != (4, 4) or not _core.valid_pose(actual):
                return failed("model FK returned an invalid rigid transform")
            linear = target[:3, 3] - actual[:3, 3]
            angular = _core.rotation_log(target[:3, :3] @ actual[:3, :3].T)
            position_error = float(np.linalg.norm([linear[i] for i in self._axes if i < 3]))
            orientation_error = float(np.linalg.norm([angular[i - 3] for i in self._axes if i >= 3]))
            if (not np.isfinite(position_error) or not np.isfinite(orientation_error)
                    or position_error > self.position_tolerance
                    or orientation_error > self.orientation_tolerance):
                return failed("IK solution failed the task pose residual check", position_error, orientation_error)
        except Exception as exc:
            return failed(f"IK solution validation raised {type(exc).__name__}: {exc}")
        return PositionIKResult(JointPositionCommand(q, stamp), "IK solution accepted", position_error, orientation_error)
