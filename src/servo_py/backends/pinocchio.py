"""Optional Pinocchio adapter; Python callbacks reacquire the GIL.

Use the native SerialChainModel when an entirely native step is required.
This adapter is useful for independent model validation and experimentation.
"""
from __future__ import annotations
import threading
import numpy as np
from ..api import Kinematics
from ..urdf import parse_chain


class PinocchioModel(Kinematics):
    @classmethod
    def from_urdf(cls, path, *, base, tip, acceleration_limits, velocity_limits=None, margin=0.01):
        return cls(path, base=base, tip=tip, acceleration_limits=acceleration_limits,
                   velocity_limits=velocity_limits, margin=margin)

    def __init__(self, path, *, base, tip, acceleration_limits, velocity_limits=None, margin=0.01):
        super().__init__()
        try:
            import pinocchio as pin
        except ImportError as error:
            raise ImportError("PinocchioModel requires the 'pin' distribution; install servo-py[pinocchio]") from error
        self._pin = pin
        self._spec = parse_chain(path, base=base, tip=tip, acceleration_limits=acceleration_limits,
                                 velocity_limits=velocity_limits, margin=margin)
        full = pin.buildModelFromUrdf(str(path))
        selected = set(self._spec.names)
        lock_ids = [i for i in range(1, full.njoints) if full.names[i] not in selected]
        self._model = pin.buildReducedModel(full, lock_ids, pin.neutral(full))
        self._data = self._model.createData()
        self._base_id = self._model.getFrameId(base)
        self._tip_id = self._model.getFrameId(tip)
        if self._base_id >= self._model.nframes or self._tip_id >= self._model.nframes:
            raise ValueError("base/tip missing from reduced Pinocchio model")
        self._joints = [self._model.joints[self._model.getJointId(name)] for name in self._spec.names]
        for joint, kind in zip(self._joints, self._spec.types):
            if joint.nv != 1 or joint.nq != (2 if kind == "continuous" else 1):
                raise ValueError("unsupported Pinocchio joint representation")
        self._lock = threading.RLock()
        self.limits = self._spec.limits._native()

    def dof(self):
        return len(self._spec.names)

    def joint_names(self):
        return list(self._spec.names)

    def base_frame(self):
        return self._spec.base

    def tip_frame(self):
        return self._spec.tip

    def _q(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof(),) or not np.isfinite(q).all():
            raise ValueError("q must be finite and match the selected joint count")
        native = self._pin.neutral(self._model)
        for i, (joint, kind) in enumerate(zip(self._joints, self._spec.types)):
            if kind == "continuous":
                native[joint.idx_q:joint.idx_q + 2] = np.cos(q[i]), np.sin(q[i])
            else:
                native[joint.idx_q] = q[i]
        return native

    def fk(self, q):
        with self._lock:
            self._pin.forwardKinematics(self._model, self._data, self._q(q))
            self._pin.updateFramePlacements(self._model, self._data)
            transform = self._data.oMf[self._base_id].inverse() * self._data.oMf[self._tip_id]
            return np.array(transform.homogeneous, copy=True)

    def jacobian(self, q):
        with self._lock:
            native = self._q(q)
            self._pin.computeJointJacobians(self._model, self._data, native)
            self._pin.updateFramePlacements(self._model, self._data)
            world = self._pin.getFrameJacobian(self._model, self._data, self._tip_id,
                                               self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            rotation = self._data.oMf[self._base_id].rotation.T
            selected = np.array(world[:, [joint.idx_v for joint in self._joints]], copy=True)
            selected[:3] = rotation @ selected[:3]
            selected[3:] = rotation @ selected[3:]
            return selected

    def difference(self, q1, q0):
        self._q(q1)
        self._q(q0)
        diff = np.asarray(q1, dtype=float) - np.asarray(q0, dtype=float)
        for i, kind in enumerate(self._spec.types):
            if kind == "continuous":
                diff[i] = np.arctan2(np.sin(diff[i]), np.cos(diff[i]))
        return diff
