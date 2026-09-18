"""Panda servoing with torque or joint-position actuators and a default viewer.

Run after installing servo-py[mujoco]: servo-py-panda
From a source checkout: python examples/mujoco_panda.py
Position control: add --control-mode joint-position or --control-mode ik-position
Drag a target in the viewer: add --interactive-target (torque or ik-position)
Headless recording: MUJOCO_GL=egl python examples/mujoco_panda.py --headless --record demo.mp4
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from contextlib import ExitStack, contextmanager, nullcontext
from io import BytesIO
import json
import os
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

try:
    import mujoco
except ImportError as exc:
    raise ImportError("Install the demo dependencies: python -m pip install 'servo-py[mujoco]'") from exc

from servo_py import (
    Action, JointPositionCommand, JointLimits, JointState, Kinematics, PoseCommand, PositionIKAdapter,
    SafetyFlag, Servo, ServoConfig, StopCommand, BoxQPSolver, RuckigSmoothing,
    LatestCommand, JsonlRecorder, command_from_dict,
)
from servo_py.examples._panda_assets import panda_archive

ASSETS = Path(__file__).resolve().parent / "assets"
CONTROL_DT = 0.01
PHYSICS_DT = 0.002
ARM_NAMES = tuple(f"joint{i}" for i in range(1, 8))
CONTROL_MODES = ("torque", "joint-position", "ik-position")


def load_panda(*, width=960, height=640, actuator_mode="torque", interactive_target=False):
    """Load the pinned, unmodified Menagerie assets from an in-memory ZIP.

    Scene and actuator changes below are specific to this example. The source
    MJCF and meshes are loaded from the checkout, a verified user-supplied ZIP,
    or the download cache. Licenses and provenance remain in examples/assets.
    """
    if actuator_mode not in ("torque", "position"):
        raise ValueError("actuator_mode must be torque or position")
    with zipfile.ZipFile(BytesIO(panda_archive(ASSETS))) as archive:
        assets = {name: archive.read(name) for name in archive.namelist()}
    root = ET.fromstring(assets["panda.xml"])
    root.find("option").set("timestep", str(PHYSICS_DT))
    # Position mode retains the original gains, angle/force limits and home
    # controls. Only torque mode replaces the seven arm position actuators.
    # The finger tendon retains the upstream position actuator in both modes.
    if actuator_mode == "torque":
        for i, actuator in enumerate(list(root.find("actuator"))[:7]):
            maximum = 87 if i < 4 else 12
            actuator.set("gainprm", "1")
            actuator.set("biasprm", "0 0 0")
            actuator.set("ctrlrange", f"{-maximum} {maximum}")
        root.find("keyframe/key").attrib.pop("ctrl", None)
    hand = root.find(".//body[@name='hand']")
    ET.SubElement(hand, "site", name="tcp", pos="0 0 0.1034", size="0.007", rgba="0.1 0.8 0.9 1")

    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    ET.SubElement(visual, "quality", shadowsize="2048", offsamples="4")
    ET.SubElement(visual, "headlight", diffuse="0.55 0.55 0.55", ambient="0.35 0.35 0.35", specular="0.2 0.2 0.2")
    ET.SubElement(visual, "rgba", haze="0.16 0.20 0.25 1")
    asset = root.find("asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.19 0.24 0.30", rgb2="0.06 0.09 0.14", width="512", height="3072")
    ET.SubElement(asset, "texture", name="floor_tex", type="2d", builtin="checker", rgb1="0.18 0.22 0.27", rgb2="0.15 0.19 0.24", width="512", height="512")
    ET.SubElement(asset, "material", name="floor_mat", texture="floor_tex", texrepeat="4 4", texuniform="true", reflectance="0.12")
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", pos="0 0 -0.002", material="floor_mat")
    ET.SubElement(world, "light", pos="1 -1 2", dir="-0.5 0.5 -1", diffuse="0.8 0.8 0.8")
    target = ET.SubElement(world, "body", name="target", mocap="true")
    if interactive_target:
        # Native perturbations use the inertial frame. Keep it at the handle
        # origin so RGB axis geoms do not shift its rotation pivot/frame.
        ET.SubElement(target, "inertial", pos="0 0 0", mass="0.01", diaginertia="0.00001 0.00001 0.00001")
    ET.SubElement(target, "geom", name="target_handle", type="sphere",
                  size="0.022" if interactive_target else "0.015",
                  rgba="1 0.55 0.16 0.95", contype="0", conaffinity="0")
    if interactive_target:
        # Selectable, non-contact pose handle. RGB axes make rotation visible.
        for axis, endpoint, color in (
            ("x", "0.08 0 0", "0.95 0.2 0.2 0.9"),
            ("y", "0 0.08 0", "0.2 0.85 0.3 0.9"),
            ("z", "0 0 0.08", "0.2 0.45 1 0.9"),
        ):
            ET.SubElement(target, "geom", name=f"target_{axis}", type="capsule",
                          fromto=f"0 0 0 {endpoint}", size="0.0025", rgba=color,
                          contype="0", conaffinity="0")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), assets=assets)


class PandaKinematics(Kinematics):
    """FK/Jacobian callbacks use private scratch data, never live physics data."""

    def __init__(self, model):
        super().__init__()
        self._model = model
        self._data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, self._data, model.key("home").id)
        self.joint_ids = np.array([model.joint(name).id for name in ARM_NAMES])
        self.q_ids = model.jnt_qposadr[self.joint_ids].copy()
        self.v_ids = model.jnt_dofadr[self.joint_ids].copy()
        self.site_id = model.site("tcp").id
        self.limits = JointLimits(
            lower=model.jnt_range[self.joint_ids, 0],
            upper=model.jnt_range[self.joint_ids, 1],
            velocity=[2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61],
            acceleration=np.full(7, 3.0),  # Conservative demo setting, not hardware specification.
            margin=0.02,
        )._native()

    def dof(self):
        return 7

    def joint_names(self):
        return list(ARM_NAMES)

    def base_frame(self):
        return "world"

    def tip_frame(self):
        return "tcp"

    def _update(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("Panda arm q must be a finite 7-vector")
        self._data.qpos[self.q_ids] = q
        mujoco.mj_kinematics(self._model, self._data)
        mujoco.mj_comPos(self._model, self._data)

    def fk(self, q):
        self._update(q)
        pose = np.eye(4)
        pose[:3, :3] = self._data.site_xmat[self.site_id].reshape(3, 3)
        pose[:3, 3] = self._data.site_xpos[self.site_id]
        return pose

    def jacobian(self, q):
        self._update(q)
        linear, angular = np.zeros((3, self._model.nv)), np.zeros((3, self._model.nv))
        mujoco.mj_jacSite(self._model, self._data, linear, angular, self.site_id)
        return np.vstack((linear[:, self.v_ids], angular[:, self.v_ids]))


def trajectory_phase(t, duration):
    u = np.clip((t - 1.0) / (duration - 4.0), 0.0, 1.0)
    return 2 * np.pi * (10 * u**3 - 15 * u**4 + 6 * u**5)


def target_pose(home_pose, t, duration):
    """One smooth 3D figure eight, followed by settling and a stop segment."""
    phase = trajectory_phase(t, duration)
    pose = home_pose.copy()
    pose[:3, 3] += [-0.045 * (1 - np.cos(phase)), 0.14 * np.sin(phase), 0.08 * np.sin(2 * phase)]
    return pose


def target_joints(home_q, t, duration):
    """A smooth seven-joint demonstration, returning home without any IK."""
    phase = trajectory_phase(t, duration)
    sine, bend = np.sin(phase), 1 - np.cos(phase)
    return home_q + np.array([
        0.30 * sine, -0.12 * bend, 0.18 * np.sin(2 * phase),
        -0.18 * bend, 0.12 * sine, 0.15 * bend, 0.20 * sine,
    ])


def pose_error(target, actual):
    """World-frame TCP translation and shortest rotation-vector error."""
    quaternion, angular = np.empty(4), np.empty(3)
    mujoco.mju_mat2Quat(quaternion, (target[:3, :3] @ actual[:3, :3].T).ravel())
    mujoco.mju_quat2Vel(angular, quaternion, 1.0)
    return np.r_[target[:3, 3] - actual[:3, 3], angular]


def checked_joint_target(backend, q):
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("joint target must be a finite 7-vector in radians")
    limits = backend.limits
    if np.any(q < limits.lower + limits.margin) or np.any(q > limits.upper - limits.margin):
        raise ValueError("joint target is outside the Servo position limits")
    return q.copy()


class PandaPositionIK:
    """Bounded, local position IK: (world_T_tcp, q_seed) -> q_target or None.

    This Python solver is upstream of Servo, independent of its differential IK.
    Replace this callable with an existing Python IK to reuse the same demo.
    """

    def __init__(self, backend, *, max_iterations=40):
        self.backend = backend
        self.max_iterations = max_iterations

    def __call__(self, target, q_seed):
        target = np.asarray(target, dtype=float)
        if (target.shape != (4, 4) or not np.isfinite(target).all()
                or not np.allclose(target[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
                or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-8, rtol=0)
                or not np.isclose(np.linalg.det(target[:3, :3]), 1, atol=1e-8, rtol=0)):
            raise ValueError("IK target must be a finite rigid 4x4 world-to-TCP transform")
        q = checked_joint_target(self.backend, q_seed)
        limits = self.backend.limits
        for iteration in range(self.max_iterations + 1):
            error = pose_error(target, self.backend.fk(q))
            if np.linalg.norm(error[:3]) <= 1e-5 and np.linalg.norm(error[3:]) <= 1e-4:
                return q
            if iteration == self.max_iterations:
                break
            jacobian = self.backend.jacobian(q)
            u, singular, vt = np.linalg.svd(jacobian, full_matrices=False)
            delta = vt.T @ ((singular / (singular**2 + 1e-6)) * (u.T @ error))
            delta *= min(1.0, 0.15 / max(np.max(np.abs(delta)), 1e-12))
            q = np.clip(q + delta, limits.lower + limits.margin, limits.upper - limits.margin)
        return None


class PandaSimulation:
    def __init__(self, *, duration=18.0, width=960, height=640, control_mode="torque", ik_solver=None,
                 smoothing="none", max_jerk=30., differential_ik="dls", nullspace_gain=0.,
                 joint_centering_gain=0., external_targets=False, interactive_target=False, recorder=None):
        if duration is None and not interactive_target:
            raise ValueError("duration=None requires interactive_target")
        if duration is not None and (not np.isfinite(duration) or duration < 6):
            raise ValueError("duration must be at least 6 seconds")
        if control_mode not in CONTROL_MODES:
            raise ValueError(f"control_mode must be one of {CONTROL_MODES}")
        if ik_solver is not None and control_mode != "ik-position":
            raise ValueError("ik_solver is only used with ik-position")
        if smoothing not in ("none", "ruckig") or differential_ik not in ("dls", "qp"):
            raise ValueError("unknown smoothing or differential IK backend")
        if interactive_target and (external_targets or control_mode == "joint-position"):
            raise ValueError("interactive_target requires torque or ik-position and no external target stream")
        self.control_mode = control_mode
        self.actuator_mode = "torque" if control_mode == "torque" else "position"
        self.duration = None if duration is None else float(duration)
        self.interactive_target = interactive_target
        self.model = load_panda(width=width, height=height, actuator_mode=self.actuator_mode,
                                interactive_target=interactive_target)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.backend = PandaKinematics(self.model)
        self.arm_actuators = np.array([self.model.actuator(f"actuator{i}").id for i in range(1, 8)])
        self.data.ctrl[self.model.actuator("actuator8").id] = 255
        mujoco.mj_forward(self.model, self.data)
        self.home_q = self.data.qpos[self.backend.q_ids].copy()
        self.smoothing, self.differential_ik = smoothing, differential_ik
        self.external_targets, self.recorder = external_targets, recorder
        self.targets = LatestCommand()
        self.last_input_error = None
        self.servo = Servo(self.backend, ServoConfig(
            position_gain=10.0, orientation_gain=10.0, max_linear_speed=0.25,
            joint_position_gain=10.0, joint_position_tolerance=1e-5,
            nullspace_gain=nullspace_gain, nullspace_reference=self.home_q,
            joint_centering_gain=joint_centering_gain,
        ), differential_ik=BoxQPSolver() if differential_ik == "qp" else None,
            motion_generator=RuckigSmoothing(max_jerk) if smoothing == "ruckig" else None)
        self.home_pose = self.backend.fk(self.home_q)
        self.target = self.home_pose.copy()
        self.mocap_id = self.model.body_mocapid[self.model.body("target").id]
        self.data.mocap_pos[self.mocap_id] = self.target[:3, 3]
        mujoco.mju_mat2Quat(self.data.mocap_quat[self.mocap_id], self.target[:3, :3].ravel())
        mujoco.mj_forward(self.model, self.data)
        self.q_reference = self.data.qpos[self.backend.q_ids].copy()
        self.dq_reference = self.data.qvel[self.backend.v_ids].copy()
        self.kp = np.array([600, 600, 500, 500, 250, 200, 150])
        self.kd = np.array([50, 50, 40, 40, 20, 20, 12])
        self.joint_target = None
        self.ik_solver = (PandaPositionIK(self.backend) if ik_solver is None else ik_solver) if control_mode == "ik-position" else None
        self.ik_adapter = PositionIKAdapter(
            self.servo, lambda target, seed: self.ik_solver(target, seed), max_joint_step=0.35,
            position_tolerance=1e-4, orientation_tolerance=1e-3,
        ) if control_mode == "ik-position" else None
        self.ik_failures = 0
        self.last_ik_error = None
        self.tick = 0
        self.result = None
        # Interactive sessions can run indefinitely. Keep a recent visual trail
        # while accumulating metrics over the entire session.
        self.history = deque(maxlen=2000) if interactive_target else []
        self.trace = deque(maxlen=2000) if interactive_target else []
        self._squared_error_sum = 0.0
        self._max_position_error = 0.0
        self._max_tracking_error = 0.0
        self.actions = Counter()
        self.flags = SafetyFlag.NONE
        self.planned_path = np.empty((0, 3)) if external_targets or interactive_target else np.array([
            self.desired_pose(t)[:3, 3] for t in np.linspace(1, duration - 3, 181)])

    @property
    def time(self):
        return self.tick * CONTROL_DT

    def desired_pose(self, t):
        if self.interactive_target:
            pose = np.eye(4)
            pose[:3, 3] = self.data.mocap_pos[self.mocap_id]
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, self.data.mocap_quat[self.mocap_id])
            pose[:3, :3] = rotation.reshape(3, 3)
            return pose
        if self.external_targets:
            return self.target.copy()
        if self.control_mode == "joint-position":
            # FK is for visualization/metrics only; the control target is q.
            return self.backend.fk(target_joints(self.home_q, t, self.duration))
        return target_pose(self.home_pose, t, self.duration)

    def command(self, now_ns):
        if self.duration is not None and self.time >= self.duration - 1:
            return StopCommand()
        if self.external_targets:
            command = self.targets.read()
            if isinstance(command, StopCommand):
                return command
            if isinstance(command, PoseCommand):
                pose = np.asarray(command.pose, dtype=float)
                if pose.shape == (4, 4) and np.all(np.isfinite(pose)):
                    self.target = pose.copy()
                if self.control_mode == "ik-position":
                    return self._prepare_ik(command, now_ns)
                if self.control_mode == "torque":
                    return command
            elif isinstance(command, JointPositionCommand) and self.control_mode != "ik-position":
                # Native Servo performs full names/limits validation. Only
                # valid targets are used to move the visualization marker.
                native = self.servo._command(command, now_ns)
                if native.valid and len(native.joint_position) == 7 and np.all(np.isfinite(native.joint_position)):
                    self.target = self.backend.fk(native.joint_position)
                return command
            self.last_input_error = "target type does not match the selected control mode"
            return StopCommand()
        if self.control_mode == "torque":
            return PoseCommand(self.target, now_ns)
        if self.control_mode == "joint-position":
            self.joint_target = checked_joint_target(self.backend, target_joints(self.home_q, self.time, self.duration))
            return JointPositionCommand(self.joint_target, now_ns)
        return self._prepare_ik(PoseCommand(self.target, now_ns), now_ns)

    def reset_target_to_tcp(self):
        """Move the interactive handle to measured TCP without resetting physics or Servo."""
        if not self.interactive_target:
            raise ValueError("reset_target_to_tcp requires interactive_target")
        self.target = self.backend.fk(self.data.qpos[self.backend.q_ids])
        self.data.mocap_pos[self.mocap_id] = self.target[:3, 3]
        mujoco.mju_mat2Quat(self.data.mocap_quat[self.mocap_id], self.target[:3, :3].ravel())
        mujoco.mj_forward(self.model, self.data)

    def _prepare_ik(self, command, now_ns):
        prepared = self.ik_adapter.solve(command, self.q_reference, now_ns=now_ns)
        if not prepared.success:
            self.joint_target = None
            self.ik_failures += 1
            self.last_ik_error = prepared.message
        else:
            self.joint_target = prepared.command.positions.copy()
        return prepared.command

    def step(self):
        now_ns = self.tick * 10_000_000  # Simulation time, independent of rendering/wall time.
        self.target = self.desired_pose(self.time)
        state = JointState(self.data.qpos[self.backend.q_ids].copy(), self.data.qvel[self.backend.v_ids].copy(), now_ns)
        command = self.command(now_ns)
        self.result = self.servo.step(state, command, dt=CONTROL_DT, now_ns=now_ns)
        if self.recorder is not None:
            self.recorder.record(now_ns, CONTROL_DT, state, command, self.result)
        if self.result.action == Action.REJECT:
            # Zero is a torque-off command, but would be a dangerous angle
            # target for position actuators. Abort simulation without stepping.
            bounds = self.model.actuator_ctrlrange[self.arm_actuators]
            self.data.ctrl[self.arm_actuators] = 0 if self.actuator_mode == "torque" else np.clip(state.q, bounds[:, 0], bounds[:, 1])
            raise RuntimeError(f"Servo rejected at {self.time:.3f}s: {self.result.message} ({self.result.flags})")
        reference = self.result.reference
        self.actions[self.result.action.name] += 1
        self.flags |= self.result.flags
        if not self.interactive_target:
            # In interactive mode mocap is input owned by the viewer. Never
            # overwrite a dragged handle with the previous/generated target.
            self.data.mocap_pos[self.mocap_id] = self.target[:3, 3]
            mujoco.mju_mat2Quat(self.data.mocap_quat[self.mocap_id], self.target[:3, :3].ravel())
        # Sample the actual reference interval, including Ruckig jerk phases.
        # qpos/qvel are never overwritten after initialization: motors drive mj_step.
        for substep in range(round(CONTROL_DT / PHYSICS_DT)):
            elapsed = substep * PHYSICS_DT
            sample = self.servo.sample_reference(elapsed)
            q_des, dq_des = sample.q, sample.dq
            if self.actuator_mode == "position":
                control = q_des  # Radians into the model's bounded position servos.
            else:
                control = (
                    self.data.qfrc_bias[self.backend.v_ids]
                    + self.kp * (q_des - self.data.qpos[self.backend.q_ids])
                    + self.kd * (dq_des - self.data.qvel[self.backend.v_ids])
                )
            bounds = self.model.actuator_ctrlrange[self.arm_actuators]
            self.data.ctrl[self.arm_actuators] = np.clip(control, bounds[:, 0], bounds[:, 1])
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.q_reference, self.dq_reference = reference.q, reference.dq
        self.tick += 1
        actual = self.data.site_xpos[self.backend.site_id].copy()
        desired = self.desired_pose(self.time)[:3, 3]
        error = float(np.linalg.norm(actual - desired))
        self.trace.append(actual)
        self.history.append((self.time, error, self.result.diagnostics.tracking_error))
        self._squared_error_sum += error * error
        self._max_position_error = max(self._max_position_error, error)
        self._max_tracking_error = max(self._max_tracking_error, self.result.diagnostics.tracking_error)
        return self.result

    def summary(self):
        return {
            "mujoco_version": mujoco.__version__, "physics_hz": 500, "servo_hz": 100,
            "control_mode": self.control_mode, "actuator_mode": self.actuator_mode,
            "smoothing": self.smoothing, "differential_ik": self.differential_ik,
            "external_targets": self.external_targets, "last_input_error": self.last_input_error,
            "interactive_target": self.interactive_target,
            "actuator_command_units": "Nm" if self.actuator_mode == "torque" else "rad",
            "ik_failures": self.ik_failures, "last_ik_error": self.last_ik_error,
            "simulated_seconds": self.time, "steps": self.tick, "feedback": "MuJoCo qpos/qvel",
            "position_rmse_m": float(np.sqrt(self._squared_error_sum / self.tick)) if self.tick else None,
            "position_max_error_m": float(self._max_position_error) if self.tick else None,
            "final_position_error_m": float(self.history[-1][1]) if self.history else None,
            "max_joint_tracking_error_rad": float(self._max_tracking_error) if self.tick else None,
            "final_action": self.result.action.name if self.result else None,
            "actions": dict(self.actions), "flags": [flag.name for flag in SafetyFlag if flag & self.flags],
            "servo_collision_monitor": "disabled",
            "trajectory": "interactive viewer target" if self.interactive_target else "external targets" if self.external_targets else (
                "joint-space loop (TCP path from FK)" if self.control_mode == "joint-position" else "figure eight, fixed TCP orientation"),
        }


def camera():
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.30, 0.0, 0.43]
    cam.distance, cam.azimuth, cam.elevation = 1.85, 135, -24
    return cam


def select_target(viewer, simulation):
    """Call under viewer.lock(); select the mocap handle for native Ctrl-drag."""
    perturb = viewer.perturb
    perturb.active = perturb.active2 = 0
    perturb.select = simulation.model.body("target").id
    perturb.localpos[:] = 0
    perturb.refpos[:] = simulation.data.mocap_pos[simulation.mocap_id]
    perturb.refquat[:] = simulation.data.mocap_quat[simulation.mocap_id]


def draw_paths(scene, simulation, *, clear=False):
    if clear:
        scene.ngeom = 0
    # Desired path is amber; actual TCP trail is cyan. Bounded scene geometry.
    for points, radius, color in [
        (simulation.planned_path[::2], 0.0018, [1.0, 0.58, 0.23, 0.40]),
        (np.asarray(simulation.trace)[::max(1, len(simulation.trace) // 200)], 0.003, [0.10, 0.85, 0.94, 1.0]),
    ]:
        for start, end in zip(points[:-1], points[1:]):
            if scene.ngeom >= scene.maxgeom:
                return
            if np.linalg.norm(end - start) < 1e-7:
                continue
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, start, end)
            scene.ngeom += 1


def annotate(frame, simulation):
    from PIL import Image, ImageDraw, ImageFont
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, "RGBA")
    try:
        title_font = ImageFont.truetype("DejaVuSans.ttf", 24)
        font = ImageFont.truetype("DejaVuSans.ttf", 16)
    except OSError:
        title_font = font = ImageFont.load_default()
    draw.rectangle((0, 0, image.width, 80), fill=(10, 18, 28, 220))
    draw.text((24, 13), "ServoPy  /  Panda in MuJoCo", fill="white", font=title_font)
    draw.text((24, 49), f"{simulation.control_mode}  |  100 Hz Servo  |  500 Hz physics", fill=(184, 205, 220), font=font)
    error = simulation.history[-1][1] * 1000 if simulation.history else 0
    action = simulation.result.action.name if simulation.result else "READY"
    draw.rectangle((0, image.height - 58, image.width, image.height), fill=(10, 18, 28, 220))
    draw.text((24, image.height - 48), f"t = {simulation.time:5.2f} s    TCP error = {error:5.1f} mm    {action}", fill="white", font=font)
    legend = "AMBER draggable target     CYAN measured TCP trail" if simulation.interactive_target else "AMBER target path     CYAN measured TCP trail"
    draw.text((24, image.height - 25), legend, fill=(78, 216, 233), font=font)
    return np.asarray(image)


@contextmanager
def passive_viewer(model, data, key_callback):
    from mujoco import viewer as mj_viewer

    # On Linux/Windows, launch_passive starts a daemon rendering thread, while
    # Handle.close() only requests exit. Join the threads created by this launch
    # before Python/GLFW teardown to avoid a race at interpreter shutdown.
    previous_threads = set(threading.enumerate())
    handle = mj_viewer.launch_passive(
        model, data, key_callback=key_callback, show_left_ui=False, show_right_ui=False
    )
    viewer_threads = set(threading.enumerate()) - previous_threads
    try:
        yield handle
    finally:
        handle.close()
        for thread in viewer_threads:
            thread.join()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-mode", choices=CONTROL_MODES, default="torque",
                        help="torque: Cartesian Servo + torque PD (default); joint-position: joint targets + position actuators; ik-position: Python IK + position actuators.")
    parser.add_argument("--headless", action="store_true", help="Disable the viewer; run as fast as possible.")
    parser.add_argument("--duration", type=float, default=None,
                        help="Simulation seconds, minimum 6. Default: 18; interactive target runs until closed.")
    parser.add_argument("--record", type=Path, help="Write an MP4 from actual simulation frames.")
    parser.add_argument("--metrics", type=Path, help="Save the measured tracking summary as JSON.")
    parser.add_argument("--smoothing", choices=("none", "ruckig"), default="none")
    parser.add_argument("--max-jerk", type=float, default=30., help="rad/s^3 for the optional Ruckig backend.")
    parser.add_argument("--differential-ik", choices=("dls", "qp"), default="dls")
    parser.add_argument("--nullspace-gain", type=float, default=0., help="Posture gain towards Panda home joints.")
    parser.add_argument("--joint-centering-gain", type=float, default=0.)
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--interactive-target", action="store_true",
                         help="Drag the viewer target with Ctrl+right mouse; Ctrl+left rotates. Requires torque or ik-position; F6 resets target to TCP.")
    sources.add_argument("--target-stdin", action="store_true", help="Read live JSONL targets; pace headless physics in wall time.")
    sources.add_argument("--targets", type=Path, help="Replay JSONL targets with simulation time in a time field (seconds).")
    parser.add_argument("--log", type=Path, help="Record feedback, accepted commands and reference endpoints as JSONL.")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args(argv)
    if args.duration is None and not args.interactive_target:
        args.duration = 18.0
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration < 6):
        parser.error("--duration must be at least 6 seconds")
    if args.interactive_target and args.headless:
        parser.error("--interactive-target requires the viewer; remove --headless")
    if args.interactive_target and args.control_mode == "joint-position":
        parser.error("--interactive-target is a pose target; use --control-mode torque or ik-position")
    if args.width < 320 or args.height < 240 or args.width % 2 or args.height % 2:
        parser.error("Recording dimensions must be even, at least 320 x 240")
    if not 1 <= args.fps <= 100:
        parser.error("--fps must be between 1 and 100")
    if args.record and args.record.suffix.lower() != ".mp4":
        parser.error("--record expects an .mp4 filename")
    if not args.headless and sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("No display found. Run on your desktop for the viewer, or use --headless (MUJOCO_GL=egl when recording).")
    if args.control_mode != "torque" and (args.differential_ik != "dls" or args.nullspace_gain or args.joint_centering_gain):
        parser.error("Differential IK and nullspace options apply to torque/Pose mode; ik-position uses the position IK adapter.")
    scheduled = []
    if args.targets:
        previous = -1.
        for number, line in enumerate(args.targets.read_text().splitlines(), 1):
            try:
                row = json.loads(line)
                stamp = float(row.pop("time"))
                if not np.isfinite(stamp) or stamp < 0 or stamp < previous:
                    raise ValueError("time must be nonnegative and ordered")
                scheduled.append((stamp, command_from_dict(row, stamp_ns=round(stamp * 1e9))))
                previous = stamp
            except (ValueError, KeyError, TypeError) as exc:
                parser.error(f"Invalid target line {number}: {exc}")
    simulation = PandaSimulation(duration=args.duration, width=args.width, height=args.height, control_mode=args.control_mode,
        smoothing=args.smoothing, max_jerk=args.max_jerk, differential_ik=args.differential_ik,
        nullspace_gain=args.nullspace_gain, joint_centering_gain=args.joint_centering_gain,
        external_targets=args.target_stdin or args.targets is not None,
        interactive_target=args.interactive_target)
    paused = threading.Event()
    reset_target = threading.Event()

    def key_callback(key):
        if key == 32:
            paused.clear() if paused.is_set() else paused.set()
        elif key == 295 and args.interactive_target:  # GLFW_KEY_F6; avoids MuJoCo's letter shortcuts.
            reset_target.set()  # Only the simulation thread touches physics.

    with ExitStack() as stack:
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            simulation.recorder = stack.enter_context(JsonlRecorder(args.log))
        if args.target_stdin:
            def read_targets():
                try:
                    for line in sys.stdin:
                        try:
                            if len(line) > 65536:
                                raise ValueError("target line exceeds 64 KiB")
                            command = command_from_dict(json.loads(line), stamp_ns=simulation.tick * 10_000_000)
                            simulation.targets.publish(command)
                        except (ValueError, TypeError) as exc:
                            simulation.last_input_error = str(exc)
                            simulation.targets.clear()
                finally:
                    simulation.targets.clear()  # EOF/disconnect means stop.
            threading.Thread(target=read_targets, name="panda-target-input", daemon=True).start()
        viewer = None
        if not args.headless:
            viewer = stack.enter_context(passive_viewer(simulation.model, simulation.data, key_callback))
            cam = camera()
            with viewer.lock():
                viewer.cam.lookat[:] = cam.lookat
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = cam.distance, cam.azimuth, cam.elevation
                if args.interactive_target:
                    select_target(viewer, simulation)
            print("Viewer running. Space: pause/resume. Close window or Ctrl+C: exit.", flush=True)
            if args.interactive_target:
                print("Target selected. Ctrl+right drag: move; Ctrl+left drag: rotate; Shift: change plane/axis.\n"
                      "Double-click the amber handle to select it again. F6: move target to current TCP.\n"
                      "Drag gradually within reach; the arm follows subject to limits and IK feasibility.", flush=True)
                if hasattr(viewer, "set_texts"):
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                                      "ServoPy | Interactive target\nCtrl+right: move | Ctrl+left: rotate\n"
                                      "Shift: change plane | F6: target to TCP\nSpace: pause / resume", ""))
        renderer = writer = None
        if args.record:
            import imageio.v2 as imageio
            args.record.parent.mkdir(parents=True, exist_ok=True)
            renderer = stack.enter_context(mujoco.Renderer(simulation.model, height=args.height, width=args.width))
            writer = stack.enter_context(imageio.get_writer(str(args.record), fps=args.fps, codec="libx264", quality=8, macro_block_size=1))
        render_cam = camera()
        next_frame, next_view, next_tick = 0.0, 0.0, time.perf_counter()
        target_index = 0
        try:
            while (args.duration is None or simulation.time < args.duration - 1e-9) and (viewer is None or viewer.is_running()):
                if args.interactive_target:
                    # sync applies native mouse perturbations to mocap state.
                    # Read that state in this cycle before advancing physics.
                    viewer.sync()
                    if reset_target.is_set():
                        with viewer.lock():
                            simulation.reset_target_to_tcp()
                            select_target(viewer, simulation)
                        reset_target.clear()
                if paused.is_set():
                    viewer.sync()
                    time.sleep(0.02)
                    next_tick = time.perf_counter()
                    continue
                if writer is not None and simulation.time + 1e-9 >= next_frame:
                    renderer.update_scene(simulation.data, camera=render_cam)
                    draw_paths(renderer.scene, simulation)
                    writer.append_data(annotate(renderer.render(), simulation))
                    next_frame += 1 / args.fps
                with viewer.lock() if viewer is not None else nullcontext():
                    while target_index < len(scheduled) and scheduled[target_index][0] <= simulation.time + 1e-9:
                        simulation.targets.publish(scheduled[target_index][1])
                        target_index += 1
                    simulation.step()
                    if viewer is not None and simulation.time >= next_view:
                        draw_paths(viewer.user_scn, simulation, clear=True)
                if viewer is not None:
                    if simulation.time >= next_view:
                        if not args.interactive_target:
                            viewer.sync()
                        next_view += 1 / 30
                if viewer is not None or args.target_stdin:
                    next_tick += CONTROL_DT
                    time.sleep(max(0.0, next_tick - time.perf_counter()))
        except KeyboardInterrupt:
            print("Demo interrupted; closing viewer and recording.", file=sys.stderr)
    summary = simulation.summary()
    summary["completed"] = args.duration is not None and simulation.time >= args.duration - 1e-9
    summary["recording"] = {"fps": args.fps, "width": args.width, "height": args.height} if args.record else None
    print(json.dumps(summary, indent=2))
    if args.metrics:
        args.metrics.parent.mkdir(parents=True, exist_ok=True)
        args.metrics.write_text(json.dumps(summary, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
