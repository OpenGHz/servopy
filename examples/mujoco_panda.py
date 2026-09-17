"""Panda Cartesian servoing with real MuJoCo dynamics and a default GUI viewer.

Run from a source checkout: python examples/mujoco_panda.py
Headless recording: MUJOCO_GL=egl python examples/mujoco_panda.py --headless --record demo.mp4
"""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack, contextmanager, nullcontext
import hashlib
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
    raise ImportError("Install the demo dependencies: python -m pip install '.[mujoco]'") from exc

from servo_py import (
    Action, JointLimits, JointState, Kinematics, PoseCommand,
    SafetyFlag, Servo, ServoConfig, StopCommand,
)

ASSETS = Path(__file__).resolve().parent / "assets"
CONTROL_DT = 0.01
PHYSICS_DT = 0.002
ARM_NAMES = tuple(f"joint{i}" for i in range(1, 8))


def load_panda(*, width=960, height=640):
    """Load the pinned, unmodified Menagerie assets from an in-memory ZIP.

    Scene and actuator changes below are specific to this example. The source
    MJCF, original meshes, license and per-file hashes remain in examples/assets.
    """
    archive_path = ASSETS / "panda.zip"
    manifest = json.loads((ASSETS / "panda-source.json").read_text())
    if hashlib.sha256(archive_path.read_bytes()).hexdigest() != manifest["archive_sha256"]:
        raise ValueError("Panda asset archive checksum mismatch")
    with zipfile.ZipFile(archive_path) as archive:
        assets = {name: archive.read(name) for name in archive.namelist()}
    root = ET.fromstring(assets["panda.xml"])
    root.find("option").set("timestep", str(PHYSICS_DT))
    # Replace the seven upstream position actuators with bounded torque inputs.
    # The finger tendon retains the upstream position actuator.
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
    ET.SubElement(target, "geom", type="sphere", size="0.015", rgba="1 0.55 0.16 0.95", contype="0", conaffinity="0")
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


def target_pose(home_pose, t, duration):
    """One smooth 3D figure eight, followed by settling and a stop segment."""
    u = np.clip((t - 1.0) / (duration - 4.0), 0.0, 1.0)
    phase = 2 * np.pi * (10 * u**3 - 15 * u**4 + 6 * u**5)
    pose = home_pose.copy()
    pose[:3, 3] += [-0.045 * (1 - np.cos(phase)), 0.14 * np.sin(phase), 0.08 * np.sin(2 * phase)]
    return pose


class PandaSimulation:
    def __init__(self, *, duration=18.0, width=960, height=640):
        if not np.isfinite(duration) or duration < 6:
            raise ValueError("duration must be at least 6 seconds")
        self.duration = float(duration)
        self.model = load_panda(width=width, height=height)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.backend = PandaKinematics(self.model)
        self.arm_actuators = np.array([self.model.actuator(f"actuator{i}").id for i in range(1, 8)])
        self.data.ctrl[self.model.actuator("actuator8").id] = 255
        mujoco.mj_forward(self.model, self.data)
        self.servo = Servo(self.backend, ServoConfig(position_gain=10.0, orientation_gain=10.0, max_linear_speed=0.25))
        self.home_pose = self.backend.fk(self.data.qpos[self.backend.q_ids])
        self.target = self.home_pose.copy()
        self.mocap_id = self.model.body_mocapid[self.model.body("target").id]
        self.data.mocap_pos[self.mocap_id] = self.target[:3, 3]
        self.q_reference = self.data.qpos[self.backend.q_ids].copy()
        self.dq_reference = self.data.qvel[self.backend.v_ids].copy()
        self.kp = np.array([600, 600, 500, 500, 250, 200, 150])
        self.kd = np.array([50, 50, 40, 40, 20, 20, 12])
        self.tick = 0
        self.result = None
        self.history = []
        self.trace = []
        self.actions = Counter()
        self.flags = SafetyFlag.NONE
        self.planned_path = np.array([target_pose(self.home_pose, t, duration)[:3, 3] for t in np.linspace(1, duration - 3, 181)])

    @property
    def time(self):
        return self.tick * CONTROL_DT

    def step(self):
        now_ns = self.tick * 10_000_000  # Simulation time, independent of rendering/wall time.
        self.target = target_pose(self.home_pose, self.time, self.duration)
        state = JointState(self.data.qpos[self.backend.q_ids].copy(), self.data.qvel[self.backend.v_ids].copy(), now_ns)
        command = StopCommand() if self.time >= self.duration - 1 else PoseCommand(self.target, now_ns)
        self.result = self.servo.step(state, command, dt=CONTROL_DT, now_ns=now_ns)
        if self.result.action == Action.REJECT:
            self.data.ctrl[self.arm_actuators] = 0
            raise RuntimeError(f"Servo rejected at {self.time:.3f}s: {self.result.message} ({self.result.flags})")
        reference = self.result.reference
        self.actions[self.result.action.name] += 1
        self.flags |= self.result.flags
        self.data.mocap_pos[self.mocap_id] = self.target[:3, 3]
        # Interpolate the constant-acceleration interval defined by Servo.
        # qpos/qvel are never overwritten after initialization: motors drive mj_step.
        for substep in range(round(CONTROL_DT / PHYSICS_DT)):
            elapsed = substep * PHYSICS_DT
            q_des = self.q_reference + elapsed * self.dq_reference + 0.5 * elapsed**2 * reference.ddq
            dq_des = self.dq_reference + elapsed * reference.ddq
            torque = (
                self.data.qfrc_bias[self.backend.v_ids]
                + self.kp * (q_des - self.data.qpos[self.backend.q_ids])
                + self.kd * (dq_des - self.data.qvel[self.backend.v_ids])
            )
            bounds = self.model.actuator_ctrlrange[self.arm_actuators]
            self.data.ctrl[self.arm_actuators] = np.clip(torque, bounds[:, 0], bounds[:, 1])
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.q_reference, self.dq_reference = reference.q, reference.dq
        self.tick += 1
        actual = self.data.site_xpos[self.backend.site_id].copy()
        desired = target_pose(self.home_pose, self.time, self.duration)[:3, 3]
        error = float(np.linalg.norm(actual - desired))
        self.trace.append(actual)
        self.history.append((self.time, error, self.result.diagnostics.tracking_error))
        return self.result

    def summary(self):
        history = np.asarray(self.history)
        return {
            "mujoco_version": mujoco.__version__, "physics_hz": 500, "servo_hz": 100,
            "simulated_seconds": self.time, "steps": self.tick, "feedback": "MuJoCo qpos/qvel",
            "position_rmse_m": float(np.sqrt(np.mean(history[:, 1] ** 2))) if self.history else None,
            "position_max_error_m": float(np.max(history[:, 1])) if self.history else None,
            "final_position_error_m": float(history[-1, 1]) if self.history else None,
            "max_joint_tracking_error_rad": float(np.max(history[:, 2])) if self.history else None,
            "final_action": self.result.action.name if self.result else None,
            "actions": dict(self.actions), "flags": [flag.name for flag in SafetyFlag if flag & self.flags],
            "servo_collision_monitor": "disabled", "trajectory": "figure eight, fixed TCP orientation",
        }


def camera():
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.30, 0.0, 0.43]
    cam.distance, cam.azimuth, cam.elevation = 1.85, 135, -24
    return cam


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
    draw.text((24, 13), "servo-py  /  Panda in MuJoCo", fill="white", font=title_font)
    draw.text((24, 49), "100 Hz Servo  |  500 Hz physics  |  Measured joint feedback", fill=(184, 205, 220), font=font)
    error = simulation.history[-1][1] * 1000 if simulation.history else 0
    action = simulation.result.action.name if simulation.result else "READY"
    draw.rectangle((0, image.height - 58, image.width, image.height), fill=(10, 18, 28, 220))
    draw.text((24, image.height - 48), f"t = {simulation.time:5.2f} s    TCP error = {error:5.1f} mm    {action}", fill="white", font=font)
    draw.text((24, image.height - 25), "AMBER target path     CYAN measured TCP trail", fill=(78, 216, 233), font=font)
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
    parser.add_argument("--headless", action="store_true", help="Disable the viewer; run as fast as possible.")
    parser.add_argument("--duration", type=float, default=18, help="Simulation seconds, minimum 6 (default: 18).")
    parser.add_argument("--record", type=Path, help="Write an MP4 from actual simulation frames.")
    parser.add_argument("--metrics", type=Path, help="Save the measured tracking summary as JSON.")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args(argv)
    if not np.isfinite(args.duration) or args.duration < 6:
        parser.error("--duration must be at least 6 seconds")
    if args.width < 320 or args.height < 240 or args.width % 2 or args.height % 2:
        parser.error("Recording dimensions must be even, at least 320 x 240")
    if not 1 <= args.fps <= 100:
        parser.error("--fps must be between 1 and 100")
    if args.record and args.record.suffix.lower() != ".mp4":
        parser.error("--record expects an .mp4 filename")
    if not args.headless and sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("No display found. Run on your desktop for the viewer, or use --headless (MUJOCO_GL=egl when recording).")
    simulation = PandaSimulation(duration=args.duration, width=args.width, height=args.height)
    paused = threading.Event()

    def key_callback(key):
        if key == 32:
            paused.clear() if paused.is_set() else paused.set()

    with ExitStack() as stack:
        viewer = None
        if not args.headless:
            viewer = stack.enter_context(passive_viewer(simulation.model, simulation.data, key_callback))
            cam = camera()
            with viewer.lock():
                viewer.cam.lookat[:] = cam.lookat
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = cam.distance, cam.azimuth, cam.elevation
            print("Viewer running. Space: pause/resume. Close window or Ctrl+C: exit.", flush=True)
        renderer = writer = None
        if args.record:
            import imageio.v2 as imageio
            args.record.parent.mkdir(parents=True, exist_ok=True)
            renderer = stack.enter_context(mujoco.Renderer(simulation.model, height=args.height, width=args.width))
            writer = stack.enter_context(imageio.get_writer(str(args.record), fps=args.fps, codec="libx264", quality=8, macro_block_size=1))
        render_cam = camera()
        next_frame, next_view, next_tick = 0.0, 0.0, time.perf_counter()
        try:
            while simulation.time < args.duration - 1e-9 and (viewer is None or viewer.is_running()):
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
                    simulation.step()
                    if viewer is not None and simulation.time >= next_view:
                        draw_paths(viewer.user_scn, simulation, clear=True)
                if viewer is not None:
                    if simulation.time >= next_view:
                        viewer.sync()
                        next_view += 1 / 30
                    next_tick += CONTROL_DT
                    time.sleep(max(0.0, next_tick - time.perf_counter()))
        except KeyboardInterrupt:
            print("Demo interrupted; closing viewer and recording.", file=sys.stderr)
    summary = simulation.summary()
    summary["completed"] = simulation.time >= args.duration - 1e-9
    summary["recording"] = {"fps": args.fps, "width": args.width, "height": args.height} if args.record else None
    print(json.dumps(summary, indent=2))
    if args.metrics:
        args.metrics.parent.mkdir(parents=True, exist_ok=True)
        args.metrics.write_text(json.dumps(summary, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
