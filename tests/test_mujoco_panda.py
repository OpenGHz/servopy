"""Optional physics integration checks; these do not require a display/GPU."""
import importlib.util

import numpy as np
import pytest

from conftest import ROOT
from servo_py import Action

mujoco = pytest.importorskip("mujoco")
spec = importlib.util.spec_from_file_location("mujoco_panda", ROOT / "examples/mujoco_panda.py")
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


@pytest.fixture(scope="module")
def model():
    return demo.load_panda()


def test_panda_tcp_jacobian_matches_pose_derivatives(model):
    backend = demo.PandaKinematics(model)
    q = np.array([0.2, -0.5, -0.2, -1.8, 0.3, 1.7, 0.3])
    rotation = backend.fk(q)[:3, :3]
    jacobian = backend.jacobian(q)
    numeric = np.empty((6, 7))
    epsilon = 1e-6
    for i in range(7):
        offset = np.eye(7)[i] * epsilon
        plus, minus = backend.fk(q + offset), backend.fk(q - offset)
        numeric[:3, i] = (plus[:3, 3] - minus[:3, 3]) / (2 * epsilon)
        omega = ((plus[:3, :3] - minus[:3, :3]) / (2 * epsilon)) @ rotation.T
        numeric[3:, i] = omega[2, 1], omega[0, 2], omega[1, 0]
    np.testing.assert_allclose(jacobian, numeric, atol=2e-9)


def test_kinematics_probes_preserve_live_physics_state(model):
    live = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, live, model.key("home").id)
    live.qvel[:7] = np.linspace(-0.1, 0.1, 7)
    live.ctrl[:7] = np.linspace(-1, 1, 7)
    mujoco.mj_step(model, live)
    fields = ("qpos", "qvel", "ctrl", "site_xpos", "qfrc_bias")
    before = {name: getattr(live, name).copy() for name in fields}
    before_time = live.time
    backend = demo.PandaKinematics(model)
    for q in ([0.2, -0.5, -0.2, -1.8, 0.3, 1.7, 0.3], live.qpos[:7]):
        backend.fk(q)
        backend.jacobian(q)
    for name in fields:
        np.testing.assert_array_equal(getattr(live, name), before[name])
    assert live.time == before_time


def test_panda_tracks_with_physics_feedback_and_finishes_holding():
    simulation = demo.PandaSimulation()
    limits = simulation.backend.limits
    tracking_errors = []
    for _ in range(1800):
        result = simulation.step()
        assert result.action != Action.REJECT, result.message
        ref = result.reference
        assert np.all(np.abs(ref.dq) <= limits.velocity + 1e-10)
        assert np.all(np.abs(ref.ddq) <= limits.acceleration + 1e-10)
        assert np.all(ref.q >= limits.lower + limits.margin - 1e-10)
        assert np.all(ref.q <= limits.upper - limits.margin + 1e-10)
        measured = simulation.data.qpos[simulation.backend.q_ids]
        tracking_errors.append(np.max(np.abs(measured - ref.q)))
        assert np.all(measured >= limits.lower)
        assert np.all(measured <= limits.upper)
    summary = simulation.summary()
    assert simulation.data.time == pytest.approx(18.0)
    assert summary["position_max_error_m"] < 0.025
    assert summary["final_position_error_m"] < 0.001
    assert simulation.result.action == Action.HOLD
    assert np.max(np.linalg.norm(np.asarray(simulation.trace) - simulation.home_pose[:3, 3], axis=1)) > 0.10
    # Dynamics must have actually integrated: measured q is not ideal replay.
    assert 1e-5 < max(tracking_errors) < 0.02
