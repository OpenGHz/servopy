"""Optional physics integration checks; these do not require a display/GPU."""
import importlib.util
from dataclasses import replace

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


@pytest.mark.parametrize("mode,max_tcp_error,final_tcp_error,max_tracking", [
    ("torque", 0.025, 0.001, 0.02),
    ("joint-position", 0.04, 0.008, 0.04),
    ("ik-position", 0.045, 0.008, 0.06),
])
def test_panda_tracks_with_physics_feedback_and_finishes_holding(mode, max_tcp_error, final_tcp_error, max_tracking, monkeypatch):
    simulation = demo.PandaSimulation(control_mode=mode)
    if mode == "joint-position":
        def unexpected_ik(*args):
            pytest.fail("Direct joint targets must not invoke IK or a Jacobian")
        simulation.ik_solver = unexpected_ik
        monkeypatch.setattr(simulation.backend, "jacobian", unexpected_ik)
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
        forces = simulation.data.actuator_force[simulation.arm_actuators]
        bounds = simulation.model.actuator_forcerange[simulation.arm_actuators]
        assert np.all(forces >= bounds[:, 0] - 1e-10)
        assert np.all(forces <= bounds[:, 1] + 1e-10)
    summary = simulation.summary()
    assert simulation.data.time == pytest.approx(18.0)
    assert summary["position_max_error_m"] < max_tcp_error
    assert summary["final_position_error_m"] < final_tcp_error
    assert summary["control_mode"] == mode
    assert summary["ik_failures"] == 0
    assert simulation.result.action == Action.HOLD
    assert np.max(np.linalg.norm(np.asarray(simulation.trace) - simulation.home_pose[:3, 3], axis=1)) > 0.10
    # Dynamics must have actually integrated: measured q is not ideal replay.
    assert 1e-5 < max(tracking_errors) < max_tracking


def test_position_actuators_accept_angles_and_generate_restoring_force():
    simulation = demo.PandaSimulation(control_mode="joint-position")
    arm = simulation.arm_actuators
    np.testing.assert_array_equal(simulation.data.ctrl[arm], simulation.home_q)
    # A 1 mrad target offset produces kp * error, not a 1 mNm torque input.
    simulation.data.ctrl[arm] += 0.001
    mujoco.mj_forward(simulation.model, simulation.data)
    np.testing.assert_allclose(
        simulation.data.actuator_force[arm],
        0.001 * simulation.model.actuator_gainprm[arm, 0], atol=1e-9,
    )
    assert np.min(simulation.data.actuator_force[arm]) > 1.0


def test_position_ik_solves_full_pose_and_returns_none_for_unreachable_target(model):
    backend = demo.PandaKinematics(model)
    solver = demo.PandaPositionIK(backend)
    seed = np.array([0.2, -0.5, -0.2, -1.8, 0.3, 1.7, 0.3])
    target = backend.fk(seed + [0.1, -0.1, 0.05, -0.1, 0.05, 0.1, 0.05])
    original_seed, original_target = seed.copy(), target.copy()
    solution = solver(target, seed)
    assert solution is not None
    actual = backend.fk(solution)
    np.testing.assert_allclose(actual[:3, 3], target[:3, 3], atol=1e-5)
    np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=1e-4)
    assert np.all(solution >= backend.limits.lower + backend.limits.margin)
    assert np.all(solution <= backend.limits.upper - backend.limits.margin)
    np.testing.assert_array_equal(seed, original_seed)
    np.testing.assert_array_equal(target, original_target)
    target[:3, 3] = [10, 10, 10]
    assert solver(target, seed) is None


@pytest.mark.parametrize("failure", ["none", "nan", "limit", "branch", "residual", "exception"])
def test_bad_external_ik_brakes_and_holds_instead_of_using_stale_target(failure):
    simulation = demo.PandaSimulation(control_mode="ik-position")
    for _ in range(400):
        simulation.step()
    assert np.max(np.abs(simulation.dq_reference)) > 0.01
    seed = simulation.q_reference.copy()

    def broken_solver(target, q_seed):
        if failure == "none":
            return None
        if failure == "nan":
            return np.full(7, np.nan)
        if failure == "limit":
            return np.full(7, 100.0)
        if failure == "branch":
            return q_seed + [0.5, 0, 0, 0, 0, 0, 0]
        if failure == "residual":
            return seed  # Valid joints, but does not solve the current pose.
        raise RuntimeError("external IK failed")

    simulation.ik_solver = broken_solver
    for _ in range(100):
        result = simulation.step()
        assert result.action in (Action.BRAKE, Action.HOLD)
        assert np.all(np.abs(result.reference.ddq) <= simulation.backend.limits.acceleration + 1e-10)
        assert simulation.joint_target is None
    assert result.action == Action.HOLD
    assert simulation.ik_failures == 100
    assert simulation.last_ik_error
    assert simulation.data.time == pytest.approx(5.0)


def test_rejected_position_control_does_not_command_zero_angles(monkeypatch):
    simulation = demo.PandaSimulation(control_mode="joint-position")
    result = simulation.step()
    now = simulation.data.time
    measured = simulation.data.qpos[simulation.backend.q_ids].copy()
    monkeypatch.setattr(simulation.servo, "step", lambda *args, **kwargs: replace(result, action=Action.REJECT, reference=None))
    with pytest.raises(RuntimeError, match="Servo rejected"):
        simulation.step()
    assert simulation.data.time == now
    np.testing.assert_array_equal(simulation.data.ctrl[simulation.arm_actuators], measured)
