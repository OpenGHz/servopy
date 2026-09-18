import numpy as np
import pytest

from servo_py import (
    Action, CollisionSample, JointJogCommand, JointPositionCommand, JointState,
    SafetyFlag as F, Servo, ServoConfig, _core,
)
from conftest import make_sliders

DT, NS = .01, 10_000_000


def test_joint_position_converges_and_reverses_with_bounded_references():
    model = make_sliders(2, velocity=[.3, .5], acceleration=[.7, 1.1])
    servo = Servo(model, ServoConfig(task_axes=(0,), joint_position_gain=4))
    state = JointState([0., 0.], [0., 0.], 0)
    for k in range(1200):
        target = np.array([.6, -.5]) if k < 550 else np.array([-.3, .2])
        result = servo.step(state, JointPositionCommand(target, k * NS), dt=DT, now_ns=k * NS)
        assert result.action != Action.REJECT, result.message
        ref = result.reference
        assert (np.abs(ref.dq) <= model.limits.velocity + 1e-10).all()
        assert (np.abs(ref.ddq) <= model.limits.acceleration + 1e-10).all()
        for t in (0., DT / 2, DT):
            within = np.asarray(state.q) + t * np.asarray(state.dq) + .5 * t**2 * ref.ddq
            assert (np.abs(within) <= .99 + 1e-10).all()
        state = JointState(ref.q, ref.dq, ref.stamp_ns)
    np.testing.assert_allclose(state.q, target, atol=1.1e-4)
    assert result.action == Action.HOLD
    assert result.flags & F.GOAL_REACHED
    assert result.diagnostics.joint_position_error <= 1e-4


@pytest.mark.parametrize("command", [
    JointPositionCommand([np.nan], 0), JointPositionCommand([np.inf], 0),
    JointPositionCommand([.1, .2], 0), JointPositionCommand([[.1]], 0),
    JointPositionCommand([.995], 0), JointPositionCommand([-1.1], 0),
    JointPositionCommand([.1], 0, ["missing"]), JointPositionCommand([.1], NS),
])
def test_invalid_position_targets_brake_without_latching(slider, command):
    servo = Servo(slider, ServoConfig(task_axes=(0,)))
    result = servo.step(JointState([0.], [.2], 0), command, dt=DT, now_ns=0)
    assert result.action == Action.BRAKE and result.flags & F.INVALID_COMMAND
    assert 0 < result.reference.dq[0] < .2
    ref = result.reference
    result = servo.step(JointState(ref.q, ref.dq, NS), JointPositionCommand([.5], NS), dt=DT, now_ns=NS)
    assert result.action == Action.TRACK
    assert not result.flags & F.FAULT_LATCHED


def test_named_position_commands_require_all_joints():
    model = make_sliders(2)
    state = JointState([.2, -.1], [0, 0], 0)
    cfg = ServoConfig(task_axes=(0,))
    a = Servo(model, cfg).step(state, JointPositionCommand([.3, -.2], 0), dt=DT, now_ns=0)
    b = Servo(model, cfg).step(state, JointPositionCommand([-.2, .3], 0, ["j1", "j0"]), dt=DT, now_ns=0)
    np.testing.assert_array_equal(a.reference.q, b.reference.q)
    for positions, names in [([.3], ["j0"]), ([.3, -.2], ["j0", "j0"])]:
        invalid = Servo(model, cfg).step(state, JointPositionCommand(positions, 0, names), dt=DT, now_ns=0)
        assert invalid.flags & F.INVALID_COMMAND
        np.testing.assert_array_equal(invalid.reference.q, state.q)


def test_position_timeout_and_mode_change_brake_with_constraints(slider):
    servo = Servo(slider, ServoConfig(task_axes=(0,), command_timeout=.02))
    state = JointState([0.], [.2], 0)
    initial = servo.step(state, JointJogCommand([.2], 0), dt=DT, now_ns=0)
    ref = initial.reference
    for k in range(1, 30):
        result = servo.step(JointState(ref.q, ref.dq, k * NS), JointPositionCommand([.5], 0), dt=DT, now_ns=k * NS)
        if k == 1:
            assert result.flags & F.MODE_SWITCH
        ref = result.reference
        assert abs(ref.ddq[0]) <= 2. + 1e-10
    assert result.flags & F.STALE_COMMAND
    assert result.action == Action.HOLD


def test_joint_position_honors_collision_scaling(slider):
    cfg = ServoConfig(task_axes=(0,), joint_position_gain=1)
    state, command = JointState([0], [0], 0), JointPositionCommand([.01], 0)
    full = Servo(slider, cfg).step(state, command, dt=DT, now_ns=0)
    reduced = Servo(slider, cfg).step(state, command, dt=DT, now_ns=0, collision=CollisionSample(.5, 0, 0))
    np.testing.assert_allclose(reduced.reference.dq, full.reference.dq * .5)
    assert reduced.flags & F.COLLISION_DECELERATION


def test_goal_flag_distinguishes_reference_hold_from_actual_arrival(slider):
    servo = Servo(slider, ServoConfig(task_axes=(0,)))
    servo.step(JointState([0], [0], 0), JointPositionCommand([0], 0), dt=DT, now_ns=0)
    result = servo.step(JointState([.01], [0], NS), JointPositionCommand([0], NS), dt=DT, now_ns=NS)
    assert result.action == Action.HOLD
    assert not result.flags & F.GOAL_REACHED
    assert result.diagnostics.joint_position_error == pytest.approx(.01)


def test_continuous_position_target_uses_shortest_angle_difference():
    joint = _core.Joint(); joint.name = "spin"; joint.type = _core.JointType.CONTINUOUS
    limits = _core.Limits()
    limits.lower = [-np.inf]; limits.upper = [np.inf]
    limits.velocity = [1.]; limits.acceleration = [2.]; limits.margin = [0.]
    model = _core.SerialChainModel([joint], limits, "base", "tool")
    servo = Servo(model, ServoConfig(task_axes=(5,)))
    state = JointState([np.pi - .1], [0.], 0)
    for k in range(600):
        result = servo.step(state, JointPositionCommand([-np.pi + .1], k * NS), dt=DT, now_ns=k * NS)
        assert result.reference is not None
        assert result.reference.q[0] >= np.pi - .1
        state = JointState(result.reference.q, result.reference.dq, (k + 1) * NS)
    assert result.action == Action.HOLD
    assert state.q[0] == pytest.approx(np.pi + .1, abs=1.1e-4)


@pytest.mark.parametrize("kwargs", [
    {"joint_position_gain": 0}, {"joint_position_gain": np.nan},
    {"joint_position_tolerance": -1}, {"joint_position_tolerance": np.inf},
])
def test_joint_position_config_rejects_invalid_values(slider, kwargs):
    with pytest.raises(ValueError):
        Servo(slider, ServoConfig(task_axes=(0,), **kwargs))
