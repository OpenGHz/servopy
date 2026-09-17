from dataclasses import replace
import numpy as np
import pytest
from servo_py import (Servo, ServoConfig, JointState, JointJogCommand, TwistCommand,
    PoseCommand, StopCommand, CollisionSample, SafetyFlag as F, Action, Kinematics)
from servo_py import _core
from conftest import make_sliders

DT = .01
NS = 10_000_000


def config(**kwargs):
    return ServoConfig(task_axes=(0,), **kwargs)


def initial(n=1, q=0., dq=0., stamp_ns=0):
    return JointState(np.full(n, q), np.full(n, dq), stamp_ns)


def follow(result):
    assert result.action != Action.REJECT, result.message
    ref = result.reference
    return JointState(ref.q, ref.dq, ref.stamp_ns)


def test_joint_jog_reference_consistency_and_braking(slider):
    servo = Servo(slider, config())
    state = initial()
    for k in range(150):
        command = JointJogCommand([.4], k * NS) if k < 80 else StopCommand()
        result = servo.step(state, command, dt=DT, now_ns=k * NS)
        assert result.reference is not None
        ref = result.reference
        np.testing.assert_allclose(ref.q, state.q + .5 * (state.dq + ref.dq) * DT, atol=1e-12)
        np.testing.assert_allclose(ref.ddq, (ref.dq - state.dq) / DT, atol=1e-12)
        assert np.max(np.abs(ref.ddq)) <= 2. + 1e-9
        assert np.max(np.abs(ref.dq)) <= .4 + 1e-9
        assert result.flags & F.COLLISION_DISABLED
        if k == 80:
            assert result.action == Action.BRAKE
        state = follow(result)
    assert result.action == Action.HOLD
    np.testing.assert_allclose(state.dq, 0, atol=1e-12)


def test_continuous_state_and_limit_constraints_under_random_commands():
    model = make_sliders(3, velocity=[.3, .5, .7], acceleration=[.7, 1.1, 2.], margin=.02)
    servo = Servo(model, config())
    state = initial(3)
    rng = np.random.default_rng(728)
    velocity = np.array([.3, .5, .7]); acceleration = np.array([.7, 1.1, 2.])
    for k in range(1600):
        if k % 40 == 0:
            target = rng.uniform(-2, 2, 3)
        result = servo.step(state, JointJogCommand(target, k * NS), dt=DT, now_ns=k * NS)
        ref = result.reference
        assert ref is not None, result.message
        assert (np.abs(ref.dq) <= velocity + 1e-9).all()
        assert (np.abs(ref.ddq) <= acceleration + 1e-8).all()
        assert (np.abs(ref.q) <= .98 + 1e-8).all()
        # Check within-interval positions as well as endpoints.
        for s in (0, DT / 3, 2 * DT / 3, DT):
            pos = state.q + state.dq * s + .5 * ref.ddq * s * s
            assert (np.abs(pos) <= .98 + 1e-8).all()
        state = follow(result)


def test_position_limit_stop_and_retreat(slider):
    servo = Servo(slider, config())
    state = initial(q=.7)
    touched = False
    for k in range(250):
        result = servo.step(state, JointJogCommand([1.], k * NS), dt=DT, now_ns=k * NS)
        touched |= bool(result.flags & F.POSITION_LIMIT)
        state = follow(result)
    assert touched
    assert .989 < state.q[0] <= .990000001
    assert abs(state.dq[0]) < 1e-6
    result = servo.step(state, JointJogCommand([-.5], 250 * NS), dt=DT, now_ns=250 * NS)
    assert result.reference.dq[0] < 0


def test_starting_in_margin_allows_retreat(slider):
    servo = Servo(slider, config())
    result = servo.step(initial(q=.995), JointJogCommand([.5], 0), dt=DT, now_ns=0)
    assert result.action == Action.HOLD
    assert result.flags & F.POSITION_LIMIT
    result = servo.step(follow(result), JointJogCommand([-.5], NS), dt=DT, now_ns=NS)
    assert result.reference.q[0] < .995


def test_infeasible_stopping_distance_returns_no_target(slider):
    result = Servo(slider, config()).step(initial(q=.98, dq=.8), JointJogCommand([0.], 0), dt=DT, now_ns=0)
    assert result.action == Action.REJECT and result.reference is None
    assert result.flags & F.INFEASIBLE


def test_expired_command_brakes_to_stop(slider):
    servo = Servo(slider, config(command_timeout=.02))
    state = initial(dq=.3)
    for k in range(40):
        result = servo.step(state, JointJogCommand([.3], 0), dt=DT, now_ns=k * NS)
        state = follow(result)
    assert result.flags & F.STALE_COMMAND
    assert result.action == Action.HOLD
    np.testing.assert_allclose(state.dq, 0, atol=1e-12)


@pytest.mark.parametrize("command", [
    JointJogCommand([np.nan], 0), JointJogCommand([np.inf], 0), JointJogCommand([1, 2], 0),
    JointJogCommand([.2], 0, ["missing"]), JointJogCommand([.2, .3], 0, ["j0", "j0"]),
    TwistCommand([0, 0, 0], [0, 0, 0], 0, expressed_in="camera"),
    TwistCommand([0, 0, 0], [0, 0, 0], 0, reference_point="base"),
    PoseCommand(np.zeros((4, 4)), 0), object(), JointJogCommand([.2], NS),
])
def test_invalid_commands_produce_controlled_braking(slider, command):
    result = Servo(slider, config()).step(initial(dq=.2), command, dt=DT, now_ns=0)
    assert result.action == Action.BRAKE
    assert result.flags & F.INVALID_COMMAND
    assert 0 < result.reference.dq[0] < .2


def test_joint_name_subset_mapping():
    model = make_sliders(2)
    result = Servo(model, config()).step(initial(2), JointJogCommand([.3], 0, ["j1"]), dt=DT, now_ns=0)
    np.testing.assert_allclose(result.reference.dq, [0., .02])


def test_stale_state_fault_latches_until_reset(slider):
    servo = Servo(slider, config())
    result = servo.step(initial(), StopCommand(), dt=DT, now_ns=1_000_000_000)
    assert result.flags & F.STALE_STATE and result.reference is None
    fresh = initial(stamp_ns=1_000_000_000)
    result = servo.step(fresh, StopCommand(), dt=DT, now_ns=1_000_000_000)
    assert result.flags & F.FAULT_LATCHED
    servo.reset(fresh, now_ns=1_000_000_000)
    assert servo.step(fresh, StopCommand(), dt=DT, now_ns=1_000_000_000).action == Action.HOLD


@pytest.mark.parametrize("dt", [0, -1, np.nan, np.inf, .5])
def test_invalid_period(slider, dt):
    result = Servo(slider, config()).step(initial(), StopCommand(), dt=dt, now_ns=0)
    assert result.flags & F.INVALID_TIMING
    assert result.reference is None


@pytest.mark.parametrize("stamp", [0, 100_000_000])
def test_clock_repeat_or_deadline_miss(slider, stamp):
    servo = Servo(slider, config())
    servo.step(initial(), StopCommand(), dt=DT, now_ns=0)
    result = servo.step(initial(stamp_ns=stamp), StopCommand(), dt=DT, now_ns=stamp)
    assert result.flags & F.INVALID_TIMING


def test_tracking_error_does_not_integrate_blindly(slider):
    servo = Servo(slider, config(max_tracking_error=.05))
    servo.step(initial(), StopCommand(), dt=DT, now_ns=0)
    result = servo.step(initial(q=.1, stamp_ns=NS), StopCommand(), dt=DT, now_ns=NS)
    assert result.flags & F.TRACKING_ERROR and result.reference is None


def test_measured_overspeed_after_initialization_faults(slider):
    servo = Servo(slider, config())
    servo.step(initial(), StopCommand(), dt=DT, now_ns=0)
    result = servo.step(initial(dq=1.1, stamp_ns=NS), StopCommand(), dt=DT, now_ns=NS)
    assert result.flags & F.INVALID_STATE and result.flags & F.VELOCITY_LIMIT
    assert result.reference is None


@pytest.mark.parametrize("sample,flag", [
    (None, F.COLLISION_MISSING), (CollisionSample(0, 0, 0), F.COLLISION_HALT),
    (CollisionSample(np.nan, 0, 0), F.COLLISION_STALE),
    (CollisionSample(1, 1, 0), F.COLLISION_STALE),
])
def test_collision_failures_are_explicit(slider, sample, flag):
    result = Servo(slider, config(collision_required=True)).step(initial(), StopCommand(), dt=DT, now_ns=0, collision=sample)
    assert result.action == Action.REJECT and result.flags & flag


def test_collision_source_state_age_and_scaling(slider):
    servo = Servo(slider, config(collision_required=True))
    result = servo.step(initial(stamp_ns=1_000_000_000), StopCommand(), dt=DT,
                        now_ns=1_000_000_000, collision=CollisionSample(1., 1_000_000_000, 0))
    assert result.flags & F.COLLISION_STALE
    result = Servo(slider, config()).step(initial(), JointJogCommand([.02], 0), dt=DT, now_ns=0,
                                           collision=CollisionSample(.5, 0, 0))
    assert result.flags & F.COLLISION_DECELERATION
    np.testing.assert_allclose(result.reference.dq, [.01])


def test_tool_and_base_twists_are_equivalent(planar, xy_config):
    q = np.array([.5, -.8])
    base_linear = np.array([.02, .01, 0.])
    tool_linear = planar.fk(q)[:3, :3].T @ base_linear
    state = JointState(q, np.zeros(2), 0)
    a = Servo(planar, xy_config).step(state, TwistCommand(base_linear, [0, 0, 0], 0), dt=DT, now_ns=0)
    b = Servo(planar, xy_config).step(state, TwistCommand(tool_linear, [0, 0, 0], 0, expressed_in="tool"), dt=DT, now_ns=0)
    np.testing.assert_allclose(a.reference.q, b.reference.q, atol=1e-12)
    np.testing.assert_allclose(a.reference.dq, b.reference.dq, atol=1e-12)


def test_pose_tracking_converges(planar, xy_config):
    servo = Servo(planar, xy_config)
    target = planar.fk(np.array([.7, -.7]))
    state = JointState(np.array([.5, -1.]), np.zeros(2), 0)
    before = np.linalg.norm(planar.fk(state.q)[:3, 3] - target[:3, 3])
    for k in range(1300):
        result = servo.step(state, PoseCommand(target, k * NS), dt=DT, now_ns=k * NS)
        state = follow(result)
    error = np.linalg.norm(planar.fk(state.q)[:3, 3] - target[:3, 3])
    assert error < 2e-4 and error < before * .01
    assert result.flags & F.GOAL_REACHED


def test_singularity_blocks_unreachable_motion_and_allows_escape(planar, xy_config):
    q = np.zeros(2)
    state = JointState(q, np.zeros(2), 0)
    blocked = Servo(planar, xy_config).step(state, TwistCommand([.1, 0, 0], [0, 0, 0], 0), dt=DT, now_ns=0)
    assert blocked.flags & F.SINGULARITY_HALT
    assert blocked.action == Action.HOLD
    escaped = Servo(planar, xy_config).step(state, TwistCommand([0, .1, 0], [0, 0, 0], 0), dt=DT, now_ns=0)
    assert escaped.flags & F.LEAVING_SINGULARITY
    assert np.isfinite(escaped.reference.q).all()
    assert np.linalg.norm(escaped.reference.dq) > 0
    np.testing.assert_array_equal(q, [0, 0])


def test_mode_change_keeps_acceleration_bounded(planar, xy_config):
    servo = Servo(planar, xy_config)
    state = JointState(np.array([.5, -.8]), np.zeros(2), 0)
    for k in range(30):
        result = servo.step(state, JointJogCommand([.4, .4], k * NS), dt=DT, now_ns=k * NS)
        state = follow(result)
    switched = servo.step(state, TwistCommand([-.2, 0, 0], [0, 0, 0], 30 * NS), dt=DT, now_ns=30 * NS)
    assert switched.flags & F.MODE_SWITCH
    assert (np.abs(switched.reference.ddq) <= 3 + 1e-9).all()


def test_returned_arrays_cannot_corrupt_internal_state(slider):
    servo = Servo(slider, config())
    result = servo.step(initial(), JointJogCommand([.1], 0), dt=DT, now_ns=0)
    state = follow(result)
    result.reference.q[:] = 1e6
    result.reference.dq[:] = 1e6
    second = servo.step(state, JointJogCommand([.1], NS), dt=DT, now_ns=NS)
    assert second.action != Action.REJECT
    assert second.reference.q[0] < .01


@pytest.mark.parametrize("kwargs", [dict(task_axes=(0, 0)), dict(task_axes=(6,)),
    dict(max_damping=0), dict(singularity_soft=.0001), dict(task_weights=[1, 1, 1, 1, 1, 0]),
    dict(max_tracking_error=float("nan"))])
def test_invalid_config_rejected(slider, kwargs):
    with pytest.raises((ValueError, TypeError)):
        Servo(slider, replace(config(), **kwargs))


def test_backend_exceptions_become_latched_faults(slider):
    class Broken(Kinematics):
        def __init__(self): super().__init__(); self.limits = slider.limits
        def dof(self): return 1
        def joint_names(self): return ["j0"]
        def base_frame(self): return "base"
        def tip_frame(self): return "tool"
        def fk(self, q): raise ValueError("deliberate backend failure")
        def jacobian(self, q): return np.zeros((6, 1))
    model = Broken()
    result = Servo(model, config()).step(initial(), TwistCommand([.1, 0, 0], [0, 0, 0], 0), dt=DT, now_ns=0)
    assert result.flags & F.MODEL_ERROR and result.reference is None


def test_orientation_tracking_at_half_turn():
    # Spherical Z-Y-X wrist with no translational task.
    joints = []
    for i, axis in enumerate(([0, 0, 1], [0, 1, 0], [1, 0, 0])):
        joint = _core.Joint(); joint.name = f"r{i}"
        joint.type = _core.JointType.CONTINUOUS; joint.axis = axis
        joints.append(joint)
    limits = _core.Limits()
    limits.lower = [-np.inf] * 3; limits.upper = [np.inf] * 3
    limits.velocity = [1] * 3; limits.acceleration = [2] * 3; limits.margin = [0] * 3
    model = _core.SerialChainModel(joints, limits, "base", "tool")
    servo = Servo(model, ServoConfig(task_axes=(3, 4, 5)))
    target = model.fk(np.array([0., 0., np.pi]))
    state = initial(3)
    for k in range(1500):
        result = servo.step(state, PoseCommand(target, k * NS), dt=DT, now_ns=k * NS)
        state = follow(result)
    rotation_error = model.fk(state.q)[:3, :3] @ target[:3, :3].T
    angle = np.arccos(np.clip((np.trace(rotation_error) - 1) / 2, -1, 1))
    assert angle < .002
    assert result.flags & F.GOAL_REACHED


@pytest.mark.parametrize("state,flag", [
    (JointState([np.nan], [0], 0), F.INVALID_STATE),
    (JointState([0], [np.inf], 0), F.INVALID_STATE),
    (JointState([0, 0], [0, 0], 0), F.INVALID_STATE),
    (JointState([0], [0], NS), F.FUTURE_TIMESTAMP),
    (JointState([2], [0], 0), F.POSITION_LIMIT),
])
def test_invalid_feedback_has_no_reference(slider, state, flag):
    result = Servo(slider, config()).step(state, StopCommand(), dt=DT, now_ns=0)
    assert result.flags & flag and result.reference is None


def test_large_monotonic_timestamp_retains_nanosecond_precision(slider):
    now = 8_000_000_000_000_000_000
    servo = Servo(slider, config())
    state = initial(stamp_ns=now)
    for k in range(5):
        result = servo.step(state, JointJogCommand([.1], now), dt=DT, now_ns=now)
        assert result.action != Action.REJECT
        state = follow(result)
        now += NS


def test_full_pose_tracking_six_axis():
    from servo_py import load_urdf
    from conftest import ROOT
    model = load_urdf(ROOT / "examples/arm6.urdf", base="base", tip="tool", acceleration_limits=3.)
    servo = Servo(model, ServoConfig(singularity_soft=.01, singularity_hard=.0001))
    q = np.array([.2, -.7, 1., .4, .6, -.3])
    target = model.fk(q + np.array([.05, .03, -.02, .04, -.02, .03]))
    state = JointState(q, np.zeros(6), 0)
    for k in range(1400):
        result = servo.step(state, PoseCommand(target, k * NS), dt=DT, now_ns=k * NS)
        state = follow(result)
    actual = model.fk(state.q)
    assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) < 2e-4
    assert np.linalg.norm(actual[:3, :3] - target[:3, :3]) < .003
