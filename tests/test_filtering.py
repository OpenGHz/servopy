import math

import numpy as np
import pytest
from servo_py import (Action, ButterworthCommandFilter, JointJogCommand, JointPositionCommand,
                      JointState, PoseCommand, SafetyFlag as F, Servo, ServoConfig, ServoRunner,
                      SimulatedDevice, StopCommand, TwistCommand, load_urdf)
from servo_py import _core
from conftest import make_sliders

NS = 10_000_000
DT = .01


def coefficient(cutoff, dt):
    return 1. / math.tan(math.pi * cutoff * dt)


def servo_for(model, **kwargs):
    return Servo(model, ServoConfig(task_axes=(0,), **kwargs))


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


@pytest.mark.parametrize("dt", [.01, .004])
def test_cutoff_is_minus_3db_independent_of_period(dt):
    """Prewarped bilinear design: unit gain at DC and 1/sqrt(2) at the cutoff."""
    servo = servo_for(make_sliders())
    flt = ButterworthCommandFilter(servo, cutoff_hz=4.)
    stamp = round(dt * 1e9)
    times, out = [], []
    for k in range(int(4. / dt)):
        t = k * dt
        command = JointJogCommand([math.sin(2 * math.pi * 4. * t)], k * stamp)
        out.append(flt.filter(command, dt=dt, now_ns=k * stamp).velocities[0])
        times.append(t)
    t = np.array(times[len(times) // 2:])
    basis = np.column_stack([np.sin(2 * math.pi * 4. * t), np.cos(2 * math.pi * 4. * t)])
    coefficients = np.linalg.lstsq(basis, np.array(out[len(out) // 2:]), rcond=None)[0]
    assert np.hypot(*coefficients) == pytest.approx(1 / math.sqrt(2), abs=1e-6)
    flt.reset()
    for k in range(400):
        held = flt.filter(JointJogCommand([.3], k * stamp), dt=dt, now_ns=k * stamp).velocities[0]
    assert held == pytest.approx(.3, abs=1e-12)


def test_velocity_commands_start_from_rest_and_stop_is_not_delayed():
    servo = servo_for(make_sliders())
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    c = coefficient(2., DT)
    first = flt.filter(JointJogCommand([.4], 0), dt=DT, now_ns=0)
    assert first.velocities[0] == pytest.approx(.4 / (1 + c))
    assert first.stamp_ns == 0
    second = flt.filter(JointJogCommand([.4], 0), dt=DT, now_ns=NS)
    assert second.velocities[0] == pytest.approx((.8 + (c - 1) * first.velocities[0]) / (1 + c))
    stop = StopCommand()
    assert flt.filter(stop, dt=DT, now_ns=2 * NS) is stop
    again = flt.filter(JointJogCommand([.4], 3 * NS), dt=DT, now_ns=3 * NS)
    assert again.velocities[0] == pytest.approx(first.velocities[0])


def test_filtered_stream_brakes_in_the_same_steps_as_unfiltered():
    """Filtering conditions the request only; Stop bypasses its lag."""
    def run(with_filter):
        servo = servo_for(make_sliders())
        flt = ButterworthCommandFilter(servo, cutoff_hz=3.)
        state = JointState([0.], [0.], 0)
        for k in range(200):
            command = JointJogCommand([.5], k * NS) if k < 120 else StopCommand()
            if with_filter:
                command = flt.filter(command, dt=DT, now_ns=k * NS)
            result = servo.step(state, command, dt=DT, now_ns=k * NS)
            assert result.action != Action.REJECT and not result.flags & F.INVALID_COMMAND
            ref = result.reference
            state = JointState(ref.q, ref.dq, ref.stamp_ns)
            if k >= 120 and result.action == Action.HOLD:
                return k, state.dq[0]
        raise AssertionError("did not stop")
    plain, filtered = run(False), run(True)
    assert filtered[0] == plain[0]
    assert filtered[1] == 0 and plain[1] == 0


def test_noisy_jog_is_smoothed_before_acceleration_limits():
    rng = np.random.default_rng(17)
    noise = rng.normal(0., .15, 400)
    def accelerations(with_filter):
        servo = Servo(make_sliders(acceleration=50.), ServoConfig(task_axes=(0,)))
        flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
        state, values = JointState([0.], [0.], 0), []
        for k in range(400):
            # Zero-mean input keeps the slider far from its position limits.
            command = JointJogCommand([noise[k]], k * NS)
            if with_filter:
                command = flt.filter(command, dt=DT, now_ns=k * NS)
            ref = servo.step(state, command, dt=DT, now_ns=k * NS).reference
            state = JointState(ref.q, ref.dq, ref.stamp_ns)
            values.append(ref.ddq[0])
        return np.std(values[100:])
    assert accelerations(True) < .25 * accelerations(False)


def test_position_target_steps_are_monotone_without_overshoot_under_variable_period():
    servo = servo_for(make_sliders(2))
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    periods = [.008, .012, .01, .009, .011]
    now, previous = 0, None
    for k in range(300):
        target = [.2, -.3] if k < 100 else [.7, .4]
        dt = periods[k % len(periods)]
        out = flt.filter(JointPositionCommand(target, now), dt=dt, now_ns=now).positions
        if k < 100:
            np.testing.assert_allclose(out, target, atol=1e-15)
        else:
            assert np.all(out >= np.minimum(previous, target) - 1e-15)
            assert np.all(out <= np.maximum(previous, target) + 1e-15)
            assert np.all(np.abs(np.asarray(target) - out) <= np.abs(np.asarray(target) - previous) + 1e-15)
        previous = out
        now += round(dt * 1e9)
    np.testing.assert_allclose(previous, [.7, .4], atol=1e-9)


def test_position_targets_at_effective_limit_remain_valid():
    servo = servo_for(make_sliders(margin=.01))
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    state = JointState([.5], [0.], 0)
    for k in range(200):
        target = .99 if k % 2 else .98999999
        command = flt.filter(JointPositionCommand([target], k * NS), dt=DT, now_ns=k * NS)
        assert command.positions[0] <= .99
        result = servo.step(state, command, dt=DT, now_ns=k * NS)
        assert not result.flags & F.INVALID_COMMAND, result.message
        state = JointState(result.reference.q, result.reference.dq, result.reference.stamp_ns)


def test_continuous_joint_targets_follow_the_shortest_branch(mixed_urdf):
    model = load_urdf(mixed_urdf, base="base", tip="tool", acceleration_limits=[3., 3.])
    servo = Servo(model, ServoConfig(task_axes=(0, 1)))
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    flt.filter(JointPositionCommand([3.1, 0.], 0), dt=DT, now_ns=0)
    unwrapped = 3.1 + (2 * math.pi - 6.2)
    state = JointState([3.1, 0.], [0., 0.], 0)
    for k in range(1, 150):
        command = flt.filter(JointPositionCommand([-3.1, 0.], k * NS), dt=DT, now_ns=k * NS)
        assert 3.1 - 1e-12 <= command.positions[0] <= unwrapped + 1e-12
        result = servo.step(state, command, dt=DT, now_ns=k * NS)
        assert result.action != Action.REJECT and not result.flags & F.INVALID_COMMAND
        state = JointState(result.reference.q, result.reference.dq, result.reference.stamp_ns)
    assert command.positions[0] == pytest.approx(unwrapped, abs=1e-6)


def test_pose_orientation_follows_the_geodesic(planar):
    servo = Servo(planar, ServoConfig(task_axes=(0, 1)))
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    start, goal = np.eye(4), np.eye(4)
    start[:3, 3] = [.3, .1, 0.]
    goal[:3, :3], goal[:3, 3] = rotation_z(math.pi / 2), [.5, .2, 0.]
    first = flt.filter(PoseCommand(start, 0), dt=DT, now_ns=0)
    np.testing.assert_allclose(first.pose, start, atol=1e-12)
    angles = []
    for k in range(1, 200):
        pose = flt.filter(PoseCommand(goal, k * NS), dt=DT, now_ns=k * NS).pose
        assert _core.valid_pose(pose)
        error = _core.rotation_log(goal[:3, :3] @ pose[:3, :3].T)
        np.testing.assert_allclose(error[:2], 0., atol=1e-12)
        angles.append(error[2])
        assert np.all(pose[:2, 3] >= start[:2, 3] - 1e-12) and np.all(pose[:2, 3] <= goal[:2, 3] + 1e-12)
    assert all(b <= a + 1e-15 for a, b in zip(angles, angles[1:]))
    np.testing.assert_allclose(pose, goal, atol=1e-9)


def test_named_commands_are_mapped_to_model_order():
    servo = servo_for(make_sliders(3))
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    jog = flt.filter(JointJogCommand([.6, .3], 0, names=["j2", "j0"]), dt=DT, now_ns=0)
    assert jog.names is None
    c = coefficient(2., DT)
    np.testing.assert_allclose(jog.velocities, np.array([.3, 0., .6]) / (1 + c))
    flt.reset()
    position = flt.filter(JointPositionCommand([.1, .2, .3], 0, names=["j1", "j2", "j0"]), dt=DT, now_ns=0)
    assert position.names is None
    np.testing.assert_allclose(position.positions, [.3, .1, .2])


def test_twist_frame_change_restarts_from_rest(planar):
    servo = Servo(planar, ServoConfig(task_axes=(0, 1)))
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    c = coefficient(2., DT)
    for k in range(50):
        base = flt.filter(TwistCommand([.1, 0., 0.], [0., 0., 0.], k * NS), dt=DT, now_ns=k * NS)
    assert base.linear[0] > .09 and base.expressed_in == "base"
    tool = flt.filter(TwistCommand([.1, 0., 0.], [0., 0., 0.], 50 * NS, expressed_in="tool"), dt=DT, now_ns=50 * NS)
    assert tool.linear[0] == pytest.approx(.1 / (1 + c)) and tool.expressed_in == "tool"


@pytest.mark.parametrize("make_command", [
    lambda: JointJogCommand([.4], 0),                                # stale at now_ns below
    lambda: JointJogCommand([.4], 500 * NS),                         # future stamp
    lambda: JointJogCommand([.4, .1], 300 * NS),                     # wrong dimension
    lambda: JointJogCommand([math.nan], 300 * NS),
    lambda: JointPositionCommand([2.], 300 * NS),                    # outside limits
    lambda: TwistCommand([.1, 0, 0], [0, 0, 0], 300 * NS, expressed_in="camera"),
    lambda: PoseCommand(np.ones((4, 4)), 300 * NS),
    lambda: "not a command",
])
def test_unusable_commands_pass_through_and_reset(make_command):
    servo = servo_for(make_sliders())
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    for k in range(290, 300):
        flt.filter(JointJogCommand([.4], k * NS), dt=DT, now_ns=k * NS)
    command = make_command()
    assert flt.filter(command, dt=DT, now_ns=300 * NS) is command
    restarted = flt.filter(JointJogCommand([.4], 301 * NS), dt=DT, now_ns=301 * NS)
    assert restarted.velocities[0] == pytest.approx(.4 / (1 + coefficient(2., DT)))


def test_invalid_period_passes_through():
    servo = servo_for(make_sliders())
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    command = JointJogCommand([.4], 0)
    for dt in (0., -.01, math.nan, .2):
        assert flt.filter(command, dt=dt, now_ns=0) is command


@pytest.mark.parametrize("cutoff", [0., -1., math.nan, math.inf, 5.01])
def test_invalid_cutoff_rejected(cutoff):
    servo = servo_for(make_sliders())  # default max_dt=0.05 allows up to 5 Hz
    with pytest.raises(ValueError):
        ButterworthCommandFilter(servo, cutoff_hz=cutoff)


def test_cutoff_limit_follows_max_dt():
    ButterworthCommandFilter(servo_for(make_sliders(), max_dt=.02), cutoff_hz=12.5)
    with pytest.raises(ValueError, match="max_dt"):
        ButterworthCommandFilter(servo_for(make_sliders(), max_dt=.02), cutoff_hz=12.6)


class Clock:
    def __init__(self):
        self.now = 0
    def clock(self):
        return self.now
    def sleep(self, duration):
        self.now += round(duration * 1e9)


class Recorder:
    def __init__(self):
        self.commands = []
    def record(self, now, dt, state, command, result, collision=None):
        self.commands.append(command)


def test_runner_filters_mailbox_reads_and_records_filtered_commands(slider):
    clock, recorder = Clock(), Recorder()
    servo = Servo(slider, ServoConfig(task_axes=(0,)))
    flt = ButterworthCommandFilter(servo, cutoff_hz=2.)
    runner = ServoRunner(servo, SimulatedDevice([0]), clock_ns=clock.clock, sleep=clock.sleep,
                         recorder=recorder, command_filter=flt)
    runner.commands.publish(JointJogCommand([.4], 0))
    runner.run(max_steps=5)
    c = coefficient(2., DT)
    assert recorder.commands[0].velocities[0] == pytest.approx(.4 / (1 + c))
    jogs = [command.velocities[0] for command in recorder.commands if isinstance(command, JointJogCommand)]
    assert len(jogs) == 5 and all(b > a for a, b in zip(jogs, jogs[1:]))
    assert isinstance(recorder.commands[-1], StopCommand)
