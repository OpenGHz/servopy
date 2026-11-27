import math

import numpy as np
import pytest
from servo_py import (Action, JointJogCommand, JointState, MotionGenerator, SafetyFlag as F, Servo,
                      ServoConfig, StopCommand, TwistCommand)
from conftest import make_sliders

NS = 10_000_000
DT = .01


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def run(servo, commands, q=(0., 0.)):
    state = JointState(np.array(q, dtype=float), np.zeros(len(q)), 0)
    references = []
    for k, command in enumerate(commands):
        if callable(command):
            command = command(k)
        result = servo.step(state, command, dt=DT, now_ns=k * NS)
        assert result.action != Action.REJECT, result.message
        ref = result.reference
        references.append((result, ref))
        state = JointState(ref.q, ref.dq, ref.stamp_ns)
    return references


def servo_pair(**kwargs):
    model = make_sliders(2, velocity=1., acceleration=[2., .2], margin=.01, **kwargs)
    return (Servo(model, ServoConfig(task_axes=(0,))),
            Servo(model, ServoConfig(task_axes=(0,), preserve_direction=True)))


def assert_within_limits(references, acceleration):
    for _, ref in references:
        assert np.all(np.abs(ref.ddq) <= np.asarray(acceleration) + 1e-9)
        assert np.all(np.abs(ref.dq) <= 1. + 1e-12)


def test_acceleration_from_rest_keeps_the_commanded_direction():
    per_joint, collinear = servo_pair()
    target = np.array([.8, .2])
    jog = lambda k: JointJogCommand(target, k * NS)
    # Joint 1 needs .2 / .2 = 1 s to reach its share, which paces both joints.
    distorted = run(per_joint, [jog] * 120, q=(-.6, -.6))
    kept = run(collinear, [jog] * 120, q=(-.6, -.6))
    # Joint 1 has a tenth of joint 0's acceleration; per-joint clamping lets
    # joint 0 run ahead, so the velocity direction initially bends.
    assert max(abs(cross(ref.dq, target)) for _, ref in distorted) > .05
    for result, ref in kept:
        assert abs(cross(ref.dq, target)) <= 1e-12
        assert np.dot(ref.dq, target) >= 0
    assert_within_limits(kept, [2., .2])
    np.testing.assert_allclose(kept[-1][1].dq, target, atol=1e-12)
    assert kept[0][0].flags & F.ACCELERATION_LIMIT and not kept[0][0].flags & F.POSITION_LIMIT
    assert not kept[-1][0].flags & F.ACCELERATION_LIMIT


def test_stop_decelerates_along_the_current_direction():
    per_joint, collinear = servo_pair()
    target = np.array([.8, .2])
    commands = [lambda k: JointJogCommand(target, k * NS)] * 120 + [StopCommand()] * 110
    kept = run(collinear, commands, q=(-.6, -.6))
    np.testing.assert_allclose(kept[119][1].dq, target, atol=1e-12)
    braking = kept[120:]
    for _, ref in braking:
        assert abs(cross(ref.dq, target)) <= 1e-12 and np.all(ref.dq >= 0)
        # Joint 1 paces the stop (.2 / .2 = 1 s); joint 0 uses a quarter of its limit.
        if np.any(ref.dq > 1e-12):
            np.testing.assert_allclose(ref.ddq, [-.8, -.2], atol=1e-9)
    stopped = [k for k, (_, ref) in enumerate(braking) if np.all(ref.dq <= 1e-12)]
    assert stopped[0] in (99, 100)
    assert kept[-1][0].action == Action.HOLD
    distorted = run(per_joint, commands, q=(-.6, -.6))[120:]
    assert max(abs(cross(ref.dq, target)) for _, ref in distorted) > .05


def test_reversal_stays_on_the_command_line():
    _, collinear = servo_pair()
    forward, backward = np.array([.6, .15]), np.array([-.6, -.15])
    commands = ([lambda k: JointJogCommand(forward, k * NS)] * 60 +
                [lambda k: JointJogCommand(backward, k * NS)] * 160)
    kept = run(collinear, commands)
    assert all(abs(cross(ref.dq, forward)) <= 1e-12 for _, ref in kept)
    assert_within_limits(kept, [2., .2])
    np.testing.assert_allclose(kept[-1][1].dq, backward, atol=1e-12)


def test_sharp_turn_finishes_the_old_direction_before_the_new_one():
    _, collinear = servo_pair()
    first, second = np.array([.1, 0.]), np.array([0., .1])
    commands = ([lambda k: JointJogCommand(first, k * NS)] * 30 +
                [lambda k: JointJogCommand(second, k * NS)] * 80)
    kept = run(collinear, commands)
    for _, ref in kept:
        assert abs(cross(ref.dq, first)) <= 1e-12 or abs(cross(ref.dq, second)) <= 1e-12
    np.testing.assert_allclose(kept[-1][1].dq, second, atol=1e-12)


def test_random_commands_respect_position_envelope():
    velocity, acceleration = np.array([.3, .5, .7]), np.array([.7, 1.1, 2.])
    model = make_sliders(3, velocity=velocity, acceleration=acceleration, margin=.02)
    servo = Servo(model, ServoConfig(task_axes=(0,), preserve_direction=True))
    state = JointState(np.zeros(3), np.zeros(3), 0)
    rng = np.random.default_rng(728)
    position_limited = fallback = 0
    parallel = lambda a, b: np.linalg.norm(np.cross(a, b)) <= 1e-9 * max(1., np.linalg.norm(a) * np.linalg.norm(b))
    for k in range(1600):
        if k % 40 == 0:
            target = rng.uniform(-2, 2, 3)
        result = servo.step(state, JointJogCommand(target, k * NS), dt=DT, now_ns=k * NS)
        ref = result.reference
        assert ref is not None, result.message
        position_limited += bool(result.flags & F.POSITION_LIMIT)
        # Near a limit the envelope can exclude both directions; per-joint
        # clamping is then the only safe choice and must still be valid.
        if not parallel(ref.dq, target) and not parallel(ref.dq, state.dq):
            fallback += 1
            assert result.flags & F.POSITION_LIMIT
        assert (np.abs(ref.dq) <= velocity + 1e-9).all()
        assert (np.abs(ref.ddq) <= acceleration + 1e-8).all()
        for s in (0, DT / 3, 2 * DT / 3, DT):
            assert (np.abs(state.q + state.dq * s + .5 * ref.ddq * s * s) <= .98 + 1e-8).all()
        state = JointState(ref.q, ref.dq, ref.stamp_ns)
    assert position_limited > 0 and fallback > 0


def test_twist_tcp_direction_is_kept_while_accelerating(planar):
    twist = np.array([-.3, .3])

    def worst_angle(preserve):
        servo = Servo(planar, ServoConfig(task_axes=(0, 1), max_linear_speed=.5, preserve_direction=preserve))
        state = JointState(np.array([.5, -1.]), np.zeros(2), 0)
        worst = 0.
        for k in range(40):
            result = servo.step(state, TwistCommand([*twist, 0.], [0., 0., 0.], k * NS), dt=DT, now_ns=k * NS)
            assert result.action != Action.REJECT, result.message
            ref = result.reference
            tcp = (planar.jacobian(state.q) @ ref.dq)[:2]
            worst = max(worst, abs(math.atan2(cross(twist, tcp), np.dot(twist, tcp))))
            state = JointState(ref.q, ref.dq, ref.stamp_ns)
        return worst

    assert worst_angle(False) > .2
    assert worst_angle(True) < 1e-6


def test_default_generator_only():
    class Passthrough(MotionGenerator):
        def reset(self):
            pass
    with pytest.raises(ValueError, match="preserve_direction"):
        Servo(make_sliders(), ServoConfig(task_axes=(0,), preserve_direction=True),
              motion_generator=Passthrough())
