import numpy as np
import pytest
from servo_py import (Action, JointJogCommand, JointPositionCommand, JointState,
                      RuckigSmoothing, SafetyFlag, Servo, ServoConfig, StopCommand)

pytest.importorskip("ruckig")


@pytest.mark.parametrize("position", [False, True])
def test_jerk_continuity_reversal_stop_and_variable_period(slider, position):
    servo = Servo(slider, ServoConfig(task_axes=(0,)), motion_generator=RuckigSmoothing(10.))
    q, dq, ddq, stamp = np.zeros(1), np.zeros(1), np.zeros(1), 0
    for k in range(550):
        dt = [.005, .01, .015][k % 3]
        value = .75 if k < 180 else -.65
        command = (JointPositionCommand([value], stamp) if position else JointJogCommand([value], stamp)) if k < 350 else StopCommand()
        result = servo.step(JointState(q, dq, stamp), command, dt=dt, now_ns=stamp)
        assert result.action != Action.REJECT, result.message
        previous = servo.sample_reference(0)
        np.testing.assert_allclose(previous.q, q, atol=1e-9)
        np.testing.assert_allclose(previous.dq, dq, atol=1e-9)
        np.testing.assert_allclose(previous.ddq, ddq, atol=1e-9)
        for t in np.linspace(0, dt, 7)[1:]:
            sample = servo.sample_reference(float(t))
            assert np.max(abs(sample.ddq - previous.ddq)) <= 10. * dt / 6 + 1e-8
            assert abs(sample.q[0]) <= .99 + 1e-9
            assert abs(sample.dq[0]) <= 1 + 1e-9
            assert abs(sample.ddq[0]) <= 2 + 1e-9
            previous = sample
        q, dq, ddq = result.reference.q, result.reference.dq, result.reference.ddq
        stamp += round(dt * 1e9)
    assert result.action == Action.HOLD
    np.testing.assert_allclose(dq, 0, atol=1e-9)
    np.testing.assert_allclose(ddq, 0, atol=1e-9)


def test_stop_remains_feasible_near_joint_limit(slider):
    servo = Servo(slider, ServoConfig(task_axes=(0,)), motion_generator=RuckigSmoothing(8))
    q, dq = [.8], [0.]
    limited = False
    for k in range(400):
        stamp = k * 10_000_000
        result = servo.step(JointState(q, dq, stamp), JointJogCommand([1], stamp), dt=.01, now_ns=stamp)
        assert result.action != Action.REJECT, result.message
        q, dq = result.reference.q, result.reference.dq
        assert q[0] <= .99 + 1e-9
        limited |= bool(result.flags & SafetyFlag.POSITION_LIMIT)
    assert limited


def test_infeasible_initial_jerk_stop_rejects_and_reset_recovers(slider):
    servo = Servo(slider, ServoConfig(task_axes=(0,)), motion_generator=RuckigSmoothing(2))
    result = servo.step(JointState([.98], [.5], 0), StopCommand(), dt=.01, now_ns=0)
    assert result.action == Action.REJECT
    assert result.flags & SafetyFlag.SMOOTHING_ERROR
    with pytest.raises(ValueError):
        servo.sample_reference(0)
    servo.reset(JointState([0], [0], 0), now_ns=0)
    assert servo.step(JointState([0], [0], 0), StopCommand(), dt=.01, now_ns=0).action == Action.HOLD


def test_native_interval_sampling_is_consistent(slider):
    servo = Servo(slider, ServoConfig(task_axes=(0,)))
    result = servo.step(JointState([0], [0], 0), JointJogCommand([.5], 0), dt=.01, now_ns=0)
    midpoint = servo.sample_reference(.005)
    assert midpoint.q[0] == pytest.approx(.000025)
    np.testing.assert_allclose(servo.sample_reference(.01).q, result.reference.q)
    with pytest.raises(ValueError):
        servo.sample_reference(.02)


def test_stale_command_brakes_with_jerk_limit(slider):
    servo = Servo(slider, ServoConfig(task_axes=(0,)), motion_generator=RuckigSmoothing(10))
    q, dq = [0], [0]
    for k in range(150):
        stamp = k * 10_000_000
        result = servo.step(JointState(q, dq, stamp), JointJogCommand([.4], min(stamp, 300_000_000)), dt=.01, now_ns=stamp)
        assert result.action != Action.REJECT
        q, dq = result.reference.q, result.reference.dq
    assert result.flags & SafetyFlag.STALE_COMMAND
    assert result.action == Action.HOLD


def test_stationary_nonzero_position_with_limits_excluding_zero():
    from conftest import make_sliders
    model = make_sliders(lower=-2., upper=-1.)
    servo = Servo(model, ServoConfig(task_axes=(0,)), motion_generator=RuckigSmoothing(10))
    result = servo.step(JointState([-1.5], [0], 0), StopCommand(), dt=.01, now_ns=0)
    assert result.action == Action.HOLD
    np.testing.assert_allclose(result.reference.q, [-1.5])


def test_failed_generator_reset_keeps_fault_latched(slider):
    from servo_py import MotionGenerator
    class BrokenReset(MotionGenerator):
        def reset(self):
            raise RuntimeError("reset failed")
    servo = Servo(slider, ServoConfig(task_axes=(0,)), motion_generator=BrokenReset())
    with pytest.raises(RuntimeError, match="reset failed"):
        servo.reset(JointState([0], [0], 0), now_ns=0)
    result = servo.step(JointState([0], [0], 0), StopCommand(), dt=.01, now_ns=0)
    assert result.flags & SafetyFlag.FAULT_LATCHED
