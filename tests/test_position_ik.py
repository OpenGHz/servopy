import numpy as np
import pytest

from servo_py import (
    Action, JointLimits, JointPositionCommand, JointState, PoseCommand,
    PositionIKAdapter, SafetyFlag as F, Servo, ServoConfig, StopCommand,
)


def test_ik_preserves_source_stamp_and_core_rejects_late_result(slider):
    servo = Servo(slider, ServoConfig(task_axes=(0,), command_timeout=.02))
    adapter = PositionIKAdapter(servo, lambda target, seed: [target[0, 3]])
    target = slider.fk(np.array([.2]))
    prepared = adapter.solve(PoseCommand(target, 10_000_000), [0.], now_ns=15_000_000)
    assert prepared.success
    assert prepared.command.stamp_ns == 10_000_000
    result = servo.step(JointState([0.], [.2], 40_000_000), prepared.command, dt=.01, now_ns=40_000_000)
    assert result.flags & F.STALE_COMMAND
    assert result.action == Action.BRAKE


@pytest.mark.parametrize("case", ["stale", "future", "frame", "matrix", "seed", "not_pose"])
def test_invalid_requests_do_not_call_solver(slider, case):
    def unexpected(*args):
        pytest.fail("invalid request reached solver")
    adapter = PositionIKAdapter(Servo(slider, ServoConfig(task_axes=(0,))), unexpected)
    target = slider.fk(np.array([.2]))
    command, seed, now = PoseCommand(target, 0), [0.], 0
    if case == "stale": now = 200_000_000
    if case == "future": command = PoseCommand(target, 1)
    if case == "frame": command = PoseCommand(target, 0, "camera")
    if case == "matrix": command = PoseCommand(np.zeros((4, 4)), 0)
    if case == "seed": seed = [np.nan]
    if case == "not_pose": command = StopCommand()
    result = adapter.solve(command, seed, now_ns=now)
    assert not result.success and isinstance(result.command, StopCommand)
    assert result.message


@pytest.mark.parametrize("solution", [None, [np.nan], [np.inf], [], [[.2]], [1.1], [.995], [-.2]])
def test_invalid_solutions_become_stop_commands(slider, solution):
    servo = Servo(slider, ServoConfig(task_axes=(0,)))
    adapter = PositionIKAdapter(servo, lambda target, seed: solution)
    result = adapter.solve(PoseCommand(slider.fk(np.array([.2])), 0), [0.], now_ns=0)
    assert not result.success and isinstance(result.command, StopCommand)
    stopped = servo.step(JointState([0.], [.2], 0), result.command, dt=.01, now_ns=0)
    assert stopped.action == Action.BRAKE


def test_adapter_uses_custom_servo_limits_and_returns_independent_limit_copies(slider):
    limits = JointLimits(lower=[-.3], upper=[.3], velocity=[.5], acceleration=[1.], margin=.02)
    servo = Servo(slider, ServoConfig(task_axes=(0,)), limits)
    copied = servo.limits
    copied.upper[0] = 100
    assert servo.limits.upper[0] == pytest.approx(.3)
    adapter = PositionIKAdapter(servo, lambda target, seed: [.29])
    target = slider.fk(np.array([.29]))
    result = adapter.solve(PoseCommand(target, 0), [0.], now_ns=0)
    assert not result.success and "limits" in result.message


def test_solver_cannot_mutate_seed_or_target_and_adapter_does_not_replay_old_solution(slider):
    calls = 0
    def solver(target, seed):
        nonlocal calls
        calls += 1
        answer = np.array([target[0, 3]])
        target[:] = np.nan
        seed[:] = 100
        if calls > 1:
            raise RuntimeError("solver failed")
        return answer
    adapter = PositionIKAdapter(Servo(slider, ServoConfig(task_axes=(0,))), solver)
    target, seed = slider.fk(np.array([.2])), np.array([0.])
    original = target.copy()
    accepted = adapter.solve(PoseCommand(target, 0), seed, now_ns=0)
    assert accepted.success
    np.testing.assert_array_equal(target, original)
    np.testing.assert_array_equal(seed, [0.])
    failure = adapter.solve(PoseCommand(target, 10_000_000), seed, now_ns=10_000_000)
    assert not failure.success and isinstance(failure.command, StopCommand)
    assert "solver failed" in failure.message


def test_ik_checks_only_selected_task_axes_and_reports_residuals(slider):
    target = slider.fk(np.array([.2]))
    target[1, 3] = 100.
    adapter = PositionIKAdapter(Servo(slider, ServoConfig(task_axes=(0,))), lambda target, seed: [.2])
    result = adapter.solve(PoseCommand(target, 0), [0.], now_ns=0)
    assert result.success
    assert result.position_error == pytest.approx(0.)
    assert result.orientation_error == pytest.approx(0.)
    target[0, 3] += .01
    result = adapter.solve(PoseCommand(target, 0), [0.], now_ns=0)
    assert not result.success
    assert result.position_error == pytest.approx(.01)


def test_discontinuous_solution_is_rejected(slider):
    adapter = PositionIKAdapter(Servo(slider, ServoConfig(task_axes=(0,))),
                                lambda target, seed: [.4], max_joint_step=.3)
    result = adapter.solve(PoseCommand(slider.fk(np.array([.4])), 0), [0.], now_ns=0)
    assert not result.success and "max_joint_step" in result.message


@pytest.mark.parametrize("kwargs", [
    {"max_joint_step": 0}, {"max_joint_step": [1, 2]}, {"max_joint_step": np.nan},
    {"position_tolerance": 0}, {"orientation_tolerance": np.inf},
])
def test_adapter_rejects_invalid_configuration(slider, kwargs):
    with pytest.raises(ValueError):
        PositionIKAdapter(Servo(slider, ServoConfig(task_axes=(0,))), lambda target, seed: seed, **kwargs)
