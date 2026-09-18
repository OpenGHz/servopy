import numpy as np
import pytest
from servo_py import (Action, JointJogCommand, LatestCommand, RuckigSmoothing, Servo,
                      ServoConfig, ServoRunner, SimulatedDevice, StopCommand)


class Clock:
    def __init__(self):
        self.now = 0
        self.sleeps = []
    def clock(self):
        return self.now
    def sleep(self, duration):
        self.sleeps.append(duration)
        self.now += round(duration * 1e9)


def setup(slider, **kwargs):
    clock = Clock()
    device = SimulatedDevice([0])
    servo = Servo(slider, ServoConfig(task_axes=(0,)))
    runner = ServoRunner(servo, device, clock_ns=clock.clock, sleep=clock.sleep, **kwargs)
    return runner, device, clock


def test_mailbox_owns_arrays_and_keeps_source_timestamp():
    mailbox = LatestCommand()
    values = np.array([.5])
    mailbox.publish(JointJogCommand(values, 123))
    values[0] = 8
    read = mailbox.read()
    assert read.velocities[0] == .5 and read.stamp_ns == 123
    read.velocities[0] = 7
    assert mailbox.read().velocities[0] == .5
    mailbox.clear()
    assert isinstance(mailbox.read(), StopCommand)


def test_runner_stale_target_stops_and_explicit_recovery(slider):
    runner, device, clock = setup(slider)
    runner.commands.publish(JointJogCommand([.5], 0))
    stats = runner.run(max_steps=30)
    assert stats.cycles >= 31
    assert stats.deadline_misses == 0
    assert device.q[0] > 0 and device.dq[0] == 0
    assert device.stopped
    assert all(t == pytest.approx(.01) for t in clock.sleeps)
    runner.recover()
    assert not device.stopped and not runner.faulted
    assert runner.run(max_steps=1).cycles == 2


def test_cancel_discards_targets_and_brakes(slider):
    runner, device, clock = setup(slider)
    original = device.read_state
    def read(now):
        if now >= 40_000_000:
            runner.cancel()
        return original(now)
    device.read_state = read
    runner.commands.publish(JointJogCommand([.8], 0))
    stats = runner.run()
    assert stats.cycles > 5
    assert isinstance(runner.commands.read(), StopCommand)
    assert not runner.faulted and device.stopped


@pytest.mark.parametrize("fault", ["read", "write", "stale", "slow", "late"])
def test_driver_and_deadline_failures_stop_and_latch(slider, fault):
    runner, device, clock = setup(slider)
    original_read, original_write = device.read_state, device.write_reference
    writes = []
    def read(now):
        if fault == "read":
            raise OSError("feedback disconnected")
        if fault == "slow":
            clock.now += 20_000_000
        state = original_read(now)
        if fault == "stale":
            from dataclasses import replace
            return replace(state, stamp_ns=0)
        return state
    def write(*args, **kwargs):
        writes.append(clock.now)
        if fault == "write":
            raise OSError("send failed")
        return original_write(*args, **kwargs)
    device.read_state, device.write_reference = read, write
    if fault == "stale":
        clock.now = 1_000_000_000
    if fault == "late":
        runner.sleep = lambda duration: setattr(clock, "now", clock.now + round(duration * 1e9) + 8_000_000)
    with pytest.raises((OSError, RuntimeError, TimeoutError)):
        runner.run(max_steps=2)
    assert device.stopped and runner.faulted
    assert len(writes) <= 1  # No catch-up burst or send after an expired budget.
    with pytest.raises(RuntimeError, match="latched"):
        runner.run(max_steps=1)


def test_slow_recording_counts_as_overload(slider):
    runner, device, clock = setup(slider)
    class Recorder:
        def record(self, *args):
            clock.now += 20_000_000
    runner.recorder = Recorder()
    with pytest.raises(TimeoutError, match="recording"):
        runner.run(max_steps=1)
    assert runner.stats.deadline_misses == 1 and device.stopped


def test_runner_ruckig_device_sampling(slider):
    pytest.importorskip("ruckig")
    runner, device, clock = setup(slider)
    runner.servo = Servo(slider, ServoConfig(task_axes=(0,)), motion_generator=RuckigSmoothing(10))
    runner.commands.publish(JointJogCommand([.4], 0))
    assert runner.run(max_steps=50).cycles == 51
    assert device.q[0] > 0 and device.dq[0] == 0


def test_recovery_requires_stopped_device(slider):
    from servo_py import JointState
    runner, device, clock = setup(slider)
    device.read_state = lambda now: JointState([0], [.1], now)
    with pytest.raises(RuntimeError, match="stopped"):
        runner.recover()
    assert runner.faulted and device.stopped
