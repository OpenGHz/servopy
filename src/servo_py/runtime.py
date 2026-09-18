"""Single-consumer periodic runner and explicit device stop/recovery contract.

Python scheduling is best effort. Devices must provide their own watchdog,
bounded I/O and emergency stop; a blocked driver cannot be preempted here.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Protocol

import numpy as np
from .api import Action, JointState, StopCommand


class Device(Protocol):
    def read_state(self, now_ns: int) -> JointState:
        """Return feedback with its acquisition timestamp in the same clock."""

    def write_reference(self, reference, *, duration: float, sample: Callable):
        """Accept the interval ending at reference.stamp_ns.

        sample(t) is valid until the next Servo.step. Copy needed samples
        synchronously, or implement equivalent interpolation in the device.
        Do not enqueue an unbounded sequence of endpoint positions.
        """

    def stop(self, reason: str):
        """Cancel device buffers and request its independent stop mechanism."""

    def recover(self):
        """Acknowledge/reset device faults; must not resume buffered commands."""


class LatestCommand:
    """Thread-safe, size-one mailbox. Source timestamps are never refreshed."""
    def __init__(self):
        self._lock = threading.Lock()
        self._command = StopCommand()
        self.replaced = 0

    def publish(self, command):
        snapshot = deepcopy(command)
        with self._lock:
            if not isinstance(self._command, StopCommand):
                self.replaced += 1
            self._command = snapshot

    def read(self):
        with self._lock:
            return deepcopy(self._command)

    def clear(self):
        self.publish(StopCommand())


class CallbackDevice:
    """Bind an SDK's feedback, send, buffer-cancel/stop and recovery functions."""
    def __init__(self, *, read_state, write_reference, stop, recover):
        for callback in (read_state, write_reference, stop, recover):
            if not callable(callback):
                raise TypeError("all device callbacks must be callable")
        self.read_state = read_state
        self.write_reference = write_reference
        self.stop = stop
        self.recover = recover


@dataclass
class RunnerStats:
    cycles: int = 0
    deadline_misses: int = 0
    max_lateness_ns: int = 0
    max_cycle_ns: int = 0
    reason: str = ""


class ServoRunner:
    def __init__(self, servo, device: Device, *, period=.01, max_lateness=None,
                 stop_timeout=5., commands=None, collision_source=None, recorder=None,
                 clock_ns=time.monotonic_ns, sleep=time.sleep):
        if not math.isfinite(period) or not 1e-9 <= period <= servo.config.max_dt:
            raise ValueError("period must be positive and no larger than Servo.max_dt")
        self.period_ns = round(period * 1e9)
        self.period = self.period_ns * 1e-9
        allowed = servo.config.timing_tolerance * self.period
        max_lateness = allowed if max_lateness is None else max_lateness
        if not math.isfinite(max_lateness) or not 0 <= max_lateness <= allowed:
            raise ValueError("max_lateness must fit within Servo.timing_tolerance")
        if not math.isfinite(stop_timeout) or stop_timeout <= 0:
            raise ValueError("stop_timeout must be positive")
        self.servo, self.device = servo, device
        self.commands = commands if commands is not None else LatestCommand()
        self.collision_source, self.recorder = collision_source, recorder
        self.clock_ns, self.sleep = clock_ns, sleep
        self.max_lateness_ns = round(max_lateness * 1e9)
        self.stop_timeout_ns = round(stop_timeout * 1e9)
        self.stats = RunnerStats()
        self._cancel = threading.Event()
        self._running = threading.Lock()
        self.faulted = False

    def cancel(self):
        """Discard pending targets and brake to rest at subsequent ticks."""
        self.commands.clear()
        self._cancel.set()

    def recover(self):
        if not self._running.acquire(blocking=False):
            raise RuntimeError("cannot recover a running loop")
        try:
            self.faulted = True
            self.commands.clear()
            self.device.recover()
            now = self.clock_ns()
            state = self.device.read_state(now)
            velocity = np.asarray(state.dq, dtype=float)
            if velocity.ndim != 1 or not velocity.size or not np.all(np.isfinite(velocity)) or np.max(np.abs(velocity)) > 1e-4:
                raise RuntimeError("recovery requires stopped joint feedback")
            self.servo.reset(state, now_ns=self.clock_ns())
            self._cancel.clear()
            self.faulted = False
        except BaseException:
            self.device.stop("recovery failed")
            raise
        finally:
            self._running.release()

    def run(self, *, max_steps=None):
        """Run until cancel + HOLD, or fault. max_steps starts controlled braking.

        Every deadline is absolute; late cycles are never replayed in a burst.
        A failure cancels the mailbox and invokes device.stop before propagating.
        """
        if max_steps is not None and (not isinstance(max_steps, int) or max_steps < 1):
            raise ValueError("max_steps must be a positive integer")
        if not self._running.acquire(blocking=False):
            raise RuntimeError("runner is already active")
        if self.faulted:
            self._running.release()
            raise RuntimeError("runner fault is latched; recover before running")
        self.stats = RunnerStats()
        deadline = self.clock_ns()
        stop_started = None
        try:
            while True:
                while self.clock_ns() < deadline:
                    self.sleep(max(0., (deadline - self.clock_ns()) * 1e-9))
                now = self.clock_ns()
                lateness = now - deadline
                self.stats.max_lateness_ns = max(self.stats.max_lateness_ns, lateness)
                if lateness > self.max_lateness_ns:
                    self.stats.deadline_misses += 1
                    raise TimeoutError("control deadline missed")
                if max_steps is not None and self.stats.cycles >= max_steps:
                    self.cancel()
                if self._cancel.is_set() and stop_started is None:
                    stop_started = now
                if stop_started is not None and now - stop_started > self.stop_timeout_ns:
                    raise TimeoutError("controlled stop timed out")
                state = self.device.read_state(now)
                now = self.clock_ns()
                if now - deadline > self.max_lateness_ns:
                    self.stats.deadline_misses += 1
                    raise TimeoutError("feedback I/O exceeded the lateness budget")
                command = StopCommand() if self._cancel.is_set() else self.commands.read()
                collision = self.collision_source(state, now) if self.collision_source else None
                result = self.servo.step(state, command, dt=self.period, now_ns=now, collision=collision)
                if result.action == Action.REJECT:
                    raise RuntimeError(f"Servo rejected: {result.message}")
                # Never send a reference after the computation/I/O budget expires.
                if self.clock_ns() >= deadline + self.period_ns:
                    self.stats.deadline_misses += 1
                    raise TimeoutError("reference computation exceeded the control interval")
                self.device.write_reference(result.reference, duration=self.period, sample=self.servo.sample_reference)
                if self.recorder is not None:
                    self.recorder.record(now, self.period, state, command, result, collision)
                elapsed = self.clock_ns() - now
                self.stats.max_cycle_ns = max(self.stats.max_cycle_ns, elapsed)
                if self.clock_ns() >= deadline + self.period_ns:
                    self.stats.deadline_misses += 1
                    raise TimeoutError("device I/O or recording exceeded the control interval")
                self.stats.cycles += 1
                if self._cancel.is_set() and result.action == Action.HOLD:
                    # HOLD refers to the reference. Require actual feedback to
                    # have stopped before clearing the downstream device queue.
                    if np.max(np.abs(state.dq)) <= 1e-4:
                        self.stats.reason = "cancelled after controlled stop"
                        break
                deadline += self.period_ns
        except BaseException as exc:
            self.faulted = True
            self.stats.reason = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self.commands.clear()
            try:
                self.device.stop(self.stats.reason)
            except BaseException:
                self.faulted = True
                raise
            finally:
                self._running.release()
        return self.stats


class SimulatedDevice:
    """Ideal reference-following driver for scheduler/adapter tests, no dynamics."""
    def __init__(self, q):
        self.q = np.asarray(q, dtype=float).copy()
        if self.q.ndim != 1 or not self.q.size or not np.all(np.isfinite(self.q)):
            raise ValueError("q must be a nonempty finite vector")
        self.dq = np.zeros_like(self.q)
        self._interval = None
        self.stopped = False
        self.stop_reason = ""

    def read_state(self, now_ns):
        if self._interval is not None:
            start, duration, sample = self._interval
            elapsed = min(max((now_ns - start) * 1e-9, 0.), duration)
            ref = sample(elapsed)
            self.q, self.dq = ref.q, ref.dq
        return JointState(self.q.copy(), self.dq.copy(), now_ns)

    def write_reference(self, reference, *, duration, sample):
        if self.stopped:
            raise RuntimeError("device must be recovered before accepting references")
        self._interval = (reference.stamp_ns - round(duration * 1e9), duration, sample)

    def stop(self, reason):
        self._interval = None
        self.dq[:] = 0
        self.stopped, self.stop_reason = True, reason

    def recover(self):
        self._interval = None
        self.stopped = False
