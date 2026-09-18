"""Optional Ruckig reference generation with a retained feasible stop trajectory."""
from __future__ import annotations

import numpy as np
from . import _core


class RuckigSmoothing(_core.MotionGenerator):
    """Jerk-limited motion backend, one instance per Servo.

    Each accepted interval has a validated continuation to rest. If a new
    target cannot preserve that property, execute the retained stop instead.
    Position extrema are solved analytically inside every executed jerk phase.
    """

    def __init__(self, max_jerk):
        super().__init__()
        try:
            import ruckig
        except ImportError as exc:
            raise ImportError("Ruckig smoothing requires pip install 'servo-py[ruckig]'") from exc
        self._r = ruckig
        self._jerk = np.asarray(max_jerk, dtype=float).copy()
        if self._jerk.ndim > 1 or not self._jerk.size or not np.all(np.isfinite(self._jerk)) or np.any(self._jerk <= 0):
            raise ValueError("max_jerk must be a positive finite scalar or vector")
        self.reset()

    def reset(self):
        self._brake = None
        self._segment = None
        self._dt = None

    def _calculate(self, start, velocity, position, limits):
        n = len(start.q)
        jerk = np.broadcast_to(self._jerk, (n,))
        inp = self._r.InputParameter(n)
        inp.current_position = start.q.tolist()
        inp.current_velocity = start.dq.tolist()
        inp.current_acceleration = start.ddq.tolist()
        inp.max_velocity = limits.velocity.tolist()
        inp.max_acceleration = limits.acceleration.tolist()
        inp.max_jerk = jerk.tolist()
        inp.target_acceleration = [0.] * n
        if position is None:
            inp.control_interface = self._r.ControlInterface.Velocity
            inp.target_velocity = np.asarray(velocity).tolist()
            inp.synchronization = self._r.Synchronization.No
        else:
            inp.target_position = np.asarray(position).tolist()
            inp.target_velocity = [0.] * n
            inp.synchronization = self._r.Synchronization.Time
        otg = self._r.Ruckig(n)
        if np.any(np.abs(start.dq) > limits.velocity + 1e-9) or np.any(np.abs(start.ddq) > limits.acceleration + 1e-9):
            raise ValueError("initial velocity or acceleration exceeds limits")
        # The library's exact current-envelope comparison can reject its own
        # output at vmax by a few ulps. Validate all continuous extrema below.
        if not otg.validate_input(inp, False, True):
            raise ValueError("Ruckig input is outside the feasible velocity/acceleration envelope")
        trajectory = self._r.Trajectory(n)
        status = otg.calculate(inp, trajectory)
        if status not in (self._r.Result.Working, self._r.Result.Finished):
            raise ValueError(f"Ruckig trajectory calculation failed: {status}")
        # Velocity control does not constrain position or max velocity. Check
        # velocity extrema analytically, including a(t)=0 inside jerk phases.
        for section in trajectory.profiles:
            for index, profile in enumerate(section):
                speeds = []
                for phases in (profile.brake, profile):
                    speeds.extend(phases.v)
                    for duration, acceleration, velocity0, phase_jerk in zip(phases.t, phases.a, phases.v, phases.j):
                        if not duration:
                            continue
                        speeds.append(velocity0 + acceleration * duration + .5 * phase_jerk * duration**2)
                        if max(abs(acceleration), abs(acceleration + phase_jerk * duration)) > limits.acceleration[index] + 1e-9:
                            raise ValueError("trajectory exceeds a joint acceleration limit")
                        if abs(phase_jerk) > jerk[index] + 1e-9:
                            raise ValueError("trajectory exceeds a joint jerk limit")
                        if phase_jerk:
                            t = -acceleration / phase_jerk
                            if 0 < t < duration:
                                speeds.append(velocity0 + acceleration * t + .5 * phase_jerk * t * t)
                if max(map(abs, speeds)) > limits.velocity[index] + 1e-9:
                    raise ValueError("trajectory exceeds a joint velocity limit")
        return trajectory

    @staticmethod
    def _at(trajectory, elapsed):
        q, dq, ddq = trajectory.at_time(elapsed)
        return _core.Reference(q, dq, ddq)

    @staticmethod
    def _inside(trajectory, lower, upper, horizon):
        # Ruckig 0.12's position_extrema can include a spurious zero for a
        # stationary velocity profile at a nonzero position. Compute exact
        # extrema of the executed prefix instead, solving v(t)=0 per phase.
        # Waypoint/cloud trajectories are never requested by this backend.
        if len(trajectory.profiles) != 1:
            raise ValueError("only local state-to-state trajectories are supported")
        start = np.asarray(trajectory.at_time(0.)[0])
        end = np.asarray(trajectory.at_time(horizon)[0])
        lo, hi = np.minimum(start, end), np.maximum(start, end)
        for i, profile in enumerate(trajectory.profiles[0]):
            elapsed = 0.
            for phases in (profile.brake, profile):
                for duration, p, v, a, j in zip(phases.t, phases.p, phases.v, phases.a, phases.j):
                    t = min(duration, max(0., horizon - elapsed))
                    elapsed += duration
                    if t <= 0:
                        continue
                    times = [0., t]
                    if abs(j) > 1e-15:
                        discriminant = a*a - 2*j*v
                        if discriminant >= 0:
                            root = np.sqrt(discriminant)
                            times.extend(x for x in ((-a-root)/j, (-a+root)/j) if 0 < x < t)
                    elif abs(a) > 1e-15 and 0 < -v/a < t:
                        times.append(-v/a)
                    values = [p + v*x + .5*a*x*x + j*x*x*x/6 for x in times]
                    lo[i], hi[i] = min(lo[i], *values), max(hi[i], *values)
        return bool(np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)) and
                    np.all(lo >= lower - 1e-10) and np.all(hi <= upper + 1e-10))

    def generate(self, start, velocity, position, dt, limits):
        lower = np.minimum(start.q, limits.lower + limits.margin)
        upper = np.maximum(start.q, limits.upper - limits.margin)
        zeros = np.zeros(len(start.q))
        if self._brake is None:
            initial = self._calculate(start, zeros, None, limits)
            if not self._inside(initial, lower, upper, initial.duration):
                raise ValueError("initial state has no validated jerk-limited stop inside joint limits")
            self._brake = (initial, 0.)
        fallback = False
        position_limited = False
        # A requested stop uses the already validated continuation, preserving
        # acceleration and avoiding changes caused by repeated synchronization.
        if position is None and np.max(np.abs(velocity)) <= 1e-12:
            trajectory, offset = self._brake
            self._brake = (trajectory, offset + dt)
        else:
            try:
                candidate = self._calculate(start, velocity, position, limits)
                end = self._at(candidate, dt)
                stop = self._calculate(end, zeros, None, limits)
                if not self._inside(candidate, lower, upper, dt) or not self._inside(stop, lower, upper, stop.duration):
                    position_limited = True
                    raise ValueError("new target would leave the feasible stopping envelope")
            except (ValueError, RuntimeError, self._r.RuckigError):
                trajectory, offset = self._brake
                self._brake = (trajectory, offset + dt)
                fallback = True
            else:
                trajectory, offset = candidate, 0.
                self._brake = (stop, 0.)
        self._segment = (trajectory, offset)
        self._dt = dt
        output = _core.MotionOutput()
        output.reference = self._at(trajectory, offset + dt)
        output.flags = (_core.JERK_LIMIT | (_core.SMOOTHING_FALLBACK if fallback else 0) |
                        (_core.POSITION_LIMIT if position_limited else 0))
        output.braking = fallback
        return output

    def sample(self, elapsed):
        if self._segment is None or not np.isfinite(elapsed) or not 0 <= elapsed <= self._dt:
            raise ValueError("sample time must lie in the last generated interval")
        trajectory, offset = self._segment
        return self._at(trajectory, offset + elapsed)
