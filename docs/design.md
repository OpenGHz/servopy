# Execution contract

The native core uses C++ and Eigen, with C++17 as the minimum language standard.
Build requirements and tested environments are described in the
[installation guide](getting-started.md#1-准备环境) and [validation record](validation.md).
It has no ROS headers, clocks,
threads, transport or plugin loading. The Python facade maps typed commands
and configuration into native values; pybind11 releases the GIL during core
step/reset calls. A Python implementation of Kinematics reacquires the GIL
for each callback. The native SerialChainModel needs no such callback.

One Servo instance has one consumer. Concurrent calls on its native core
raise an error instead of waiting on the core mutex. The Python facade and
user-owned state/configuration must also not be mutated concurrently. Native
models are immutable after construction; the Pinocchio adapter locks its
mutable work data. This release uses dynamic allocations and is not a hard
real-time or allocation-free implementation.

All model coordinates are scalar revolute/prismatic coordinates (`nq == nv`
at the servo boundary). A backend must implement FK and geometric Jacobians
at the configured TCP, expressed in the fixed base axes, with linear rows
first. Its integrate/difference methods must respect the same scalar joint
representation. Continuous joints use unwrapped integration and shortest
angular tracking errors. General floating-base manifolds are outside scope.

The current measured q is used for FK, Jacobians and pose feedback. A separate
reference q/dq is propagated from the preceding accepted output. References
are never used to overwrite measured feedback. Configuration integration and
singularity probes operate on separate values. A max-absolute joint tracking
error larger than the configured threshold is a latched fault.

All timestamps are signed 64-bit nonnegative nanoseconds in one monotonic
time domain. The core reads no clock. Incoming command, state and collision
samples have separate expiration checks. Collision samples also identify the
timestamp of the state used for their calculation; both that timestamp and
the result timestamp must be recent. This checks age, not geometric state
equivalence or a swept path.

`dt` is the duration of the next planned reference interval in seconds. Each
output is scheduled at `now_ns + round(dt * 1e9)`. The next call should occur
at that endpoint; calls outside `timing_tolerance * previous_dt` are rejected.
Tolerance permits bounded caller jitter; it does not perform latency
prediction or fill gaps. A fixed period is preferred. Changing the period
can make previously feasible sampled braking infeasible, in which case the
core rejects. The maximum configurable interval is one second; the default
`max_dt` is 50 ms. Clock repeats, backwards time and timestamp overflow fault.

With the default motion backend, each accepted interval has constant acceleration. With previous endpoint
velocity v0, new endpoint velocity v1 and period h:

    a = (v1 - v0) / h
    q1 = integrate(q0, (v0 + v1) * h / 2)

The returned dq is the endpoint velocity, not the interval-average velocity.
An adapter must either use this reference with compatible interpolation or
document its own controller behavior. A generic position-only device will
not automatically reproduce the planned acceleration.

Desired velocity is constrained using a per-joint feasible interval:

1. Joint velocity bounds.
2. Endpoint velocity changes bounded by a_max * h.
3. Position bounds including the current interval and a future sampled stop.

For speed v >= 0, let r be the remainder of v divided by a_max * h. The future
stopping distance with full-period constant-acceleration segments is:

    d_stop = v² / (2 a_max) + r * (a_max * h - r) / (2 a_max)

The second term accounts for the final interval. Omitting it would assume a
mid-interval stop and can force unwanted boundary reversals. The solver also
checks within-interval extrema if velocity changes sign. Monotone scalar
searches find the feasible endpoint interval; empty intersections are faults.
When initialized within a soft position margin, motion may retreat but may
not progress further outward. A state outside physical position limits is
rejected. Constraints govern the generated reference, not unmodeled actuator
dynamics or communication delays. The default backend imposes no jerk bound; the optional Ruckig backend has a different piecewise-jerk interpolation contract described below.

Twist commands denote TCP linear/angular velocity, expressed in base or TCP
axes. Changing expression axes rotates both three-vectors; it does not change
their physical reference point. Arbitrary external frames and other reference
points are rejected. Pose targets are base-expressed rigid 4x4 transforms.
Their feedback law uses position difference and the shortest SO(3) rotation
logarithm, separate gains, active-axis selection and Cartesian speed caps.
Reaching active-task tolerances requests braking before holding by default.
When nullspace gains are enabled, the primary task stops requesting motion
inside tolerance but the secondary posture may continue (GOAL_REACHED + TRACK).

JointPosition commands contain a finite target for every controlled joint,
within effective limits including margins. Named Python commands must be a
complete permutation; missing joints never default to zero. The native law
uses joint_position_gain * difference(target, previous reference), then the
same velocity, acceleration and sampled stopping constraints. Continuous joints
use shortest angular error. A reference inside joint tolerance requests braking;
GOAL_REACHED additionally requires measured position error inside tolerance.
Neither that flag nor HOLD alone proves the physical device has stopped.
Diagnostics expose target-to-measured joint_position_error separately from
reference-to-measured tracking_error. No Cartesian speed cap, Jacobian
singularity policy is applied in this joint branch. Optional Ruckig smoothing applies joint jerk bounds here as in the other branches.

PositionIKAdapter is an explicit Python command preparation layer. It takes a
PoseCommand and caller-selected seed, validates solver results against the
Servo's effective limits, optional joint-step bounds and active-task FK residuals,
and returns JointPositionCommand or StopCommand. It preserves source timestamps,
never replays old successful results and reads no clock. The caller must invoke
Servo.step even on failure and supply fresh time after solving. The adapter
cannot interrupt a blocking solver or execute device stops; it does not replace
the Pose/Twist differential solver in the core.

Weighted task Jacobians use configurable selected rows. SVD implements damped
least squares with factors s / (s² + lambda²). Damping increases smoothly as
the smallest singular value drops. A small configuration probe along the
requested joint motion tests whether that singular value improves. Improving
motion is allowed; other motion is slowed between soft and hard thresholds,
and requests braking at the hard threshold. This is a local heuristic, not a
global escape guarantee. Thresholds depend on model units, task weights and
geometry, and must be tuned. Acceleration and position constraints can alter
the requested Cartesian direction.

Malformed or stale commands request braking while valid, fresh state remains
available. Invalid feedback, missing required collision results, a zero
collision scale, infeasible reference constraints, timing errors and backend
failures return REJECT with no reference and latch. The caller must handle
that result by cancelling any queued trajectory and invoking its controller's
stop procedure. The kernel has no device I/O and cannot perform this action.
Reset requires a fresh state within physical position and velocity limits.
It reinitializes reference history and clears the latch.

Invalid configuration raises at construction. Python argument-conversion
errors (for example a non-vector state array or an integer outside int64)
and concurrent-use errors can raise before a Result is produced. A device
adapter must route both REJECT results and exceptions to its stop procedure.

Collision input in this release is an externally computed target velocity
scale plus timestamps. A partial scale is applied before acceleration/position
constraints; actual output may take several cycles to decelerate. Unknown,
stale and disabled collision checking are explicit states. Geometry loading,
distance queries, collision pairs and collision-free stopping trajectories
remain future backend work.

The core intentionally differs from MoveIt Servo: explicit timing and state,
no IK-plugin dependency to identify base/TCP, DLS, sampled acceleration-limited
references, local singularity escape and latched faults. Upstream review
baseline is recorded in NOTICE. It is not a drop-in numerical replacement.

## Optional solvers, smoothing and runtime (0.3.0)

ServoCore accepts DifferentialIK and MotionGenerator implementations at construction.
The default DLS behavior remains. BoxQPSolver minimizes weighted task error plus
positive damping regularization subject to feasible per-joint velocity bounds.
An exact SVD nullspace projector supplies posture and finite-limit centering
preferences. Constraints and subsequent singularity/scaling operations can change
the task direction; this is not a general inequality or strict hierarchy solver.
SOLVER_ERROR latches on exceptions, dimension errors or invalid solver output.

RuckigSmoothing is an optional Python MotionGenerator backed by Ruckig 0.19.4.
JointPosition uses a position target, other branches use target velocities.
It checks continuous position/velocity extrema and acceleration/jerk phases,
and retains a feasible continuation to zero velocity and acceleration for each
accepted endpoint. New targets outside this envelope fall back to the retained
stop. Infeasible initial stops reject with SMOOTHING_ERROR. There is no post-hoc
filter applied to a separately propagated core reference. Each generator belongs
to one Servo. Custom generators are trusted backends responsible for their whole
trajectory contract; the core additionally checks endpoint dimensions and limits.

Use sample_reference(t) within the last accepted [0, dt] interval to execute
piecewise jerk trajectories. Endpoint ddq is not constant throughout a Ruckig
interval. Reset discards history and assumes zero initial reference acceleration;
recovery should occur after the device has stopped. Neither reset nor REJECT
allows sampling the previous interval. Position-only hardware must implement
compatible interpolation to reproduce reference constraints.

The separate Python ServoRunner uses an absolute monotonic schedule, a size-one
command mailbox and an explicit Device stop/recover protocol. It rejects late
work instead of replaying missed cycles, routes exceptions to downstream stop,
and requires explicit recovery. This best-effort loop cannot preempt blocked
callbacks and requires device-side watchdogs and bounded communication. See
advanced-control.md for the API, JSONL replay and offline MoveIt export comparison.
