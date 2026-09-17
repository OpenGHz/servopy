# Validation record

Date: 2026-09-17. Implementation: servo-py 0.1.0.

Development environment: Linux x86_64 / glibc 2.39, GCC 13.3.0,
Python 3.12.14, Eigen 3.4.1, pybind11 3.1.0, CMake 4.4.3,
scikit-build-core 1.0.3, NumPy 2.5.3 and optional Pinocchio 4.1.0.

The numerical suite covers known planar FK, finite-difference Jacobians,
fixed TCP offsets, rotated joint origins, prismatic and continuous joints,
and native/Pinocchio parity for two-axis, mixed and six-axis models. Servo
tests include full pose tracking, a 180-degree orientation target, task-axis
selection, tool/base expression equivalence, singularity halt/escape, limit
retreat, sampled stopping distance, mode switches, invalid values, command
and feedback expiry, tracking errors, collision sample freshness, latching,
reset, array ownership and monotonic timestamp precision.

72 Python tests passed with Pinocchio installed, running against the built
wheel. The standalone C++ CMake build and smoke test passed.

A fresh virtual environment without system site packages installed the wheel
with NumPy 2.5.3 and pytest only. The base suite passed 69 tests; the optional
Pinocchio test module was skipped. The pose replay also passed in that
environment. Dynamic linkage of the extension contains only the ordinary
C/C++ runtime libraries, with no ROS, MoveIt, Pinocchio or Eigen shared library.
NumPy 1.26 verification was attempted but its download timed out; no claim is
made that this older NumPy version was tested.

The source distribution was also built and installed with default build
isolation in the clean environment, using cached build dependencies and no
ROS, MoveIt or Pinocchio installation. Archive inspection confirms the C++
sources, headers, Python modules, examples, tests and licenses are included,
while build products and Python caches are excluded.

The planar Pose example replays 1200 steps with perfect feedback. It ends in
HOLD with a position error of approximately 9.81e-5 m. It has no physics,
sensor noise, communication delay or collision world.

`benchmark.json` contains one measured run of 5000 Python `Servo.step` calls,
after 200 warmup calls, using the native synthetic six-axis model and Twist
commands. Its percentiles include Python/native conversion and computation,
but exclude device I/O and model loading. Simulated timestamps are used;
the loop does not test periodic scheduling. Both the upper percentiles and
maximum are retained because wall-clock outliers matter. This run is not a
hardware control-frequency guarantee.

No physical robot, ROS/MoveIt runtime, geometric collision detector, jerk
limiter or upstream numerical-equivalence test was exercised. Real-device
integration still requires its own feedback timing, buffer cancellation and
stop contract.

## Panda MuJoCo example

The optional Panda example was tested with MuJoCo 3.13.0 on the Linux /
Python 3.12 environment above. The expanded suite passed 75 tests with both
Pinocchio and MuJoCo installed. The base environment still passed 69 tests,
with the two optional backend modules skipped.

The three added tests cover the Panda TCP Jacobian against finite differences,
isolation of kinematics probes from live physics, and a complete 18-second
dynamics run. The latter checks reference position/velocity/acceleration
limits, measured joint positions, actual motion, and final HOLD. Simulation
feedback comes from integrated MuJoCo qpos/qvel, not reference replay.

The default interactive viewer completed the same 1800 Servo steps using
GLFW with an Xvfb virtual X11 display, and exited with code 0. The rendering
threads are joined after closing the passive viewer to avoid interpreter /
GLFW shutdown races. A window screenshot was inspected. This is a virtual
display check, not a test of every desktop GPU/driver or manual interaction.

The committed MP4 was rendered using EGL: 18 seconds, 540 frames, 960 x 640,
30 fps, H.264. The README GIF contains 216 frames at 640 x 427. Beginning,
middle and ending video frames and a GIF frame were inspected. The recorded
trajectory had 7.04 mm TCP position RMSE, 17.62 mm maximum error and 0.077 mm
final error; final action was HOLD. See `media/panda-servo.json` for the
unrounded measured results. These are simulation tracking results, not
physical-robot accuracy specifications.

Source-distribution inspection confirms inclusion of the demo, pinned Panda
asset archive, provenance checksums, Apache-2.0 license, documentation,
recordings and optional tests. The basic package remains independent of
MuJoCo; no geometry collision monitor or robot driver was added to Servo.

## Panda joint-position modes

The `joint-position` and `ik-position` options were tested with the same
MuJoCo 3.13.0 / Python 3.12 environment. All 86 Python tests passed, including
14 Panda tests. The latter exercise all three modes for 18 seconds, check
reference limits, measured joint limits and actuator force bounds, and verify
final HOLD. Direct joint mode is checked without IK/Jacobian calls. Further
checks cover position-actuator angle semantics, full-pose Python IK and an
unreachable target, braking on six types of failed/invalid external IK output,
and avoiding a zero-angle command on REJECT.

Both added modes send interpolated joint angles to the original Menagerie
position actuators. They retain the original PD gains and have no bias or
desired-velocity feedforward, so gravity produces a steady-state offset.
The 18-second direct-joint run had 16.76 mm TCP RMSE and 6.725 mm final error;
the position-IK run had 15.45 mm RMSE, 6.722 mm final error and zero IK failures.
Their target paths differ; these are not directly comparable solver benchmarks.
The original torque run reproduced its previous metrics exactly.

Both modes also completed the minimum 6-second duration without REJECT.
At that deliberately fast duration the position-IK continuity check rejected
331 solutions, and final TCP error was 180.5 mm. The documentation therefore
calls out that completion/HOLD does not mean successful target tracking,
and recommends the default 18-second demonstration.

The new IK-position mode also completed an EGL recording: 18 seconds,
216 frames, 640 x 480 at 12 fps, with identical tracking metrics. A middle
frame was inspected for the mode label and target/measured paths. This was
a temporary verification recording; the existing README media still show
the default torque mode. A fresh interactive-viewer check could not be
completed in this session because the virtual X server could not create
its listening socket. The previous viewer check above is for torque mode;
it is not claimed as a new position-mode desktop validation.
