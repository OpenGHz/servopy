# Validation record

The versions below identify the environments used for each recorded run.
Declared build and dependency requirements are listed in the
[installation guide](getting-started.md#1-准备环境); the recorded versions are
not additional minimum or exact-version requirements.

Date: 2026-09-17. Implementation: ServoPy 0.1.0.

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

## Version 0.2.0: native joint targets and reusable position IK

Date: 2026-09-18. A new CPython 3.12 Linux wheel was built with GCC 13.3.0,
Eigen 3.4.1 and pybind11 3.1.0, limiting build concurrency to two. The restored
implementation was rebuilt and tested from the installed 0.2.0 wheel in a
dedicated virtual environment with NumPy 2.3.5, MuJoCo 3.13.0 and Pinocchio
4.1.0. All 128 Python tests passed. A separate Python-free C++ build and CTest
passed; its smoke test now exercises JOINT_POSITION convergence as well as
JointJog and braking.

The 42 new tests cover position convergence/reversal under reference limits,
invalid and stale targets, complete named mappings, collision scaling,
continuous-joint wrapping, actual arrival versus reference HOLD, configuration
validation, and position IK preparation. Adapter tests check source timestamp
preservation and expiration after a delayed solve, suppression of stale/invalid
requests, invalid solver outputs, custom Servo limits, input-copy isolation,
exceptions without stale-result replay, active-task residuals and continuity
bounds. The 14 Panda tests also pass after both position modes migrated to
the native command and shared adapter; each control mode completes an 18-second
dynamics run and ends in HOLD.

No new interactive viewer or physical-robot verification is claimed. Existing
position-actuator gravity offsets and the lack of jerk/geometric collision
checking still apply. Pending capabilities are tracked in `roadmap.md`.

## Version 0.3.0: smoothing, solver plugins and runtime

Date: 2026-09-18. A rebuilt 0.3.0 wheel was installed and all **178 Python
tests passed** with MuJoCo 3.13.0, Pinocchio 4.1.0 and optional Ruckig 0.12.2.
A separate Python-free C++ build and CTest passed. The latter now tests bounded
QP redistribution, native solver injection and reference interval sampling.
The source archive was inspected for the new native/Python modules, examples,
documentation and tests, and excludes build products and virtual environments.

The 50 added tests cover a QP oracle that independently enumerates active sets,
constraint redistribution, invalid solver outputs, seven-joint nullspace
posture under both Twist and already-reached Pose targets, finite-limit
centering, jerk continuity, target reversal, variable periods, position-boundary
stops, infeasible initial stops, command expiry, and generator reset failures.
A regression covers Ruckig 0.12's spurious position-extrema zero for a stationary
joint at a nonzero position. The wrapper instead computes analytic extrema
over the executed polynomial phases and validates the retained stopping path.

Runtime tests use an injected deterministic clock to exercise mailbox copy
ownership, timestamp preservation, cancellation, controlled stop, recovery,
stale feedback, failed reads/writes, late scheduling, slow feedback/recording,
and Ruckig device sampling. JSONL tests replay recorded state/commands exactly,
detect deliberately perturbed numerical references and reject misaligned
timestamps. JointTrajectory comparison uses synthetic exports and validates
joint names; it does not use an actual MoveIt execution.

All three Panda modes additionally ran 18 seconds with Ruckig, checking sampled
acceleration continuity, reference joint bounds, actual motion and final HOLD.
The torque run also enabled native QP and home-posture nullspace control.
Existing default-mode tracking regressions remain passing. A six-second CLI
run used scheduled external joint targets, Ruckig, JSONL recording and metrics;
it produced 600 records, reached HOLD, and its self-comparison had zero q/dq/ddq,
action and flag differences. Position actuator gravity offsets remain.
An additional six-second headless live-stdin trial sent joint targets at 20 Hz,
then closed the stream. It generated 600 records, moved the first reference
joint by 0.032 rad, cleared the target on EOF and finished in HOLD.

A best-effort wall-clock runner trial during concurrent build activity hit
its 5 ms lateness budget and stopped with TimeoutError. A subsequent trial
completed 101 cycles with no deadline misses, maximum observed lateness
1.396 ms and maximum cycle computation/I/O time 2.504 ms. These two observations
demonstrate the overload policy, not a real-time or hardware-frequency guarantee.

No new viewer/graphics, physical robot, vendor communication protocol,
geometric collision, position-mode gravity compensation, cross-platform build
or real MoveIt numerical-equivalence validation is claimed. Hardware SDK
callbacks and offline comparison tools are available for those environment-
dependent checks; their remaining scope is recorded in `roadmap.md`.

## Documentation refresh after 0.3.0

Date: 2026-09-18. The documentation refresh does not change control code or
supersede the functional test results above. The following documentation checks
were run locally on Linux / Python 3.12:

- `python scripts/check_docs.py --run` checked 27 Markdown files, local links
  and anchors, all 38 public exports, 27 ServoConfig defaults, 28 SafetyFlag
  names and 24 CLI options. All eight marked Python examples executed in
  isolated processes, including joint control, position IK, QP, Ruckig braking,
  deterministic scheduling, target-file creation and record/replay.
- The quickstart `track_pose.py` run completed 1,200 steps in HOLD with a final
  position error of 9.8116e-5 m, matching the documented expected output.
- The target-file workflow in [the recording tutorial](recording.md) ran eight
  seconds in Panda joint-position mode. It generated 800 JSONL records, reported
  no input error and finished in HOLD.
- MkDocs 1.6.1 / Material 9.7.7 built with `--strict`. The generated search index
  includes Chinese word segmentation via jieba 0.42.1. The source distribution
  contains the documentation, build configuration, checker and workflow, with
  generated site files excluded.

The [documentation workflow](https://github.com/OpenGHz/servopy/blob/main/.github/workflows/docs.yml)
runs the checker, Python examples and strict site build. This local validation
does not claim a completed GitHub Actions run or a deployed documentation site.

## ServoPy branding and README revision

Date: 2026-09-18. English is now the default README, with a separate Chinese
translation. Local Chromium previews checked both logo themes at 1,160 px and
English/Chinese layouts at 390 px: image loading, aspect ratio, header anchors,
language links and horizontal overflow. The preview uses GitHub-style Markdown
layout; it is not a screenshot of a deployed documentation site.

All nine marked Python examples pass, including the examples in both READMEs.
The source archive includes both languages, the legacy English entry and the
three SVG brand assets. The link checker now also checks picture srcset paths.

The Panda recording title was changed to ServoPy and the default torque demo
was rendered again through EGL: 18 seconds, 1,800 control steps, 960 × 640 at
30 fps, ending in HOLD. The refreshed GIF contains 216 frames at 640 × 427.
Tracking metrics match the previous recording within 1e-12; the control
implementation was not changed by this visual update.

## Linux distribution preparation

This section records the initial packaging build. Its bundled-Panda layout is
superseded by [the smaller distribution below](#panda-model-download-and-smaller-distributions).

Date: 2026-09-18. The Python distribution remains `servo-py` 0.3.0 and the
import name remains `servo_py`. Packaging now includes example modules, URDFs,
the pinned Panda archive, its manifest and licenses, plus a `servo-py-panda`
console entry point. Ruckig is pinned to 0.19.4 for this release preparation.

Local validation on Linux x86_64 / CPython 3.12:

- `python -m build` built the wheel from the generated sdist using GCC 13.3.0,
  scikit-build-core 1.0.3, pybind11 3.1.0 and cmeel-eigen 3.4.1.
- `twine check --strict` passed for both archives. The wheel contains the
  expected model resources and third-party license files.
- All **178 Python tests passed** against the installed wheel with NumPy 2.5.3,
  MuJoCo 3.13.0, Pinocchio 4.1.0 and Ruckig 0.19.4.
- `scripts/smoke_wheel.py --mujoco` checked version agreement, bundled model
  checksums and the installed entry point. From a temporary directory it ran
  the installed ideal-feedback example and default 18-second headless Panda
  demo, with final position errors below 0.1 mm and 1 mm respectively.
- All nine marked documentation examples and the strict MkDocs build passed.

The local wheel is a `linux_x86_64` development artifact. Portable release
wheels are built separately in manylinux_2_28 containers by the
[distribution workflow](https://github.com/OpenGHz/servopy/actions/workflows/release.yml).

The first complete [GitHub Actions distribution run](https://github.com/OpenGHz/servopy/actions/runs/35324779011)
passed on commit `4e28fefe84a28853d2a10281b4ee73ade028159c`:

| Environment | Installed-wheel result |
|---|---|
| manylinux_2_28 x86_64, each CPython 3.10–3.14 | 157 tests passed per wheel with Ruckig; 2 optional modules skipped; packaged example smoke check passed |
| manylinux_2_28 ARM64, each CPython 3.10–3.14 | 148 tests passed per wheel; 4 optional skips; packaged example smoke check passed |
| Ubuntu 22.04 x86_64 / Python 3.10 / NumPy 2.2.6 | 178 tests passed with MuJoCo 3.13.0, Pinocchio 4.1.0 and Ruckig 0.19.4 |
| Ubuntu 24.04 x86_64 / Python 3.12 / NumPy 2.5.3 | 178 tests passed with the same optional backend versions |
| Ubuntu 24.04 ARM64 / Python 3.12 / NumPy 2.5.3 | 166 tests passed with MuJoCo and Pinocchio; 5 Ruckig-related skips |

All three Ubuntu jobs installed the project and runtime dependencies using
`--only-binary=:all:`, passed `pip check`, and ran the installed headless Panda
entry point outside the checkout. Auditwheel produced x86_64 tags for
manylinux_2_27 / manylinux_2_28 and ARM64 tags for manylinux_2_26 /
manylinux_2_28. The advertised baseline remains glibc 2.28, including the
runtime dependency requirements.

The final gate checked all 10 wheels and the sdist, including versions,
platform tags, metadata, extension modules, models and license files. The
run's `publish-distributions` artifact contains the validated files. Its
TestPyPI and PyPI jobs were skipped because this was a main-branch build;
this record does not claim that the package has been uploaded to either index.
Account setup and publication steps are in [the release guide](publishing.md).

## Panda model download and smaller distributions

Date: 2026-09-18. At the user's request, both wheels and sdists now exclude
`examples/assets/panda.zip`. The example code, small URDFs, Panda license and
provenance manifest remain available. The installed Panda demo downloads the
archive from a fixed Git commit, verifies its SHA-256, and caches it outside
the package directory. A Git checkout or `SERVO_PY_PANDA_ARCHIVE` can supply
the same archive offline.

Local CPython 3.12 / Linux x86_64 validation:

- A wheel built from the sdist is about **0.42 MB**, down from **5.45 MB**
  (approximately **92% smaller**). Inspection confirmed that neither archive
  contains `panda.zip`; this comparison excludes runtime dependencies.
- **188 tests passed**, including ten new cases for local/offline archives,
  checksum failures, bounded downloads, cache reuse and repair, network
  errors, and temporary-file cleanup after a failed cache update.
- The installed-wheel smoke check ran from a temporary directory and an
  empty model cache. The basic example and Panda `--help` did not create a
  cache. The headless Panda run downloaded and verified the model, completed
  the default trajectory with less than 1 mm final position error, and a
  subsequent model load succeeded with the network function disabled.
- The nine executable documentation examples and strict MkDocs build passed.

The release gate now requires the manifest and model loader while rejecting
Panda ZIPs in either distribution. First use of the installed Panda demo
requires network access or a separately supplied archive; importing ServoPy
and using the core API do not fetch the model.

The updated [Linux distribution run](https://github.com/OpenGHz/servopy/actions/runs/35327133523)
passed for commit `96ac8bb6b6c6fb015308c119b55ea26c96c93728`. All five
x86_64 wheels are approximately **414–418 kB**; all five ARM64 wheels are
approximately **378–382 kB**. Each CPython 3.10–3.14 wheel passed its core
tests (167 passes on x86_64 with Ruckig, 158 on ARM64 without it). Ubuntu
22.04 / Python 3.10 and 24.04 / Python 3.12 on x86_64 each passed all 188
tests; Ubuntu 24.04 / Python 3.12 on ARM64 passed 176 tests with 5 optional
Ruckig-related skips. All three Ubuntu jobs passed the installed Panda
download/cache/offline checks. The complete-distribution gate passed, and
publishing jobs remained skipped on this main-branch build.

## TestPyPI publication and installation

Date: 2026-09-18. The maintainer-triggered [TestPyPI workflow](https://github.com/OpenGHz/servopy/actions/runs/35327922513)
completed successfully for commit `888f685467b802240ea0c8bc0cbc823402a7c287`.
After the build, Ubuntu installation and artifact gates passed, Trusted
Publishing uploaded all ten Linux wheels and the sdist to
[TestPyPI 0.3.0](https://test.pypi.org/project/servo-py/0.3.0/). The production
PyPI job was skipped, as configured for this event.

A new CPython 3.12 / Linux x86_64 virtual environment installed NumPy 2.5.3
and MuJoCo 3.13.0 from the production index, then installed only
`servo-py==0.3.0` from TestPyPI with `--only-binary=:all: --no-deps`.
The served CPython 3.12 x86_64 wheel is **417,750 bytes**. `pip check` passed.
The package was imported from this environment's site-packages, not the
checkout or a locally built wheel.

The installed-wheel smoke check passed from a temporary directory with a
fresh cache: the basic example completed 1,200 steps in HOLD with less than
0.1 mm final error; Panda `--help` did not download a model; the headless
Panda demo downloaded and verified the separate model, completed its
18-second trajectory with less than 1 mm final error, and reloaded the
cached model with network calls disabled. The installed package contains
no Panda ZIP. This validated the TestPyPI-served artifact; the subsequent
production publication is recorded below.

## PyPI publication and installation

Date: 2026-09-18. Publishing the GitHub Release `v0.3.0` triggered
[the production workflow](https://github.com/OpenGHz/servopy/actions/runs/35331455567)
for commit `9d11807a900cbcd438f254e7a8253b31695d630f`. All gates and the
PyPI publishing job succeeded. The [production PyPI release](https://pypi.org/project/servo-py/0.3.0/)
contains ten Linux wheels and one sdist.

A fresh CPython 3.12 / Linux x86_64 virtual environment installed
`servo-py[mujoco]==0.3.0` with `--only-binary=:all:` and an explicit
`https://pypi.org/simple` index. `pip check` passed. The installed-wheel smoke
check ran outside the checkout and passed the basic example, Panda entry
point, a full headless simulation, first-use model download and checksum,
and cached model loading with network calls disabled. The installed wheel
contains no Panda archive.

## Realtime control documentation

Date: 2026-09-18. Documentation now presents ServoPy as realtime joint and
Cartesian servo control: targets and measured feedback enter a continuous loop,
and each control cycle updates the next reference interval. The hard real-time
limitations in the execution contract remain explicit.

All ten marked Python documentation examples passed, including the new
`realtime-target-stream` example. Using an integer simulation clock and ideal
feedback, it publishes commands at 20 Hz into a 100 Hz control loop, changes
the requested direction during motion, then stops publishing. The example
checks both directions, command expiry, and final HOLD. It is a behavior
demonstration, not a wall-clock latency or hardware-frequency benchmark.

The documentation checker passed link, API, configuration, CLI and example
checks across 31 Markdown pages. The strict MkDocs build also passed.
