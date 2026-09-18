# ServoPy

**Realtime robot control with C++ and Python.** ServoPy provides realtime joint and Cartesian servo control without ROS.

[Documentation](https://openghz.github.io/servopy/) · [Source](https://github.com/OpenGHz/servopy) · [中文介绍](https://github.com/OpenGHz/servopy/blob/main/README.zh-CN.md)

Stream joint positions, joint velocities, end-effector poses or twists while the robot moves. Each control cycle uses the latest target and measured joint feedback to update the next motion reference for your controller. Target sources can be teleoperation, vision feedback or other applications; the application supplies those inputs and the device connection.

Use `Servo.step()` in your own loop or `ServoRunner` for periodic feedback and output callbacks, a latest-target mailbox, cancellation and recovery. Native URDF or optional Pinocchio kinematics, DLS or bounded QP solvers, and optional Ruckig jerk control support these continuous control updates. NumPy is the only required third-party Python runtime dependency.

The [realtime servo guide](https://openghz.github.io/servopy/realtime-servo/) demonstrates changing targets during motion and braking when the command stream expires. Realtime describes the ongoing control loop; the Python runner uses best-effort scheduling and does not guarantee hard real-time deadlines.

## Install on Ubuntu

Prebuilt Linux wheels target **CPython 3.10–3.14**, **x86_64 / ARM64**, and **glibc 2.28 or newer**. A matching wheel needs no C++ compiler, CMake, Eigen installation or ROS. Ubuntu 22.04 and 24.04 provide suitable default Python versions; on Ubuntu 20.04, install Python 3.10 or newer in a separate environment first. Python 3.8/3.9, 32-bit and free-threaded builds are outside this wheel matrix.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install --only-binary=:all: servo-py
python -m servo_py.examples.track_pose
```

If Ubuntu reports that `venv` is missing, install its matching `python3-venv` package. The example uses bundled URDF data, runs without a checkout or display, and finishes in `HOLD` with a position error of about `9.8e-5 m`.

## Panda simulation

```bash
python -m pip install --only-binary=:all: 'servo-py[mujoco]'
servo-py-panda --control-mode joint-position
```

For a machine without a desktop, run `servo-py-panda --headless`. The default demo runs an 18-second simulation with a 100 Hz servo loop and 500 Hz physics. Choose `torque`, `joint-position` or `ik-position`; press Space to pause the viewer. Add `--target-stdin` to receive live JSONL targets while it runs; see the [streaming input guide](https://openghz.github.io/servopy/recording/#panda-外部目标与回放). Video recording also needs a working graphics backend.

The wheel and source distribution **do not contain Panda model assets**. On first use the demo downloads a fixed, approximately 5 MB archive, verifies its SHA-256, and caches it under `~/.cache/servo-py` (or `$XDG_CACHE_HOME/servo-py`). Further runs reuse the cache offline. Installing or importing ServoPy, the basic URDF example and `servo-py-panda --help` do not download this model. For offline setup, supply the pinned archive via `SERVO_PY_PANDA_ARCHIVE=/path/to/panda.zip`; see the [model setup guide](https://openghz.github.io/servopy/mujoco-panda/#模型下载与离线运行). The small provenance manifest and license are included in the package.

## Optional dependencies

- `servo-py[mujoco]`: simulation, viewer and video recording.
- `servo-py[pinocchio]`: Pinocchio kinematics.
- `servo-py[ruckig]`: Ruckig 0.19.4 trajectory smoothing. Its upstream Linux wheels currently cover x86_64; installing this extra on ARM64 requires an upstream source build.

Extras can be combined, for example `python -m pip install 'servo-py[mujoco,ruckig]'` on x86_64. See the [installation guide](https://openghz.github.io/servopy/getting-started/) and [Panda tutorial](https://openghz.github.io/servopy/mujoco-panda/).

## Source builds and scope

Source builds require Python 3.10 or newer, a compiler and standard library supporting C++17 or newer, CMake 3.20 or newer, and Eigen headers (provided by `cmeel-eigen>=3.4,<4` during isolated pip builds). C++17 is the minimum language standard.

The realtime servo loop outputs motion references for a downstream controller; your application owns feedback, actuator commands and device stopping. Physical-robot validation, geometric collision checking and hard real-time execution are outside the current verified scope. See the [design contract](https://openghz.github.io/servopy/design/) and [validation record](https://openghz.github.io/servopy/validation/).

ServoPy is [MIT-licensed](https://github.com/OpenGHz/servopy/blob/main/LICENSE). The separately downloaded Panda assets are Apache-2.0; see [NOTICE](https://github.com/OpenGHz/servopy/blob/main/NOTICE). This is an independent project and does not claim MoveIt compatibility.
