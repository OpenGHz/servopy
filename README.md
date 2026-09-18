<p align="center">
  <strong>English</strong> &nbsp;·&nbsp; <a href="README.zh-CN.md">简体中文</a>
</p>

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/assets/brand/logo-dark.svg">
    <img src="docs/assets/brand/logo-light.svg" alt="ServoPy" width="420">
  </picture>
</p>

<p align="center">
  <strong>From robot targets to controlled motion.</strong><br>
  C++17 core &nbsp;·&nbsp; Python API &nbsp;·&nbsp; ROS-independent
</p>

<p align="center">
  <a href="#quick-start">Quick start</a> &nbsp;·&nbsp;
  <a href="#panda-demo">Panda demo</a> &nbsp;·&nbsp;
  <a href="#documentation">Documentation</a>
</p>

---

**ServoPy** turns joint or Cartesian targets and measured feedback into bounded motion references. Use it to prototype robot controllers, connect an existing IK solver, or run closed-loop experiments in MuJoCo.

<p align="center">
  <a href="docs/media/panda-servo.mp4">
    <img src="docs/media/panda-servo.gif" alt="Panda following a figure-eight target in MuJoCo: orange target and cyan measured TCP path" width="640">
  </a>
</p>

<p align="center">
  Panda in MuJoCo · Recorded torque mode · 100 Hz servo / 500 Hz physics<br>
  <a href="docs/media/panda-servo.mp4">Watch the video</a> &nbsp;·&nbsp;
  <a href="docs/media/panda-servo.json">Measured results</a>
</p>

## What you can build

- **Control in joint or Cartesian space.** Send positions, velocities, poses or twists; bring your own position IK when needed.
- **Shape the motion.** Apply joint position, velocity and acceleration limits, with optional Ruckig jerk control.
- **Choose the numerical tools.** Use native URDF kinematics or Pinocchio, DLS or bounded QP, and optional nullspace posture objectives.
- **Connect and reproduce.** Run the Panda demo, bind a device SDK, stream targets, and record or replay control sessions.

The default Python runtime depends only on NumPy. MuJoCo, Pinocchio and Ruckig are optional.

## Quick start

You need **Python 3.10+** and a **C++17 compiler**. The commands below target the validated Linux environment; run them from a source checkout.

```bash
git clone https://github.com/OpenGHz/servopy.git
cd servopy
python -m venv .venv
source .venv/bin/activate
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install .
python examples/track_pose.py
```

No display or robot is needed. This ideal-feedback example runs **1,200 steps**, finishes in **HOLD**, and reports a final position error of approximately **9.8e-5 m**. It tests reference generation without simulating dynamics.

### Your first control step

Run this complete example from the repository root. `Servo.step()` consumes feedback and returns the next reference; your simulator or device adapter executes it.

<!-- runnable: readme-step -->
```python
from servo_py import (
    Action, JointPositionCommand, JointState, Servo, ServoConfig, load_urdf,
)

model = load_urdf(
    "examples/planar2.urdf", base="base", tip="tool",
    acceleration_limits=[3.0, 3.0],
)
servo = Servo(model, ServoConfig(task_axes=(0, 1)))
result = servo.step(
    JointState(q=[0.5, -1.0], dq=[0.0, 0.0], stamp_ns=0),
    JointPositionCommand(positions=[0.7, -0.7], stamp_ns=0),
    dt=0.01, now_ns=0,
)
if result.action == Action.REJECT:
    raise RuntimeError(result.message)
print(result.action.name, result.reference.q)
```

Expected output: `TRACK [ 0.50015 -0.99985]`. For execution timing and device-side stopping, read the [execution contract](docs/design.md).

## Panda demo

After the quick start, add MuJoCo and choose a control mode:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install '.[mujoco]'
python examples/mujoco_panda.py --control-mode joint-position
```

| `--control-mode` | Target → reference → actuator |
|---|---|
| `torque` (default) | Pose → differential IK → joint reference → torque control |
| `joint-position` | Joint target → joint reference → position actuator |
| `ik-position` | Pose → position IK → joint reference → position actuator |

The viewer runs an 18-second simulation. Press **Space** to pause, or add `--headless` to run without a display. Model assets are bundled with the source. The [Panda guide](docs/mujoco-panda.md) covers Ruckig smoothing, external targets, recording and measured tracking behavior.

## Documentation

**Guides and API reference are currently in Chinese.** The [design contract](docs/design.md) and [validation record](docs/validation.md) are in English; this README contains a complete English quick start.

| Next step | Read |
|---|---|
| Write a controller | [Joint control](docs/joint-position.md) · [Position IK](docs/python-ik.md) |
| Tune or extend it | [Smoothing](docs/smoothing.md) · [QP & nullspace](docs/solvers.md) · [C++](docs/cpp.md) |
| Connect and inspect | [Devices & scheduling](docs/runtime.md) · [Recording & replay](docs/recording.md) |
| Look up an interface | [API](docs/api.md) · [Configuration](docs/configuration.md) · [Troubleshooting](docs/troubleshooting.md) |

[Browse all documentation →](docs/index.md)

## Project status

Source version **0.3.0** was validated on **Linux x86_64 / Python 3.12** with 178 Python tests, a standalone C++ test and Panda dynamics in all three control modes. Conditions and results are preserved in the [validation record](docs/validation.md).

ServoPy generates references; the application owns feedback, actuator commands and device stopping. Physical-robot validation, geometric collision checking and hard real-time execution are outside the current verified scope. Position-mode Panda demos retain gravity-related tracking offsets. See the [roadmap](docs/roadmap.md) for capability boundaries.

## Contributing

See the [contribution guide](CONTRIBUTING.md) for setup, relevant tests and documentation checks, and the [changelog](CHANGELOG.md) for API migration notes. You can preview the searchable documentation locally with:

```bash
python -m pip install '.[docs]'
python scripts/check_docs.py
python -m mkdocs serve
```

## License

ServoPy is [MIT-licensed](LICENSE). Bundled Panda assets are Apache-2.0; their provenance and dependency notices are in [NOTICE](NOTICE). ServoPy is an independent project and does not claim MoveIt compatibility.
