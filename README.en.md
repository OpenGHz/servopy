# servo-py

**Turn joint or Cartesian targets into continuous motion references.**

A ROS-independent servo kernel in C++17 / Eigen, with a Python API for models,
commands and device integration. Use it for robot-control prototypes, IK
experiments and simulation. The base Python runtime depends only on NumPy.

[中文](README.md) · [Documentation](docs/index.md) · [Panda guide](docs/mujoco-panda.md) · [API reference](docs/api.md)

![Panda tracking a figure-eight target in MuJoCo](docs/media/panda-servo.gif)

*Recorded MuJoCo dynamics, default torque mode. Amber: target; cyan: measured TCP.*

## Quick start

These commands target the validated Linux environment. Source builds require
Python 3.10+ and a C++17 compiler; access to the source repository is required.

```bash
git clone https://github.com/OpenGHz/servopy.git
cd servopy
python -m venv .venv
source .venv/bin/activate
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install .
python examples/track_pose.py
```

The example needs no display. It runs 1,200 ideal-feedback steps, finishes in
`HOLD`, and reports a final position error of approximately `9.8e-5 m`.

## Run Panda

```bash
python -m pip install '.[mujoco]'
python examples/mujoco_panda.py
```

Add `--headless` for a machine without a display. The model and meshes are
bundled in the source checkout. Press Space to pause; close the window to exit.

| Goal | Interface or option | Guide |
|---|---|---|
| Joint position / velocity targets | `JointPositionCommand`, `JointJogCommand` | [Joint control](docs/joint-position.md) |
| Cartesian pose / velocity targets | `PoseCommand`, `TwistCommand` | [Conventions](docs/concepts.md) |
| Use an existing position IK solver | `PositionIKAdapter` | [Position IK](docs/python-ik.md) |
| Jerk-limited reference generation | `RuckigSmoothing` | [Smoothing](docs/smoothing.md) |
| Bounded differential IK and posture control | `BoxQPSolver`, nullspace gains | [Solvers](docs/solvers.md) |
| Periodic execution and SDK callbacks | `ServoRunner`, `CallbackDevice` | [Runtime](docs/runtime.md) |
| Record, replay and compare trajectories | JSONL and JointTrajectory tools | [Recording](docs/recording.md) |

The Panda demo supports three modes:

```bash
python examples/mujoco_panda.py --control-mode torque
python examples/mujoco_panda.py --control-mode joint-position
python examples/mujoco_panda.py --control-mode ik-position
```

The first uses Cartesian differential IK and torque actuation. The second
sends joint targets to position actuators. The third solves position IK before
generating joint-position references. Optional smoothing works in all three:

```bash
python -m pip install '.[mujoco,ruckig]'
python examples/mujoco_panda.py --control-mode ik-position --smoothing ruckig
```

## Execution contract

`Servo.step(state, command, dt=..., now_ns=...)` returns a reference at the end
of the next interval. The caller supplies real feedback, consistent monotonic
timestamps and device I/O. `sample_reference(t)` samples the accepted interval,
including piecewise jerk phases when Ruckig is enabled.

`TRACK`, `BRAKE` and `HOLD` carry references. `REJECT` has no reference and
latches a fault until explicit recovery. A device adapter must cancel queued
motion and invoke its own stop mechanism on rejection or exceptions. See the
[English execution contract](docs/design.md) for the mathematical details.

Version **0.3.0** was validated on Linux x86_64 / Python 3.12 with 178 Python
tests, a standalone C++ test, and Panda dynamics in all three modes. These are
reference-generation and simulation results, not physical-robot validation.
There is no geometric collision checker or hard real-time guarantee; the
position actuators retain their gravity-related steady-state offset. Other
platforms and actual MoveIt numerical parity remain unverified.

## Contribute

Read the [contribution guide](CONTRIBUTING.md), [changelog](CHANGELOG.md) and
[validation record](docs/validation.md). Task tutorials and API guides are
currently maintained in Chinese; this page and the execution/validation
documents provide English entry points.

MIT-licensed. Bundled Panda assets are Apache-2.0. See [LICENSE](LICENSE) and
[NOTICE](NOTICE) for provenance and dependency notices.
