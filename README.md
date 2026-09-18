# servo-py

独立于 ROS/MoveIt 的在线 Servo 包，使用 C++17/Eigen 计算，提供 Python API。
当前版本：`0.3.0`。基础 Python 运行依赖只有 NumPy。

已实现 JointJog、JointPosition、Twist、Pose、阻尼微分 IK、奇异性减速/离开策略、关节速度与加速度约束、考虑采样制动距离的位置限制、超时停止和故障锁存。附带原生串联运动学、URDF 加载、可选 Pinocchio 后端与外部位置 IK 适配器。

这是初版参考实现，尚未真机验证。它对生成的参考施加约束；实际机器人制动、通信超时和轨迹缓冲取消由设备控制器负责。碰撞功能目前接受外部检查结果，不包含几何碰撞检测器。

**安装与运行**

从本项目源码根目录安装：

```bash
python -m pip install .
python examples/track_pose.py
```

源码构建需要 C++17 编译器；pip 会在隔离构建环境中安装构建依赖。可用的 Python wheel 可以直接安装，无需编译器：

```bash
python -m pip install ./servo_py-0.3.0-cp312-cp312-linux_x86_64.whl
```

本地构建的 wheel 仅对应 CPython 3.12、Linux x86_64，在 Ubuntu 24.04/glibc 2.39 环境构建，未经 manylinux 修复；其他平台或较老系统请从源码构建。项目声明支持 Python 3.10+，当前实际验证环境为 Python 3.12。`0.3.0` 增加了 C++ 求解器、轨迹生成接口与绑定，更新源码后需重新安装，旧版 wheel 不提供新接口：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install --upgrade '.[mujoco]'
```

**Panda + MuJoCo 仿真**

![Panda 在 MuJoCo 中使用 servo-py 跟踪空间八字轨迹](docs/media/panda-servo.gif)

[完整 MP4 录制](docs/media/panda-servo.mp4) · [运行与实现说明](docs/mujoco-panda.md) · [本次录制指标](docs/media/panda-servo.json)

从源码根目录运行，**默认打开 MuJoCo viewer**，演示 18 秒后自动结束；空格暂停/继续，关闭窗口退出。模型和 mesh 已随仓库提供，无需运行时下载。

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install '.[mujoco]'
python examples/mujoco_panda.py

# 直接给定七个关节的目标位置，使用 Panda 位置执行器。
python examples/mujoco_panda.py --control-mode joint-position

# 末端位姿 → Python IK → 关节目标 → Panda 位置执行器。
python examples/mujoco_panda.py --control-mode ik-position
```

示例以 100 Hz 调用 Servo，使用 MuJoCo 的关节位置和速度反馈，以 500 Hz 插值参考并推进物理。默认 `--control-mode torque` 使用力矩控制，末端保持朝向、跟踪空间八字轨迹；`joint-position` 直接跟踪关节轨迹，`ik-position` 先用 Python 位置 IK 求解同一八字位姿轨迹，再进行关节位控。两种位控均通过原生 `JointPositionCommand` 使用 Servo 的关节约束，将关节角写入模型原有位置执行器的 `ctrl`，最后发送停止命令。

橙色为目标路径，青色为实际末端轨迹。上方动画仍对应默认力矩模式。位控保留模型原始 PD 增益，未添加重力补偿，因此会有静态误差。模式区别、实测指标和替换自己的 Python IK 见 [示例文档](docs/mujoco-panda.md#三种控制模式)。

在 Linux 无桌面环境中录制：

```bash
MUJOCO_GL=egl python examples/mujoco_panda.py --headless \
  --record panda-servo.mp4 --metrics panda-servo.json
```

MuJoCo 是可选依赖；基础包仍仅依赖 NumPy。三个模式均未启用 Servo 的外部碰撞监控；两种关节位控模式不执行内置笛卡尔限速或奇异性减速。macOS 的 viewer 请使用 `mjpython examples/mujoco_panda.py`，同样支持 `--control-mode`。

**可选平滑与冗余控制**

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install --upgrade '.[mujoco,ruckig]'
python examples/mujoco_panda.py --control-mode joint-position --smoothing ruckig
python examples/mujoco_panda.py --control-mode ik-position --smoothing ruckig
python examples/mujoco_panda.py --differential-ik qp --nullspace-gain 0.1 --smoothing ruckig
```

新增 `--target-stdin` 实时目标、`--targets` 定时目标回放及 `--log` 控制记录。QP 和零空间参数用于 Pose/Twist 微分 IK；两种位控模式可以单独启用 Ruckig。用法与边界见 [高级控制接口](docs/advanced-control.md)。

**最小示例**

以下代码使用仓库内的二维机械臂。关节速度为 rad/s 或 m/s，Twist 的线速度为 m/s、角速度为 rad/s。

```python
import numpy as np
from servo_py import Servo, ServoConfig, JointState, TwistCommand, Action, load_urdf

model = load_urdf(
    "examples/planar2.urdf",
    base="base", tip="tool",
    acceleration_limits=[3.0, 3.0],
)
servo = Servo(model, ServoConfig(task_axes=(0, 1)))

state = JointState(q=[0.5, -0.8], dq=[0.0, 0.0], stamp_ns=0)
command = TwistCommand(
    linear=[0.02, 0.0, 0.0], angular=[0.0, 0.0, 0.0],
    stamp_ns=0, expressed_in="base", reference_point="tool",
)
result = servo.step(state, command, dt=0.01, now_ns=0)

if result.action == Action.REJECT:
    # 实机适配器在此取消旧参考并请求控制器停止。
    raise RuntimeError(result.message)
print(result.reference.q, result.reference.dq, result.flags)
```

实际循环中，应持续传入真实关节反馈与同一单调时间域的时间戳。`examples/track_pose.py` 把上一次参考作为下一步反馈，演示的是理想跟踪回放，未模拟动力学和传输时延。

使用完整六维任务时可保留默认 `ServoConfig()`；`task_axes` 顺序对应 `[vx, vy, vz, wx, wy, wz]`。任务维数不得大于模型自由度。`examples/arm6.urdf` 是自带的合成六轴模型，不代表任何真实设备。

**其他命令与状态**

```python
from servo_py import JointJogCommand, JointPositionCommand, PoseCommand, StopCommand

jog = JointJogCommand(velocities=[0.1], names=["elbow"], stamp_ns=now_ns)
# 必须包含所有受控关节，顺序与 model.joint_names() 一致。
joint_position = JointPositionCommand(positions=target_joint_angles, stamp_ns=now_ns)
pose = PoseCommand(pose=target_transform_4x4, stamp_ns=now_ns)
stop = StopCommand()
```

`JointPositionCommand` 直接接收关节目标，由 C++ 使用 `joint_position_gain` 和 `joint_position_tolerance` 生成受约束参考，支持连续关节最短角差。带名称的指令必须覆盖全部关节；无效或越出 margin 限位的目标会触发制动。它不调用 IK，也不限制末端速度。详细语义见 [关节位置控制](docs/joint-position.md)。

| 返回动作 | 含义 |
|---|---|
| `TRACK` | 有效的运动参考 |
| `BRAKE` | 正在按加速度限制减速；本步可能仍移动 |
| `HOLD` | 参考已静止；仍返回有效保持点 |
| `REJECT` | 无有效参考，`reference is None`；需要下游停机处理 |

`flags` 是 `SafetyFlag` 位集合，保留多个原因。无效或过期命令在状态有效时触发受控减速；反馈过期、时序错误、过大跟踪误差、碰撞故障、模型错误和不可行约束触发 `REJECT` 并锁存。排除原因后调用 `servo.reset(fresh_state, now_ns=...)`。

默认 `collision_required=False`，无外部结果时返回 `COLLISION_DISABLED`。如果应用启用碰撞监控，应设 `collision_required=True`，每步提供 `CollisionSample(velocity_scale, stamp_ns, state_stamp_ns)`；缺失/过期结果会拒绝生成参考。该比例是对目标速度的缩放，输出的减速过程还受加速度约束，不能据此声称完整未来轨迹无碰撞。

**模型范围**

URDF 加载支持指定 base→tip 的固定、转动、连续转动和移动关节链，保留固定变换与 TCP 偏置。其他分支不参与当前链的运动学。mimic、浮动关节、平面关节和闭链会在选中链中被拒绝。基础加载器不处理 mesh、动力学或碰撞几何。

`acceleration_limits` 必须显式提供，可用按关节顺序的向量、按名称的字典或统一标量。位置/速度上限来自 URDF，速度可显式覆盖。连续关节参考保持展开角度，跟踪误差采用最短角差。

可选 Pinocchio 后端：

```bash
python -m pip install '.[pinocchio]'
```

```python
from servo_py.backends.pinocchio import PinocchioModel

model = PinocchioModel.from_urdf(
    "examples/arm6.urdf", base="base", tip="tool", acceleration_limits=3.0
)
```

这个适配器使用 Python 回调，会重新获取 GIL，适合模型对照与研究。原生 `load_urdf` 路径的运动学和 Servo 计算均在 C++ 中执行。默认导入 `servo_py` 不导入或依赖 Pinocchio。

**接入已有的 Python IK**

`PositionIKAdapter(servo, solve_ik)` 可接入已有的 `solve_ik(target_pose, q_seed)`，输出经过限位、连续性和任务位姿残差校验的 `JointPositionCommand`；无解、异常或无效解返回 `StopCommand`。它保留原始目标时间戳，并使用当前 Servo 的有效限位。完整循环、时间戳和失败处理见 [Python IK 接入教程](docs/python-ik.md)。

直接传给 `Servo.step()` 的 `PoseCommand` / `TwistCommand` 默认使用 DLS 微分 IK，也可以通过 `differential_ik=` 替换求解器。外部位置 IK 通过适配器走关节位置分支，不自动获得内置笛卡尔限速或奇异性减速；已有微分 IK 仍可输出 `JointJogCommand`。

**构建和测试**

```bash
python -m pip install -e '.[test]'
python -m pytest -q
python examples/benchmark.py --steps 5000
python -m pip install build
python -m build
```

如需验证 Pinocchio 对照测试，安装 `.[pinocchio,test]`；没有 Pinocchio 时该模块自动跳过。
Panda 的 FK/Jacobian 与动力学跟踪测试需要 `.[mujoco,test]`，运行测试无需显示器或 OpenGL 上下文。

独立 C++ 编译只要求编译器、CMake 和 Eigen 3.4，不需要 Python：

```bash
cmake -S . -B build-native \
  -DSERVO_PY_BUILD_PYTHON=OFF -DSERVO_PY_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-native -j2
ctest --test-dir build-native --output-on-failure
```

如果 Eigen 不在系统搜索路径，传入 `-DCMAKE_PREFIX_PATH=/path/to/eigen/prefix`。
安装 C++ 库后，消费者可以通过 `find_package(servo_py CONFIG REQUIRED)` 和 `servo_py::core` 链接。

数值和时序契约见 [docs/design.md](docs/design.md)，验证结果见 [docs/validation.md](docs/validation.md)。可选 Ruckig、QP/零空间控制、周期调度、设备适配协议及记录对照工具见 [高级控制接口](docs/advanced-control.md)。几何碰撞检查、位控重力补偿和跨平台构建仍未实现。该实现改变了 MoveIt Servo 的部分行为，未做逐步数值等价验证。

已完成与待实现功能、建议顺序和验收条件见 [功能清单](docs/roadmap.md)。
