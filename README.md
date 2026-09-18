# servo-py

**把关节目标或末端目标，变成可连续执行的运动参考。**

独立于 ROS/MoveIt 的在线伺服内核：C++17 / Eigen 负责计算，Python 负责接入模型、目标和设备。适合机械臂控制原型、IK 算法实验与仿真集成。基础 Python 运行依赖只有 NumPy。

[快速上手](docs/getting-started.md) · [文档目录](docs/index.md) · [Panda 演示](docs/mujoco-panda.md) · [API 参考](docs/api.md) · [English](README.en.md)

![Panda 在 MuJoCo 中跟踪空间八字轨迹：橙色为目标，青色为实际 TCP 轨迹](docs/media/panda-servo.gif)

*默认力矩模式的实际 MuJoCo 仿真。另支持直接关节位控、位置 IK 后位控。 [视频](docs/media/panda-servo.mp4) · [录制指标](docs/media/panda-servo.json)*

## 从这里开始

以下命令适用于已验证的 Linux 环境；源码构建需要 Python 3.10+ 和 C++17 编译器。

```bash
git clone https://github.com/OpenGHz/servopy.git
cd servopy
python -m venv .venv
source .venv/bin/activate
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install .
python examples/track_pose.py
```

最后一条命令不需要显示器，运行二维机械臂的理想反馈回放。预期输出包含 `"steps": 1200`、`"final_action": "HOLD"`，最终位置误差约 `9.8e-5 m`。安装选项和构建排障见 [快速上手](docs/getting-started.md)。

### 一次伺服计算

在仓库根目录运行以下完整代码。输入实际关节状态和目标，输出下一个周期末的参考；`Servo.step()` 本身不执行设备 I/O。

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
    raise RuntimeError(result.message)  # 设备适配层还需取消旧缓冲并停止。
print(result.action.name, result.reference.q)
```

继续学习：[完整关节控制循环](docs/joint-position.md) · [反馈与参考的区别](docs/concepts.md) · [设备接入](docs/runtime.md)

### 看 Panda 动起来

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install '.[mujoco]'
python examples/mujoco_panda.py
```

默认打开 viewer，运行 18 秒；空格暂停，关闭窗口退出。无桌面时添加 `--headless`。模型和 mesh 随源码提供，无需运行时下载。

| 希望验证的控制方式 | 运行选项 | 目标到执行器的路径 |
|---|---|---|
| 末端伺服，力矩执行器 | 默认 `--control-mode torque` | 位姿 → 微分 IK → 参考 → 力矩控制 |
| 直接关节位控 | `--control-mode joint-position` | 关节目标 → 参考 → 位置执行器 |
| 先位置 IK 再位控 | `--control-mode ik-position` | 位姿 → 位置 IK → 关节目标 → 位置执行器 |

例如：`python examples/mujoco_panda.py --control-mode ik-position`。[完整运行指南](docs/mujoco-panda.md) 包含平滑、外部目标、录制和指标解释。

## 按任务选择功能

| 任务 | 接口 | 教程 |
|---|---|---|
| 给定关节位置或速度 | `JointPositionCommand` / `JointJogCommand` | [关节控制](docs/joint-position.md) |
| 跟踪末端位姿或速度 | `PoseCommand` / `TwistCommand` | [模型与数据约定](docs/concepts.md) |
| 接入已有位置 IK | `PositionIKAdapter` | [Python IK](docs/python-ik.md) |
| 限制 jerk、连续加速度 | `RuckigSmoothing` | [轨迹平滑](docs/smoothing.md) |
| 替换微分 IK、优化冗余关节 | `DifferentialIK` / `BoxQPSolver` | [QP 与零空间](docs/solvers.md) |
| 安排控制周期、接入设备 SDK | `ServoRunner` / `CallbackDevice` | [周期与设备](docs/runtime.md) |
| 发送实时目标、复现一次运行 | JSONL / `JsonlRecorder` / `replay` | [输入、记录与对照](docs/recording.md) |

内核还提供关节位置/速度/加速度约束、奇异性减速与离开策略、命令/反馈超时、跟踪误差检查和故障锁存。模型可使用原生 URDF 串联链、可选 Pinocchio 或自定义 `Kinematics`。

## 当前状态

当前代码版本为 **0.3.0**。已验证环境为 Linux x86_64 / Python 3.12；功能验证包含 **178 项 Python 测试**、独立 C++ 测试和 Panda 三种控制模式的动力学仿真。详细条件、历史记录和复现说明见 [验证记录](docs/validation.md)。

约束作用于生成的参考。真实设备的插值、缓冲取消和停止由设备适配层负责；Python 调度不提供硬实时保证。目前没有几何碰撞检查器或经过验证的真机驱动，位控示例保留原始 PD 的重力稳态偏差。跨平台构建和实际 MoveIt 数值等价尚未验收，见 [功能状态](docs/roadmap.md)。

## 开发与文档

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install -e '.[test,docs]'
python -m pytest -q
python scripts/check_docs.py
python -m mkdocs serve
```

文档支持目录导航、全文搜索、深浅主题与代码复制；也可直接在 GitHub 阅读 Markdown。[贡献指南](CONTRIBUTING.md) 说明 C++ 构建、可选测试和文档验证；[更新记录](CHANGELOG.md) 说明版本变化与迁移。

## 许可证与来源

项目使用 [MIT 许可证](LICENSE)。Panda 资产来自 MuJoCo Menagerie，按 Apache-2.0 分发；构建依赖和上游设计来源见 [NOTICE](NOTICE)。servo-py 是独立实现，与 MoveIt 项目没有隶属关系，不是其数值或行为兼容替代品。
