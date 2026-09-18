<p align="center">
  <a href="README.md">English</a> &nbsp;·&nbsp; <strong>简体中文</strong>
</p>

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/assets/brand/logo-dark.svg">
    <img src="docs/assets/brand/logo-light.svg" alt="ServoPy" width="420">
  </picture>
</p>

<p align="center">
  <strong>基于 C++ 与 Python 的实时机器人控制。</strong><br>
  C++ 内核 &nbsp;·&nbsp; Python 接口 &nbsp;·&nbsp; 无需 ROS
</p>

<p align="center">
  <a href="#快速上手">快速上手</a> &nbsp;·&nbsp;
  <a href="#panda-演示">Panda 演示</a> &nbsp;·&nbsp;
  <a href="https://openghz.github.io/servopy/">在线文档</a>
</p>

---

**ServoPy** 是一个**实时关节与笛卡尔伺服控制库**，提供 C++ 内核和 Python API，无需 ROS。每个控制周期根据最新目标和实际关节反馈，更新交给下游控制器执行的运动指令。

机器人运动过程中，可以持续发送关节位置、关节速度、末端位姿或 Twist。目标可以来自遥操作界面、视觉反馈循环或其他应用；ServoPy 随目标变化持续更新控制输出。目标来源和设备连接由应用接入。

<p align="center">
  <a href="docs/media/panda-servo.mp4">
    <img src="docs/media/panda-servo.gif" alt="Panda 在 MuJoCo 中跟踪八字轨迹：橙色为目标，青色为实际 TCP 路径" width="640">
  </a>
</p>

<p align="center">
  Panda / MuJoCo 实际录像 · 力矩模式 · 100 Hz 伺服 / 500 Hz 物理仿真<br>
  <a href="docs/media/panda-servo.mp4">观看视频</a> &nbsp;·&nbsp;
  <a href="docs/media/panda-servo.json">查看测量结果</a>
</p>

## 实时伺服控制

- **运动中更新目标。** 持续输入关节或末端命令；最新目标邮箱保留最新命令，供下一控制周期读取。
- **闭环响应反馈。** 每周期把实际反馈交给 `Servo.step()`，也可用 `ServoRunner` 周期调度反馈读取、控制输出和设备停止回调。
- **约束每次控制更新。** 限制关节位置、速度和加速度，处理奇异性，按需启用 Ruckig 的 jerk 约束。指令过期请求制动，反馈和时序故障明确上报。
- **接入自己的运动学与 IK。** 选择原生 URDF 或 Pinocchio、DLS 或盒约束 QP、零空间姿态目标，也可接入已有 Python 位置 IK。
- **观察正在运行的控制器。** 体验 Panda 三种控制模式、流式发送实时目标，并记录和回放控制过程。

每次 `step()` 输出下一控制区间的参考；下一周期可以采用新目标，无需等上一个目标执行完成。[实时伺服教程](https://openghz.github.io/servopy/realtime-servo/)提供运行中切换目标、低频目标源与高频伺服循环配合、断流后制动的完整示例。

这里的 **realtime（实时）** 指周期控制循环持续响应目标与反馈。`ServoRunner` 使用尽力而为的 Python 调度，ServoPy 不保证硬实时 deadline；时序要求与设备职责见[执行契约](https://openghz.github.io/servopy/design/)。

必需的第三方 Python 运行依赖只有 NumPy；从源码构建还需要 C++ 工具链。MuJoCo、Pinocchio 和 Ruckig 均为可选依赖。

## 快速上手

需要 **Python 3.10 或更新版本**。[ServoPy 0.3.0 已发布到 PyPI](https://pypi.org/project/servo-py/0.3.0/)，提供 CPython 3.10–3.14、x86_64 / ARM64（glibc 2.28+）的 Linux wheel，包含示例代码和小型 URDF。Panda 模型仅在运行该示例时下载。

可在虚拟环境直接安装，无需 C++ 编译器：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install --only-binary=:all: servo-py
python -m servo_py.examples.track_pose
```

现在也可从源码安装，此时需要**支持 C++17 或更新标准的编译器**。C++17 表示最低语言标准；构建依赖和已测试环境见[安装要求](https://openghz.github.io/servopy/getting-started/#1-准备环境)。

```bash
git clone https://github.com/OpenGHz/servopy.git
cd servopy
python -m venv .venv
source .venv/bin/activate
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install .
python examples/track_pose.py
```

无需显示器或机器人。这个理想反馈示例运行 **1,200 步**，最终进入 **HOLD**，位置误差约 **9.8e-5 m**。它验证参考生成，不包含动力学仿真。

### 第一次伺服计算

安装后可从任意目录运行以下完整示例。它展示一个控制周期：`Servo.step()` 接收最新目标和实际反馈，返回交给仿真器或设备适配层执行的下一参考。运行中的控制器用新反馈和最新命令重复这一周期。

<!-- runnable: readme-step-zh -->
```python
from importlib.resources import files
from servo_py import (
    Action, JointPositionCommand, JointState, Servo, ServoConfig, load_urdf,
)

model = load_urdf(
    files("servo_py.examples").joinpath("planar2.urdf"), base="base", tip="tool",
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

预期输出：`TRACK [ 0.50015 -0.99985]`。接下来运行[持续更新目标的完整循环](https://openghz.github.io/servopy/realtime-servo/)，并通过[设备接入](https://openghz.github.io/servopy/runtime/)了解执行时序与设备侧停止。

## Panda 演示

完成快速上手后，安装 MuJoCo 并选择控制模式：

```bash
python -m pip install --only-binary=:all: 'servo-py[mujoco]'
servo-py-panda --control-mode joint-position
```

| `--control-mode` | 目标 → 参考 → 执行器 |
|---|---|
| `torque`（默认） | 位姿 → 微分 IK → 关节参考 → 力矩控制 |
| `joint-position` | 关节目标 → 关节参考 → 位置执行器 |
| `ik-position` | 位姿 → 位置 IK → 关节参考 → 位置执行器 |

默认运行 18 秒，伺服循环为 100 Hz、物理仿真为 500 Hz。每个伺服周期读取 MuJoCo 反馈并更新控制参考。按**空格**暂停，无桌面时加 `--headless`。使用 `--target-stdin` 可接入实时 JSONL 目标流，见[流式目标输入](https://openghz.github.io/servopy/recording/#panda-外部目标与回放)。

wheel 和源码发行包不包含 Panda 模型；安装后的示例首次运行时下载约 5 MB，经校验后缓存，之后可离线复用。Git 克隆可直接使用仓库已有压缩包。[Panda 教程](https://openghz.github.io/servopy/mujoco-panda/)进一步介绍离线模型路径、Ruckig 平滑、外部目标、录制和实际跟踪表现。

## 文档

[打开在线文档 →](https://openghz.github.io/servopy/)：支持全文搜索的入门指南、控制教程和 API 参考。

教程和 API 参考以中文维护；[设计契约](https://openghz.github.io/servopy/design/)与[验证记录](https://openghz.github.io/servopy/validation/)为英文。

| 下一步 | 阅读入口 |
|---|---|
| 编写实时控制循环 | [实时伺服](https://openghz.github.io/servopy/realtime-servo/) · [设备和调度](https://openghz.github.io/servopy/runtime/) |
| 编写控制器 | [关节控制](https://openghz.github.io/servopy/joint-position/) · [位置 IK](https://openghz.github.io/servopy/python-ik/) |
| 调整或扩展 | [轨迹平滑](https://openghz.github.io/servopy/smoothing/) · [QP 与零空间](https://openghz.github.io/servopy/solvers/) · [C++](https://openghz.github.io/servopy/cpp/) |
| 接入与分析 | [设备和调度](https://openghz.github.io/servopy/runtime/) · [记录与回放](https://openghz.github.io/servopy/recording/) |
| 查询接口 | [API](https://openghz.github.io/servopy/api/) · [配置](https://openghz.github.io/servopy/configuration/) · [排障](https://openghz.github.io/servopy/troubleshooting/) |

[浏览全部文档 →](https://openghz.github.io/servopy/)

## 项目状态

发行版本 **0.3.0** 已通过 Ubuntu 22.04 / 24.04 的安装验证：**x86_64 通过 188 项 Python 测试**，**ARM64 通过 176 项**，跳过可选 Ruckig 检查。验证还包括独立 C++ 测试和 Panda 三种控制模式的动力学仿真。条件和结果保存在[验证记录](https://openghz.github.io/servopy/validation/)中。

实时伺服循环输出供下游控制器执行的运动参考；反馈获取、执行器命令和设备停止由应用负责。目前尚未完成真机验证、几何碰撞检查或硬实时执行验证，Panda 位控演示保留重力导致的跟踪偏差。完整能力边界见[路线图](https://openghz.github.io/servopy/roadmap/)。

## 参与贡献

[贡献指南](CONTRIBUTING.md)介绍开发环境、相关测试和文档检查；[更新记录](CHANGELOG.md)说明 API 迁移。可以在本地预览支持搜索的文档：

```bash
python -m pip install '.[docs]'
python scripts/check_docs.py
python -m mkdocs serve
```

## 许可证

ServoPy 使用 [MIT 许可证](LICENSE)。Panda 资产按 Apache-2.0 分发，来源与依赖声明见 [NOTICE](NOTICE)。ServoPy 是独立项目，不声明与 MoveIt 兼容。
