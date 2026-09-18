# ServoPy：实时机器人伺服控制

**Realtime robot control with C++ and Python.** ServoPy 面向持续运行的机器人控制循环：每个周期读取最新目标与实际关节反馈，更新下一控制区间的运动输出。本文档对应 **0.3.0**，涵盖独立于 ROS 的 C++ 内核、Python API 与 Panda / MuJoCo 示例。

可以在运动中更新关节位置、关节速度、末端位姿或 Twist，让机器人持续响应遥操作、视觉反馈或上层任务。输入源和设备连接由应用提供；ServoPy 负责控制循环中的伺服计算、约束处理与参考更新。

从[实时伺服控制](realtime-servo.md)了解最新目标、反馈闭环、控制周期与断流制动。文档中的“实时”指在线周期响应；当前 Python 调度采用 best effort，不提供硬实时 deadline 保证。

## 第一次使用

1. [快速上手](getting-started.md)：安装，完成一次无需显示器的运行。
2. [实时伺服控制](realtime-servo.md)：在持续运行的循环中改变目标，观察反向与断流制动。
3. [关键概念](concepts.md)：区分目标、实际反馈和参考，选择控制入口。
4. [Panda 仿真](mujoco-panda.md)：在力矩、直接位控、IK 位控之间切换，接收实时外部目标。

![默认力矩模式的 Panda 仿真](media/panda-servo.gif)

## 按你的任务继续

| 我想…… | 从这里开始 | 完成后可以…… |
|---|---|---|
| 持续跟踪遥操作或感知给出的目标 | [实时伺服控制](realtime-servo.md) | 将上游目标流接入周期控制循环 |
| 给机械臂发送关节目标 | [关节位置与速度](joint-position.md) | 运行完整控制与停止循环 |
| 接入已有 Python IK | [位置 IK 教程](python-ik.md) | 验证解并转换成关节目标 |
| 限制加速度变化 | [Ruckig 平滑](smoothing.md) | 正确采样 jerk 受限参考 |
| 换求解器、控制冗余姿态 | [微分 IK / QP / 零空间](solvers.md) | 配置或实现数值后端 |
| 接入设备 SDK | [实时循环与设备](runtime.md) | 周期读取反馈、发送输出并处理停止与恢复 |
| 发送实时目标或分析一段运行 | [输入、记录与对照](recording.md) | 生成日志、回放、量化差异 |
| 只使用 C++ | [原生 C++](cpp.md) | 构建并链接独立内核 |

## 查接口与解决问题

| 资料 | 用途 |
|---|---|
| [Python API](api.md) | 类型、方法签名、输入输出和异常 |
| [配置参数](configuration.md) | 全部默认值、单位和调参顺序 |
| [动作与诊断](status.md) | `Action`、`SafetyFlag`、误差字段 |
| [命令行参考](cli.md) | 示例脚本的全部参数 |
| [排障](troubleshooting.md) | 按安装、时序、IK、仿真症状定位 |

## 理解设计与参与开发

[执行契约（英文）](design.md) 说明算法和时序；[验证记录（英文）](validation.md) 保留测试环境与实际观测。功能完成度见 [路线图](roadmap.md)，版本迁移见 [更新记录](changelog.md)。参与修改请阅读 [贡献指南](contributing.md)。

所有教程命令默认从仓库根目录执行。标为“完整示例”的 Python 代码可以直接运行；标为“接入片段”的代码需要应用提供模型、状态或 SDK 回调。
