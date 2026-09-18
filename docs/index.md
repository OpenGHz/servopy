# ServoPy 文档

从目标和关节反馈开始，生成可连续执行的参考。本文档对应 **0.3.0**，涵盖 C++ 内核、Python 接口与 Panda / MuJoCo 示例。

## 第一次使用

1. [快速上手](getting-started.md)：安装，完成一次无需显示器的运行。
2. [关键概念](concepts.md)：区分目标、实际反馈和参考，选择控制入口。
3. [Panda 仿真](mujoco-panda.md)：在力矩、直接位控、IK 位控之间切换。

![默认力矩模式的 Panda 仿真](media/panda-servo.gif)

## 按你的任务继续

| 我想…… | 从这里开始 | 完成后可以…… |
|---|---|---|
| 给机械臂发送关节目标 | [关节位置与速度](joint-position.md) | 运行完整控制与停止循环 |
| 接入已有 Python IK | [位置 IK 教程](python-ik.md) | 验证解并转换成关节目标 |
| 限制加速度变化 | [Ruckig 平滑](smoothing.md) | 正确采样 jerk 受限参考 |
| 换求解器、控制冗余姿态 | [微分 IK / QP / 零空间](solvers.md) | 配置或实现数值后端 |
| 接入设备 SDK | [周期与设备](runtime.md) | 明确发送、停止、恢复边界 |
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
