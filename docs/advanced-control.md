# 高级控制导览

本文档已按任务拆分，以下入口对应 0.3.0 的实际接口。先从[实时伺服控制](realtime-servo.md)了解持续更新目标、实际反馈和周期输出，再为控制循环选择平滑、求解器和设备适配。

## Ruckig 与参考采样

从 [轨迹平滑](smoothing.md) 学习 jerk 限制、可行停止轨迹和 `sample_reference()`。所有控制模式均可选用，需安装 `.[ruckig]`。

## 微分 IK、QP 与零空间

从 [求解器教程](solvers.md) 学习 `DifferentialIK`、内置盒约束 QP、姿态与关节居中。位置 IK 的独立接法见 [Python IK](python-ik.md)。

## 周期调度与设备接口

从 [设备接入](runtime.md) 学习最新目标邮箱、绝对周期、取消、故障停止与显式恢复。

## Panda 外部目标与回放

从 [外部目标与记录](recording.md) 学习 stdin、带仿真时间的目标文件及 JSONL 回放。

## 数值对照

[日志及 JointTrajectory 对照](recording.md#数值对照) 提供 q/dq/ddq 差异、时间对齐和命令行报告。实际 MoveIt 数据与真机验收情况见 [功能状态](roadmap.md)。

[返回文档目录](index.md)
