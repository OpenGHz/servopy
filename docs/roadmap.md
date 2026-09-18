# 功能状态与后续工作

本清单对应 `0.3.0`。ServoPy 的主要能力是[实时关节与笛卡尔伺服控制](realtime-servo.md)：运行中持续接收目标、读取实际反馈，并逐周期更新控制输出。已提供控制算法、周期调度、实时目标输入、记录与对照工具，以及 Linux wheel；几何碰撞与位控重力补偿仍在当前范围之外。

## 已实现

| 功能 | 实现与验证 |
|---|---|
| 实时伺服循环 | 每周期目标与反馈输入、在线反向与目标更新、命令过期制动、最新目标邮箱；Python 墙钟调度为 best effort |
| 原生关节位置指令 | Python/C++ JointPosition、名称映射、连续关节、限位、超时、实际目标误差 |
| 外部位置 IK | PositionIKAdapter 验证解、限位、连续性、位姿残差及源时间戳；失败制动 |
| jerk / Ruckig | 可选 RuckigSmoothing、分段轨迹采样、连续极值检查、保留可行停止轨迹；验证变周期、反向、过期、限位和 Panda 三种模式 |
| 微分 IK 插件 | C++/Python DifferentialIK 接口；异常、错误维度和非有限结果锁存 SOLVER_ERROR |
| QP 与冗余控制 | 原生主动集盒约束 QP、零空间姿态和关节居中；对照穷举小规模最优解及七轴任务不变性 |
| 周期调度与设备适配 | ServoRunner、最新目标邮箱、Device/CallbackDevice、SimulatedDevice；取消、停止、恢复、断流及过载处理 |
| Panda 外部目标 | 实时 stdin JSONL、带仿真时间的文件回放、三种模式接入与目标超时 |
| 记录回放与数值对照 | JSONL 控制记录、确定性 replay、日志比较、ROS 2 JointTrajectory JSON 导出比较和命令行报告 |
| Linux wheel / CI | CPython 3.10–3.14、x86_64 / ARM64、manylinux_2_28；Ubuntu 安装验证与 Trusted Publishing，首次上传配置见[发布指南](publishing.md) |

已有功能还包括 JointJog、Twist、Pose、奇异性减速/离开策略、外部碰撞比例接口、URDF/串联运动学、Pinocchio、MuJoCo viewer 与录制。高级用法见 [advanced-control.md](advanced-control.md)，验证范围见 [validation.md](validation.md)。

## 本批排除项

| 功能 | 状态 |
|---|---|
| 实际几何碰撞检查 | 仍只有外部 CollisionSample；未添加自碰撞/环境距离后端 |
| Panda 位控重力补偿 | 位置执行器仍保留原始 PD 与稳态偏差 |
| 其他平台 wheel | 暂不提供 Windows、macOS、32 位、PyPy 或 free-threaded Python 的 wheel |

## 仍需外部环境验收

| 项目 | 已提供的软件与未完成的实测 |
|---|---|
| 真实机器人 | 设备协议、SDK 回调适配与故障/调度测试已实现；厂商通信协议、真实 watchdog、断电/急停和负载性能仍须在目标设备验收 |
| MoveIt 数值对照 | 导出数据比较工具及合成样本测试已实现；尚无真实 MoveIt 运行导出，未宣称逐周期数值等价 |

Python 周期调度不是硬实时。Ruckig 是可选受限参考生成器，QP 只处理盒约束；均不等同于避障规划或真实设备的停止执行。
