# 动作、标志与诊断

先看 `result.action` 判断能否执行参考，再看 `flags` 和 `diagnostics` 定位原因。flags 是位集合，不是互斥枚举；可用 `if result.flags & SafetyFlag.STALE_COMMAND:` 检查单项。

## Action

| 动作 | reference | 应用层行为 |
|---|---|---|
| `TRACK` | 有效 | 执行生成的参考 |
| `BRAKE` | 有效 | 继续执行制动参考，不能直接跳过发送 |
| `HOLD` | 有效 | 执行保持点；结合实际速度和目标误差判断完成 |
| `REJECT` | None | 取消旧设备缓冲，请求设备停止，处理原因后显式恢复 |

`Servo.reset()` 清除内核故障；`ServoRunner.recover()` 还执行设备恢复和停止反馈检查。反复 reset 不是故障处置方法。

## SafetyFlag

### 命令问题

| 标志 | 原因与行为 |
|---|---|
| `INVALID_COMMAND` | 命令形状、值、帧或目标范围无效；在有效状态下制动 |
| `STALE_COMMAND` | 运动目标过期，制动 |
| `FUTURE_TIMESTAMP` | 命令来自未来时制动；反馈来自未来时拒绝 |

### 状态与执行错误

| 标志 | 原因与行为 |
|---|---|
| `INVALID_STATE` | 反馈维度/数值、位置或速度无效，拒绝 |
| `STALE_STATE` | 反馈过期，拒绝 |
| `INVALID_TIMING` | dt、调用间隔、单调时钟或端点时间无效，拒绝 |
| `TRACKING_ERROR` | 实际关节未能跟踪参考，拒绝 |
| `INFEASIBLE` | 不能生成满足条件的参考，拒绝 |
| `FAULT_LATCHED` | 此前故障尚未显式恢复，拒绝 |
| `MODEL_ERROR` | FK、Jacobian、坐标运算等模型调用失败，拒绝 |
| `SOLVER_ERROR` | 微分 IK 抛异常、返回错误维度/非有限值或 QP 未收敛，拒绝 |
| `SMOOTHING_ERROR` | 平滑器无法给出有效输出或初始状态无可行停止，拒绝 |

### 约束与运行信息

| 标志 | 解释 |
|---|---|
| `NONE` | 没有置位标志 |
| `VELOCITY_LIMIT` | 目标速度触发缩放 |
| `ACCELERATION_LIMIT` | 默认参考生成触发加速度限制 |
| `POSITION_LIMIT` | 位置/停止包络限制；也可能伴随无效状态或不可行拒绝 |
| `SINGULARITY_DECELERATION` | 进入奇异性减速区间 |
| `SINGULARITY_HALT` | 奇异性策略请求制动 |
| `LEAVING_SINGULARITY` | 局部探测判定请求运动有助于离开奇异区 |
| `JERK_LIMIT` | 启用了 jerk 受限生成，并不表示本步一定触及最大 jerk |
| `SMOOTHING_FALLBACK` | 新目标不可接受，继续此前验证的停止轨迹 |
| `GOAL_REACHED` | 相应目标进入容差；开启零空间时可与 TRACK 同时存在 |
| `MODE_SWITCH` | 命令类型相对前一步变化 |

### 外部碰撞结果

| 标志 | 解释 |
|---|---|
| `COLLISION_DISABLED` | 当前无样本且未强制要求检查，不表示几何安全 |
| `COLLISION_MISSING` | 要求检查但无结果，拒绝 |
| `COLLISION_STALE` | 结果/源反馈过期或样本无效，拒绝 |
| `COLLISION_DECELERATION` | 有效比例在 0 与 1 之间，缩放目标速度 |
| `COLLISION_HALT` | 比例为 0，拒绝并要求设备侧停止 |

## Diagnostics

| 字段 | 解释 |
|---|---|
| `sigma_min` | 选轴加权 Jacobian 的最小奇异值 |
| `damping` | 本周期微分 IK 阻尼 |
| `singularity_scale` | 奇异性策略缩放 |
| `velocity_scale` | 目标关节速度的统一缩放 |
| `collision_scale` | 外部样本速度比例 |
| `tracking_error` | 参考关节与测量关节的最大绝对差 |
| `joint_position_error` | JointPosition 目标与测量关节的最大绝对差 |
| `position_error` | Pose 活跃平移轴误差的范数 |
| `orientation_error` | Pose 活跃旋转轴 SO(3) 对数误差的范数 |
| `task_residual` | 求解器输出的加权任务残差，早于后续缩放和平滑 |
| `nullspace_speed` | 投影后的次级关节速度范数 |

字段按所走分支计算；未计算的误差/奇异值通常为 0，缩放默认为 1。不能把未走 Pose 分支时的 `position_error == 0` 解读为实际 TCP 无误差。

下一步：[按症状排障](troubleshooting.md) · [参数默认值](configuration.md) · [完整执行契约](design.md)
