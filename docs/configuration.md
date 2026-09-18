# 配置参数

所有表格对应 **0.3.0** 的 `ServoConfig` 默认值。构造控制器时生效；运行时修改 Python 对象不会改变已经复制到 C++ 的配置。创建前完成模型和任务轴选择，再设置限位、时序、增益与容差。

## 时间与反馈

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `command_timeout` | `0.1` | s | 运动命令有效期，超时请求制动 |
| `state_timeout` | `0.1` | s | 反馈有效期，超时 REJECT |
| `collision_timeout` | `0.1` | s | 碰撞结果及其源反馈有效期 |
| `max_dt` | `0.05` | s | 单次参考区间上限；配置最大允许 1 s |
| `timing_tolerance` | `0.5` | 比例 | 调用间隔相对前一次 dt 的允许误差 |
| `max_tracking_error` | `0.2` | rad 或 m | 参考与实际关节最大绝对差上限 |

命令过期与反馈过期处理不同。不能靠扩大超时或跟踪阈值来代替修复时钟、通信或执行器问题；先从 [诊断字段](status.md) 确认原因。

## 目标反馈与速度

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `position_gain` | `2.0` | 1/s | Pose 的位置误差到线速度增益 |
| `orientation_gain` | `2.0` | 1/s | Pose 的姿态误差到角速度增益 |
| `joint_position_gain` | `2.0` | 1/s | 默认 JointPosition 反馈律增益；Ruckig 位置规划不使用它决定速度 |
| `joint_position_tolerance` | `1e-4` | rad 或 m | 每关节位置目标容差 |
| `max_linear_speed` | `0.2` | m/s | 内核 Pose/Twist 活跃线速度分量的范数上限 |
| `max_angular_speed` | `0.5` | rad/s | 内核 Pose/Twist 活跃角速度分量的范数上限 |
| `position_tolerance` | `1e-4` | m | Pose 活跃位置轴的误差范数容差 |
| `orientation_tolerance` | `1e-3` | rad | Pose 活跃姿态轴的误差范数容差 |

笛卡尔速度上限不适用于 JointPosition、JointJog 或位置 IK 转换后的关节命令。应由相应目标生成器限制任务空间运动。

## 微分 IK 与奇异性

| 参数 | 默认值 | 含义 |
|---|---|---|
| `min_damping` | `1e-4` | 良好条件下的阻尼 |
| `max_damping` | `0.1` | 最强阻尼 |
| `damping_threshold` | `0.1` | 自适应阻尼使用的最小奇异值尺度 |
| `singularity_soft` | `0.05` | 奇异性减速区间上界 |
| `singularity_hard` | `0.001` | 无法判定为离开奇异点时的停止阈值 |
| `singularity_probe_step` | `0.01` | 关节空间探测步长的范数 |
| `singularity_escape_epsilon` | `1e-6` | 探测最小奇异值所需改善量 |

阻尼和奇异值阈值依赖模型、任务轴和权重；混合平移/旋转任务时没有通用的机器人无关单位。必须满足 `min_damping <= max_damping` 与 `singularity_hard < singularity_soft`。

## 任务与次级姿态

| 参数 | 默认值 | 含义 |
|---|---|---|
| `task_axes` | `(0, 1, 2, 3, 4, 5)` | 活跃任务轴，唯一且非空，数量不得大于自由度 |
| `task_weights` | `(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)` | 六个轴的正有限权重；禁用轴用 task_axes，而非零权重 |
| `nullspace_gain` | `0.0` | 朝参考关节姿态的次级速度增益 |
| `joint_centering_gain` | `0.0` | 有限关节边界内的居中增益 |
| `nullspace_reference` | `()` | 可选完整关节姿态；正 nullspace_gain 时必填 |
| `collision_required` | `False` | 每步是否必须提供有效外部 CollisionSample |

次级增益需有限且非负。姿态目标需满足有效限位和 margin。除增益、时序比例和布尔项外，其余标量阈值要求正有限值；`timing_tolerance` 允许 `[0,1]`。

## 关节限位与平滑参数

`JointLimits` 与 `ServoConfig` 分开传入。`lower`、`upper`、`velocity`、`acceleration` 是 `(n,)`；`margin` 默认 `0.01`，可为标量或 `(n,)`。连续关节可使用无限位置边界。`servo.limits` 可查看控制器实际采用的副本。

| 单独配置的接口 | 参数 | 默认行为 |
|---|---|---|
| `RuckigSmoothing` | `max_jerk` | 必填，正有限标量或向量 |
| `PositionIKAdapter` | `max_joint_step` | None，不启用相对种子的连续性检查 |
| `BoxQPSolver` | `max_iterations` / `tolerance` | 200 / 1e-9 |
| `ServoRunner` | `period` / `stop_timeout` | 0.01 s / 5.0 s |

## 建议调参顺序

1. 对齐关节顺序、单位、base/TCP 和任务轴，用 FK/Jacobian 对照确认模型。
2. 根据设备能力设置位置、速度、加速度与 margin；再选择是否需要 jerk。
3. 先跑固定目标并记录误差，确认参考可被实际执行器跟踪。
4. 调整增益和容差，再引入动态目标、QP 或零空间项，每次只改变一类参数。

教程参数只是演示值；它们不是 Panda 或其他机器人硬件的通用推荐值。
