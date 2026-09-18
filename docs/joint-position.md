# 原生关节位置控制

`servo-py 0.2.0` 增加 Python `JointPositionCommand` 与 C++ `CommandType::JOINT_POSITION`。控制器可直接输入关节目标，内核计算连续、受关节位置/速度/加速度约束的参考。它不调用 FK、Jacobian 或 IK，也不限制 TCP 速度。

## Python 用法

```python
from servo_py import JointPositionCommand, Servo, ServoConfig

servo = Servo(model, ServoConfig(
    joint_position_gain=2.0,       # 1/s
    joint_position_tolerance=1e-4, # 转动关节 rad，移动关节 m
))
command = JointPositionCommand(positions=q_target, stamp_ns=target_stamp_ns)
result = servo.step(state, command, dt=0.01, now_ns=now_ns)
```

实际反馈 `state`、目标时间戳、当前时间以及下游执行由调用方提供。`result.reference.q/dq/ddq` 仍是下一周期端点参考。真实设备必须执行参考而非把它当成真实反馈；遇到 `REJECT` 或异常应取消旧队列并调用设备停止接口。

自由度少于 6 的模型仍须在构造 Servo 时选择合适的 `task_axes`，例如两关节模型使用 `ServoConfig(task_axes=(0, 1))`；这是当前通用配置的校验要求，并不会在关节位置分支启用笛卡尔求解。

无名称时，目标顺序必须与 `model.joint_names()` 一致。带名称时允许任意排列，但必须覆盖所有受控关节。例如两关节模型名称为 `shoulder, elbow`：

```python
command = JointPositionCommand(
    positions=[-0.7, 0.4], names=["elbow", "shoulder"], stamp_ns=now_ns,
)
```

只指定部分关节、重名、未知名称或错误维度均视为无效指令，触发受控制动。省略的关节不会被补为零目标；这与支持子集速度命令的 `JointJogCommand` 不同。

## 数值与状态语义

内核先计算 `error = model.difference(q_target, q_reference)`，再使用 `joint_position_gain * error` 作为待约束速度。`q_reference` 是上一步接受的参考，真实反馈用于独立的状态和跟踪误差保护。连续关节采用最短角差，参考位置保持展开；转动关节用 rad，移动关节用 m。

目标必须在 `lower + margin` 到 `upper - margin` 内，且全部有限；不会悄悄裁剪越界目标。无效、过期和未来时间戳的指令按现有规则请求制动，正常情况下不锁存故障。有效关节限位可通过 `servo.limits` 读取，返回值为独立副本。

`joint_position_gain` 与 `joint_position_tolerance` 必须为有限正数，默认分别为 `2.0` 和 `1e-4`。参考误差进入容差时请求制动，随后保持。位置反馈律不保证单调接近目标，高增益、初始速度或突然反向时可能超调；现有约束保证的是参考满足关节限位，不是任何目标变化都能立即到达。

| 返回信息 | 含义 |
|---|---|
| `diagnostics.joint_position_error` | 目标与本周期真实反馈的最大绝对关节差 |
| `diagnostics.tracking_error` | 上一步参考与本周期真实反馈的最大绝对关节差 |
| `GOAL_REACHED` | 参考误差和真实位置误差均已进入容差；仍须检查动作与实际速度 |
| `HOLD` | 参考已静止；不单独证明真实设备已准确到位 |

外部碰撞速度缩放、故障锁存、模式切换及复位逻辑保持适用。全套 [执行契约](design.md) 仍有效；`0.3.0` 可通过 `motion_generator=RuckigSmoothing(...)` 启用 jerk 限制；使用 Ruckig 时直接规划完整关节位置目标，以 `sample_reference(t)` 采样。没有端到端碰撞安全保证。

## C++ 用法

```cpp
servo_py::Command command;
command.type = servo_py::CommandType::JOINT_POSITION;
command.joint_position = target;
command.stamp_ns = now_ns;
auto result = servo.step(state, command, 0.01, now_ns);
```

已有位置 IK 可通过 [PositionIKAdapter](python-ik.md#推荐接法positionikadapter) 生成这个指令。Panda 的 `--control-mode joint-position` 与 `--control-mode ik-position` 都已接入此路径。
