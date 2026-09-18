# 关节位置与速度控制

**目标：** 在实时伺服循环中持续跟踪关节位置或速度，处理停止、命名映射与目标到达。前置：已完成[安装](getting-started.md)，了解[实时目标更新](realtime-servo.md)。本页完整示例只需要基础依赖。

## 1. 运行完整关节位置循环

以下代码在仓库根目录直接运行，使用二维模型与理想反馈。前 6 秒由本地任务持续发布同一关节目标，后 2 秒发 Stop；仿真时间不会等待墙钟。

<!-- runnable: joint-position-loop -->
```python
import numpy as np
from servo_py import (
    Action, JointPositionCommand, JointState, Servo, ServoConfig,
    StopCommand, load_urdf,
)

model = load_urdf(
    "examples/planar2.urdf", base="base", tip="tool",
    acceleration_limits=[3.0, 3.0],
)
servo = Servo(model, ServoConfig(task_axes=(0, 1)))
q, dq = np.array([0.5, -1.0]), np.zeros(2)
target = np.array([0.7, -0.7])
for tick in range(800):
    now = tick * 10_000_000
    command = JointPositionCommand(target, now) if tick < 600 else StopCommand()
    result = servo.step(JointState(q, dq, now), command, dt=0.01, now_ns=now)
    if result.action == Action.REJECT:
        raise RuntimeError(result.message)
    q, dq = result.reference.q, result.reference.dq  # 仅用于理想反馈回放。

assert result.action == Action.HOLD
assert np.max(np.abs(q - target)) < 1.1e-4
assert np.max(np.abs(dq)) < 1e-9
print(result.action.name, "joint error:", np.max(np.abs(q - target)))
```

预期：最终 `HOLD`，最大关节目标误差小于 `1.1e-4 rad`。接真实设备时，用测量值构造 `JointState`，并把参考交给控制器；不要用参考覆盖真实反馈。

## 2. 理解关节位置分支

默认反馈律基于上一参考的位置：`desired_dq = joint_position_gain * model.difference(target, reference.q)`。随后应用速度、加速度和采样制动位置约束。连续关节使用最短角差，参考角度保持展开。

完整目标必须在 `lower + margin` 与 `upper - margin` 内。错误维度、NaN 或越限目标会请求制动。该分支不调用 FK/Jacobian/IK，也不直接约束 TCP 速度；任务空间行为需要由上层目标规划控制。

## 3. 按名称传目标

以下是命令构造片段。位置目标必须覆盖全部关节，但可以重排；关节速度命令允许子集，未列关节的目标速度为零。

```python
from servo_py import JointJogCommand, JointPositionCommand

position = JointPositionCommand(
    positions=[-0.7, 0.7], names=["elbow", "shoulder"], stamp_ns=0,
)
jog = JointJogCommand(velocities=[0.05], names=["elbow"], stamp_ns=0)
```

重复、未知或遗漏的位置关节名称均无效。无名称时按 `model.joint_names()` 顺序；转动关节单位是 rad，不是角度。

## 4. 区分 HOLD 与到达目标

| 信号 | 含义 |
|---|---|
| `diagnostics.joint_position_error` | 目标与实际关节的最大绝对差 |
| `diagnostics.tracking_error` | 生成参考与实际关节的最大绝对差 |
| `GOAL_REACHED` | 参考与实际目标误差都进入 `joint_position_tolerance` |
| `HOLD` | 参考静止，可能来自 Stop、命令过期或到达目标 |

不能仅用 `HOLD` 判断运动成功。目标是否达到看误差与 flags；真实设备是否停止还要看反馈速度。

## 5. 启用 jerk 约束或执行到 Panda

`motion_generator=RuckigSmoothing(...)` 会对有效关节目标直接规划位置轨迹；此时比例增益不决定该轨迹的运动速度。执行端应使用 `sample_reference(t)`，细节见 [平滑教程](smoothing.md)。

```bash
python examples/mujoco_panda.py --control-mode joint-position
python examples/mujoco_panda.py --control-mode ik-position
```

前者直接给关节目标，后者先做位置 IK。完整区别见 [Panda 三种模式](mujoco-panda.md#三种控制模式)。下一步：[已有 IK 接入](python-ik.md) · [动作与诊断](status.md) · [设备循环](runtime.md)
