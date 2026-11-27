# 命令滤波与方向保持

**目标：** 抑制遥操作或感知目标的抖动；加速度受限时让末端仍沿命令方向运动。前置：完成[实时伺服控制](realtime-servo.md)。两项功能都不需要可选依赖，自 0.4.0 起提供。

两者处在控制链的不同位置。`ButterworthCommandFilter` 在 `Servo.step` 之前处理命令；`preserve_direction` 决定默认参考生成器如何满足加速度和位置包络。生成的参考仍由内核约束，停止行为不变。

## 与 MoveIt Servo 平滑插件的对应

MoveIt Servo 2.15 在 `getNextJointState` 的最后一步调用平滑插件，默认使用 Butterworth。下表列出 ServoPy 中的对应位置：

| MoveIt 插件 | 作用位置 | ServoPy |
|---|---|---|
| `ButterworthFilterPlugin`（默认） | 输出的关节位置，在限位处理之后 | `ButterworthCommandFilter`，作用于输入命令 |
| `AccelerationLimitedPlugin` | 相邻输出之间的加速度，沿命令方向缩放 | `ServoConfig(preserve_direction=True)` |
| `RuckigFilterPlugin` | jerk 限制 | [RuckigSmoothing](smoothing.md) |

ServoPy 不对输出参考做事后滤波。上游 Butterworth 只修改位置、不更新速度，滞后还会推迟停止；本项目要求 q/dq/ddq 自洽、区间可采样，并且每段参考都保留可行的停止轨迹，两者相冲突。滤波放在命令侧，参考生成的保证就不受影响。

## 命令低通滤波

以下完整示例给关节速度命令加入噪声，比较滤波前后参考加速度的波动，然后发 Stop 制动到静止：

<!-- runnable: command-filter-jog -->
```python
import numpy as np
from servo_py import (
    Action, ButterworthCommandFilter, JointJogCommand, JointState,
    Servo, ServoConfig, StopCommand, load_urdf,
)

model = load_urdf("examples/planar2.urdf", base="base", tip="tool",
                  acceleration_limits=[3.0, 3.0])
noise = np.random.default_rng(0).normal(0.0, 0.05, (200, 2))

def run(cutoff_hz):
    servo = Servo(model, ServoConfig(task_axes=(0, 1)))
    command_filter = ButterworthCommandFilter(servo, cutoff_hz=cutoff_hz) if cutoff_hz else None
    q, dq, accelerations = np.array([0.5, -1.0]), np.zeros(2), []
    for tick in range(200):
        now = tick * 10_000_000
        command = (JointJogCommand([0.2, -0.1] + noise[tick], now)
                   if tick < 150 else StopCommand())
        if command_filter is not None:
            command = command_filter.filter(command, dt=0.01, now_ns=now)
        result = servo.step(JointState(q, dq, now), command, dt=0.01, now_ns=now)
        if result.action == Action.REJECT:
            raise RuntimeError(result.message)
        ref = result.reference
        q, dq = ref.q, ref.dq
        accelerations.append(ref.ddq)
    assert result.action == Action.HOLD
    return np.std(accelerations[50:150])

raw, filtered = run(None), run(2.0)
print(f"ddq std: raw {raw:.2f}, filtered {filtered:.2f} rad/s^2")
assert filtered < 0.3 * raw
```

每个控制周期调用一次 `filter(command, dt=dt, now_ns=now_ns)`，然后用同样的 `dt` 和 `now_ns` 调用 `Servo.step`。源头更新慢于控制周期时，重复传入最新命令即可，相当于采样保持。

滤波器为一阶 Butterworth（双线性变换并做频率预畸变）。每次调用按实际周期计算系数 `C = cot(π · cutoff_hz · dt)`，周期变化时截止频率保持不变。输出是当前输入、上一输入和上一输出的非负加权平均，不会超出输入范围。合法的 JointPosition 目标因此仍在有效限位内，Twist 的范数也不会超过各次输入范数的最大值。

| 行为 | 说明 |
|---|---|
| 截止频率上限 | `cutoff_hz` 不得超过 `1 / (4 * max_dt)`；默认 `max_dt=0.05` 时为 5 Hz，需要更高截止频率时先减小 `max_dt` |
| 滞后 | 时间常数约为 `1 / (2π · cutoff_hz)`，2 Hz 约 80 ms；只影响运动命令 |
| 起始 | JointJog / Twist 从零速开始；JointPosition / Pose 从第一个目标开始 |
| 关节与姿态 | JointPosition 用模型的关节差值，连续关节走最短角度分支；Pose 的姿态在 SO(3) 上滤波 |
| 透传并复位 | Stop、过期、来自未来、格式错误或不支持的命令原样返回，由 Servo 制动并报告原因；制动不受滤波滞后影响 |
| 复位 | 命令类型或 Twist 表达坐标系改变时自动复位；`reset()` 可手动清除历史 |
| 时间戳 | 输出保留最新命令的源时间戳，不会延长命令有效期 |

命名命令按模型关节顺序输出。从 MoveIt 迁移时，`butterworth_filter_coeff = C` 在发布周期 `T` 下对应 `cutoff_hz = atan(1 / C) / (π · T)`。例如 C=1.5、T=0.01 s 时约为 18.7 Hz，需要 `max_dt` ≤ 0.0133 s。

使用 `ServoRunner` 时传入 `command_filter=`。runner 每周期在读取邮箱后调用它，开始运行和恢复时复位；recorder 记录滤波后、真正交给 Servo 的命令，所以回放结果一致。以下为接入片段：

```python
runner = ServoRunner(servo, device, period=0.01,
                     command_filter=ButterworthCommandFilter(servo, cutoff_hz=3.0))
```

位置 IK 场景可以先滤 PoseCommand 再求解，也可以滤 `PositionIKAdapter` 输出的 JointPositionCommand。每路命令流使用独立的滤波器实例。

## 保持方向的加速度限制

默认参考生成器会对每个关节单独裁剪到可行速度区间。关节加速度能力不同、或某个关节先触及位置包络时，关节速度比例会改变，末端运动方向也随之弯曲。`preserve_direction=True` 改为对所有关节使用同一个比例：

<!-- runnable: preserve-direction-twist -->
```python
import numpy as np
from servo_py import JointState, Servo, ServoConfig, TwistCommand, load_urdf

model = load_urdf("examples/planar2.urdf", base="base", tip="tool",
                  acceleration_limits=[3.0, 3.0])
twist = np.array([-0.3, 0.3, 0.0])

def worst_direction_error(preserve_direction):
    config = ServoConfig(task_axes=(0, 1), max_linear_speed=0.5,
                         preserve_direction=preserve_direction)
    servo = Servo(model, config)
    q, dq, worst = np.array([0.5, -1.0]), np.zeros(2), 0.0
    for tick in range(40):
        now = tick * 10_000_000
        command = TwistCommand(twist, [0.0, 0.0, 0.0], now)
        result = servo.step(JointState(q, dq, now), command, dt=0.01, now_ns=now)
        tcp = model.jacobian(q)[:3] @ result.reference.dq
        cosine = tcp @ twist / (np.linalg.norm(tcp) * np.linalg.norm(twist))
        worst = max(worst, np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))
        q, dq = result.reference.q, result.reference.dq
    return worst

per_joint, kept = worst_direction_error(False), worst_direction_error(True)
print(f"TCP direction error while accelerating: per-joint {per_joint:.1f} deg, "
      f"preserve_direction {kept:.6f} deg")
assert per_joint > 10 and kept < 1e-3
```

每个周期按以下顺序选取下一参考速度，与 MoveIt AccelerationLimitedPlugin 的思路一致，并额外考虑了位置包络：

1. 沿请求速度 `s · v_desired` 取可行区间内最大的 `s ∈ [0, 1]`；
2. 若该方向不可行，沿当前速度 `s · v_current` 取最小的可行 `s`，即在保持当前方向的前提下尽快减速；
3. 只有位置停止包络把两个方向都排除时，才退回逐关节裁剪，此时带 `POSITION_LIMIT`。

候选值都取自每个关节原有的可行区间，所以速度、加速度、位置和采样停止约束与默认模式相同。Stop 也沿当前方向减速，各关节在同一周期停下，代价是制动时间由最慢的关节决定。

命令方向突变时，参考会先沿原方向减速，直到新方向变得可行；方向变化越大，减速越多，接近 90° 的转向基本等同于先停再走。需要的是贴合路径而不是最短时间时启用此项。`ButterworthCommandFilter` 可以让输入方向渐变，两者常一起使用。

此选项只作用于默认参考生成器。与 `motion_generator`（如 Ruckig）同时设置时，构造 Servo 会抛出 ValueError，不会被静默忽略。

## 如何选择

| 现象 | 建议 |
|---|---|
| 手柄、SpaceMouse 或视觉目标抖动 | `ButterworthCommandFilter`，截止频率从 2–5 Hz 开始 |
| Twist / Pose 加速时末端偏离命令方向，或停止时轨迹弯曲 | `preserve_direction=True` |
| 需要限制 jerk | [RuckigSmoothing](smoothing.md)，可与命令滤波同时使用 |

---

下一步：[配置参数](configuration.md) · [Python API](api.md) · [执行契约](design.md)
