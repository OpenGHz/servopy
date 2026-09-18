# Ruckig 轨迹平滑

**目标：** 在速度和加速度约束之外限制 jerk，并按正确的轨迹执行每个周期。前置：完成 [关节控制](joint-position.md)，安装 `.[ruckig]`。

## 完整示例：从运动减速到静止

以下理想反馈示例在 1 秒后发 Stop，并检查相邻周期的加速度变化。执行到真实设备时仍需对区间进行采样。

<!-- runnable: ruckig-stop -->
```python
import numpy as np
from servo_py import (
    Action, JointJogCommand, JointState, RuckigSmoothing,
    Servo, ServoConfig, StopCommand, load_urdf,
)

model = load_urdf("examples/planar2.urdf", base="base", tip="tool",
                  acceleration_limits=[3.0, 3.0])
servo = Servo(model, ServoConfig(task_axes=(0, 1)),
              motion_generator=RuckigSmoothing(20.0))
q, dq, acceleration = np.array([0.5, -1.0]), np.zeros(2), np.zeros(2)
for tick in range(250):
    now = tick * 10_000_000
    command = JointJogCommand([0.1, 0.0], now) if tick < 100 else StopCommand()
    result = servo.step(JointState(q, dq, now), command, dt=0.01, now_ns=now)
    if result.action == Action.REJECT:
        raise RuntimeError(result.message)
    middle = servo.sample_reference(0.005)
    assert np.all(np.isfinite(middle.q))
    ref = result.reference
    assert np.max(np.abs(ref.ddq - acceleration)) <= 20.0 * 0.01 + 1e-8
    q, dq, acceleration = ref.q, ref.dq, ref.ddq
assert result.action == Action.HOLD
print("Stopped with continuous acceleration")
```

## Ruckig 与参考采样

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install --upgrade '.[mujoco,ruckig]'
python examples/mujoco_panda.py --control-mode joint-position --smoothing ruckig --max-jerk 30
```

以下是接入片段，`model` 与 `config` 使用现有控制器的配置：

```python
from servo_py import Servo, RuckigSmoothing

servo = Servo(model, config, motion_generator=RuckigSmoothing(max_jerk=[30.] * model.dof()))
result = servo.step(state, command, dt=0.01, now_ns=now_ns)
# result 成功时，采样本次区间起点之后 2 ms 的实际参考。
sample = servo.sample_reference(0.002)
```

`max_jerk` 接受正有限标量或每关节向量，单位为 rad/s³ 或 m/s³。当前可选依赖固定为 `ruckig==0.19.4`，使用本地 state-to-state 计算，不调用 waypoint 云接口。上游目前为 Linux x86_64 提供预编译包；ARM64 安装该可选项可能需要编译 Ruckig。历史验证记录保留原测试版本。上游接口说明见 [Ruckig 文档](https://docs.ruckig.com/)。

有效 JointPosition 目标使用位置轨迹规划，终点速度/加速度为零，不再使用比例增益来决定此模式的运动速度。目标连续关节仍使用最短角差、展开积分。Twist、Pose、JointJog、碰撞比例缩放及制动使用目标速度模式。Ruckig 的速度模式未直接限制位置，因此本实现解析计算各 jerk 段的位置和速度极值；每个输出区间都必须保留一个在限位内可执行的停止轨迹。

新目标不满足条件时，执行上一周期保留的停止轨迹；Stop/过期目标也沿该轨迹减速，保持加速度连续。初始状态连停止轨迹都不可行时返回 `REJECT | SMOOTHING_ERROR | INFEASIBLE`。生成器不改变实际反馈。复位清除平滑历史，初始参考加速度设为零；设备在恢复前必须停止，不能将 reset 当作运动中的平滑重规划。

采样周期可改变，停止轨迹按实际 dt 推进；`Servo` 的时序检查仍然生效。每个 Servo 必须独占一个 `RuckigSmoothing` 实例。`JERK_LIMIT` 表示启用了 jerk 受限生成；回退停止带 `SMOOTHING_FALLBACK`，其中由位置包络导致的回退另带 `POSITION_LIMIT`。这不是所有合法目标都可到达的全局规划器。

`sample_reference(t)` 只在最后一次成功生成的 `[0, dt]` 区间有效。默认后端为恒加速度；Ruckig 为分段恒 jerk，`ddq` 是端点加速度。设备不能继续套用 `q0 + v0*t + 0.5*result.ddq*t²` 来执行 Ruckig 轨迹。Panda 示例已经使用此采样接口。`REJECT` 或 reset 后不可继续采样旧参考。

---

[文档目录](index.md) · [API 参考](api.md) · [排障](troubleshooting.md)
