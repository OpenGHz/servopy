# 接入 Python 位置 IK

**目标：** 把 `solve(base_T_tip, q_seed) -> q_target | None` 接入 Servo，保留目标时间戳，并在 IK 失败时继续制动。前置：[关键概念](concepts.md) 与 [关节控制](joint-position.md)。本文使用 0.3.0 接口。

## 先确认接入点

| 你已有的代码 | 对应接口 | 输出走向 |
|---|---|---|
| 给位姿求关节角 | `PositionIKAdapter(servo, solver)` | `JointPositionCommand` → Servo |
| 给 Jacobian/任务速度求关节速度 | `DifferentialIK.solve(request)` | Pose/Twist 内核分支，见 [求解器教程](solvers.md) |
| 只有 FK/Jacobian | `Kinematics` 后端 | 使用内置 DLS 或 QP |

`Servo` 没有 `ik_solver=` 参数。`PandaSimulation(..., ik_solver=...)` 是示例包装层的参数，二者不要混淆。

## 1. 跑通一个完整适配例子

本例对仓库二维机械臂使用解析位置 IK，固定选择肘部负角分支。只控制 x/y 任务，因此不检查未启用的姿态轴。模型两段长度都是 1 m；不要把这个教学求解器用于其他模型。

<!-- runnable: position-ik-loop -->
```python
import numpy as np
from servo_py import (
    Action, JointState, PoseCommand, PositionIKAdapter,
    Servo, ServoConfig, load_urdf,
)

model = load_urdf(
    "examples/planar2.urdf", base="base", tip="tool",
    acceleration_limits=[3.0, 3.0],
)
servo = Servo(model, ServoConfig(task_axes=(0, 1)))

def solve_ik(base_T_tip, q_seed):
    x, y = base_T_tip[:2, 3]
    cosine = (x*x + y*y - 2.0) / 2.0
    if abs(cosine) > 1.0:
        return None
    elbow = -np.arccos(cosine)
    shoulder = np.arctan2(y, x) - np.arctan2(np.sin(elbow), 1 + np.cos(elbow))
    return np.array([shoulder, elbow])

adapter = PositionIKAdapter(servo, solve_ik, max_joint_step=0.35)
q, dq = np.array([0.5, -1.0]), np.zeros(2)
target = model.fk(np.array([0.6, -0.8]))
for tick in range(600):
    now = tick * 10_000_000
    prepared = adapter.solve(PoseCommand(target, now), q, now_ns=now)
    # 失败时 prepared.command 是 StopCommand，仍然交给 Servo 执行。
    result = servo.step(JointState(q, dq, now), prepared.command, dt=0.01, now_ns=now)
    if result.action == Action.REJECT:
        raise RuntimeError(result.message)
    q, dq = result.reference.q, result.reference.dq  # 教学用理想反馈。

assert prepared.success
assert result.action == Action.HOLD
assert np.max(np.abs(q - [0.6, -0.8])) < 1.1e-4
print(prepared.message, result.action.name)
```

预期：打印 `IK solution accepted HOLD`。把 `solve_ik` 替换为自己的函数即可；输入、输出顺序都必须与 `model.joint_names()` 一致。

## 2. 适配器检查什么

1. 请求是 base 表达的刚体 Pose，时间戳有效且未过期。
2. 种子和解都是正确维度的有限关节向量。
3. 解满足 `servo.limits` 的有效边界与 margin。
4. 可选 `max_joint_step` 检查解与种子的逐关节差；连续关节用最短角差。
5. 解的 FK 在启用任务轴上满足位置、姿态残差容差。

传给求解器的是独立副本。无解、异常、越限、分支跳变或残差失败均返回新的 StopCommand，不重放旧解。`PositionIKResult` 的字段见 [API](api.md#位置-ik)。

`max_joint_step=None` 默认不检查连续性；正标量或 `(n,)` 向量可以开启检查。这个阈值限制相邻解的变化，不保证全局分支连续。位置/姿态容差默认继承 Servo，也可在构造适配器时覆盖。

## 3. 在实际控制循环中使用

以下是接入片段，假设应用已提供 `controller`、`adapter`、`servo`、上游 `target`、`target_stamp_ns` 和 `seed`：

```python
import time
from servo_py import Action, PoseCommand

try:
    state = controller.read_joint_state()
    prepared = adapter.solve(
        PoseCommand(target, target_stamp_ns), seed,
        now_ns=time.monotonic_ns(),
    )
    result = servo.step(
        state, prepared.command, dt=0.01, now_ns=time.monotonic_ns(),
    )
    if result.action == Action.REJECT:
        raise RuntimeError(result.message)
    controller.send_reference(result.reference)
except Exception:
    controller.stop_and_clear_queue()
    raise
```

IK 结束后重新读取当前时钟，使求解耗时参与超时判断。成功结果保留源目标时间戳；不能通过重新盖时间戳绕过过期。实际 SDK 应按 [设备协议](runtime.md) 处理参考采样、缓冲与停止。

适配器不持有参考、不启动线程，也不能打断阻塞求解器。异步 IK 的队列、超时和取消由应用安排。

## 4. 在 Panda 中替换求解器

直接运行：

```bash
python examples/mujoco_panda.py --control-mode ik-position
```

在应用代码中，可把自定义 `(target_pose, q_seed)` 函数传给示例的 `PandaSimulation(control_mode="ik-position", ik_solver=your_solver)`。示例默认采用局部迭代 DLS 位置 IK，并额外启用连续性与残差验证；其内部模型、种子与容差必须和你的求解器一致。

常见的不可达、分支跳变与时间域问题见 [排障](troubleshooting.md)。旧版通过 JointJog 手写位置反馈的迁移方法见 [更新记录](changelog.md)。
