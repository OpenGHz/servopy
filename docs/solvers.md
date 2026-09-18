# 微分 IK、QP 与零空间

**目标：** 为实时伺服循环选择或替换 Pose/Twist 的逐周期求解器，并利用冗余关节调整姿态。前置：[实时伺服控制](realtime-servo.md)和[关键概念](concepts.md)；无需额外求解器依赖。

## 完整示例：在 Pose 分支使用 QP

<!-- runnable: qp-pose -->
```python
import numpy as np
from servo_py import (
    Action, BoxQPSolver, JointState, PoseCommand, Servo, ServoConfig, load_urdf,
)

model = load_urdf("examples/planar2.urdf", base="base", tip="tool",
                  acceleration_limits=[3.0, 3.0])
servo = Servo(model, ServoConfig(task_axes=(0, 1), max_linear_speed=0.5),
              differential_ik=BoxQPSolver())
q, dq = np.array([0.5, -1.0]), np.zeros(2)
target = model.fk(np.array([0.7, -0.7]))
for tick in range(1200):
    now = tick * 10_000_000
    result = servo.step(JointState(q, dq, now), PoseCommand(target, now),
                        dt=0.01, now_ns=now)
    if result.action == Action.REJECT:
        raise RuntimeError(result.message)
    q, dq = result.reference.q, result.reference.dq
assert result.action == Action.HOLD
assert np.linalg.norm(model.fk(q)[:2, 3] - target[:2, 3]) < 1.1e-4
print("QP pose target reached")
```

## 添加零空间姿态目标

下面的接入片段假定已有冗余 `model` 和合法的 `home_joints`。

```python
from servo_py import Servo, ServoConfig, BoxQPSolver

servo = Servo(model, ServoConfig(
    nullspace_gain=0.1,
    nullspace_reference=home_joints,
    joint_centering_gain=0.05,
), differential_ik=BoxQPSolver())
```

默认 `DampedLeastSquares` 保留原行为。`BoxQPSolver` 使用主动集法求解：

```text
min  0.5 ||J dq - task||² + 0.5 λ² ||dq - preferred_velocity||²
s.t. lower <= dq <= upper
```

`J` 和 `task` 已选择任务轴并加权；bounds 联合关节速度、下一周期加速度及采样制动位置约束。正阻尼使目标严格凸。QP 在约束内重新分配关节运动；它只支持这些盒约束，不包含任意不等式或碰撞约束。超过迭代上限会锁存 `SOLVER_ERROR`，不会发布未检查的迭代值。

姿态项为 `nullspace_gain * difference(posture, measured_q)`；关节居中项在有限上下界之间产生朝中点的速度。无界连续关节跳过居中项。两者经过未阻尼的正交零空间投影，再供 DLS/QP 使用。开启次级目标时，Pose 到达任务容差后可以继续零空间运动，因而 `GOAL_REACHED` 可以与 `TRACK` 同时出现。硬关节约束和奇异性减速可能改变最终末端运动，不能保证饱和状态下严格任务优先级。`task_residual` 是求解器输出的加权任务误差范数，早于后续奇异性/速度/平滑处理；`nullspace_speed` 是投影后的次级速度范数。

Python 可以继承 `DifferentialIK`，实现 `solve(request) -> dq`，再传给 `Servo(..., differential_ik=solver)`。request 包含 `jacobian`、`task`、`q`、`lower`、`upper`、`preferred_velocity` 和 `damping`。异常、维度错误、非有限输出触发 `SOLVER_ERROR`；有效输出仍通过内核限速/限位。Python 回调会获取 GIL。

C++ 提供相同的 `DifferentialIK`、`DampedLeastSquares`、`BoxQPSolver` 和可扩展 `MotionGenerator` 接口，不依赖 Python。自定义运动生成器属于受信任数值后端，必须保证整个参考区间和后续停止轨迹满足限位、连续性及其声明的 jerk 契约，内核只对最终参考再次校验；不能用任意输出滤波器替代它。

---

[文档目录](index.md) · [API 参考](api.md) · [排障](troubleshooting.md)
