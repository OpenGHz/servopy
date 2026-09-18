# 接入已有的 Python IK

适用于 servo-py `0.2.0`。已有 Python 位置 IK 可直接接入 `PositionIKAdapter`，由它校验并生成 `JointPositionCommand`。从 `0.1.0` 升级时先重新安装项目，以获得新版 C++ 绑定；之后更换自己的 Python 求解器不需要修改或重新编译内核。本文后半部分也保留旧版 JointJog 接法。

## 当前接口与接入位置

直接传给 `Servo.step()` 的 `PoseCommand`、`TwistCommand` 使用 C++ 内置的阻尼微分 IK。外部位置 IK 使用 `PositionIKAdapter(servo, solve_ik)` 准备关节位置指令；`Servo` 本身没有 `ik_solver=` 构造参数。`Kinematics` 提供 FK、Jacobian 和关节坐标运算，仅继承它并添加 `solve()` 不会自动接管 IK。

| 已有接口 | 当前版本的接法 |
|---|---|
| 位姿 IK：`solve(target_pose, q_seed) -> q_target` | 推荐 `PositionIKAdapter` → `JointPositionCommand`；旧版可使用 JointJog |
| 微分 IK：输出关节速度 `dq_target` | 校验后直接发送 `JointJogCommand` |
| Python FK / Jacobian | 实现 `Kinematics` 后端，继续使用 Servo 内置的微分 IK |

下面以第一种情况为例。应用层调用自己的 IK，`0.2.0` 的 C++ 关节位置分支负责位置反馈律和满足关节约束的短周期参考，下游控制器负责执行。

可直接运行的例子见 [Panda / MuJoCo 示例](mujoco-panda.md#三种控制模式)：`python examples/mujoco_panda.py --control-mode ik-position` 会打开 viewer，用 Python 位姿 IK 求解后通过原生关节位置指令生成参考，再发送给 Panda 位置执行器。`PandaSimulation(control_mode="ik-position", ik_solver=solve_ik)` 支持传入已有求解器；`--control-mode joint-position` 则跳过 IK，直接跟踪关节目标。

## 推荐接法：PositionIKAdapter

先创建正常的 `Servo`，再传入自己的求解函数。下面的模型、目标和 `your_ik` 应替换为应用自己的实现：

```python
import time
from servo_py import Action, PoseCommand, PositionIKAdapter, Servo, ServoConfig

servo = Servo(model, ServoConfig(joint_position_gain=2.0))
adapter = PositionIKAdapter(
    servo,
    lambda target, seed: your_ik.solve(target, seed),
    max_joint_step=0.35,       # 可选；转动关节用 rad，移动关节用 m。
    position_tolerance=1e-4,
    orientation_tolerance=1e-3,
)
previous_reference = None

# 以下是调用方周期循环中的一次执行。
try:
    state = controller.read_joint_state()
    seed = state.q if previous_reference is None else previous_reference.q
    prepared = adapter.solve(
        PoseCommand(target_pose, target_stamp_ns), seed,
        now_ns=time.monotonic_ns(),
    )
    # 即使 prepared.success 为 False，也要执行 StopCommand 以持续制动。
    result = servo.step(
        state, prepared.command, dt=0.01,
        now_ns=time.monotonic_ns(),  # IK 结束后重新采样，保留实际耗时。
    )
    if result.action == Action.REJECT:
        raise RuntimeError(result.message)
    controller.send_reference(result.reference)
    previous_reference = result.reference
except Exception:
    controller.stop_and_clear_queue()
    raise
```

`controller` 是应用的设备适配层；`0.3.0` 也提供 `ServoRunner`、`Device` 协议和 `CallbackDevice`，见 [高级控制接口](advanced-control.md)。本教程仍展示手动步进。`target_stamp_ns` 是上游目标的原始时间戳，与真实反馈和 `now_ns` 使用同一单调时钟。持续读取旧目标时不能刷新其时间戳。固定目标应由上层任务明确管理其持续有效、取消与更新。

`PositionIKResult` 提供 `success`、`command`、`message`、`position_error` 和 `orientation_error`。成功时 `command` 是完整的 `JointPositionCommand`；无解、异常或验证失败时为 `StopCommand`，且不会返回上次成功的解。两个误差字段是解的 FK 与目标在 `ServoConfig.task_axes` 上的残差，仅在完成残差计算时有数值，不是实际设备反馈误差。

适配器检查目标的时间戳/有效期/基坐标系/刚体变换、种子维度与有限值、解的维度/有限值/限位、可选的逐关节最大角差以及任务位姿残差。限位来自 `servo.limits`，包括构造 Servo 时传入的覆盖值和 margin。`max_joint_step` 可设为正标量或每关节向量；默认 `None`，不检查与初值的变化量。该阈值是局部连续性检查，不保证 IK 分支全局连续。

位姿残差默认使用 Servo 的位置、姿态容差，可通过适配器参数覆盖；只检查启用的任务轴。角差使用模型的 `difference()`，因此连续关节采用最短角差。传给求解器的是独立的目标和种子副本，修改它们不会影响原始输入。

成功指令保留原始 PoseCommand 的时间戳。适配器不读取时钟、不持有 Servo 参考、不启动线程，也不能中断阻塞的 Python 求解器。真实控制中务必在求解后以新鲜时间调用 `Servo.step()`；若超出命令、状态或调度预算，它会按现有契约制动或拒绝。设备侧仍需要断流停止机制。

## 接入前对齐数据约定

| 项目 | 约定 |
|---|---|
| 关节顺序 | IK 输出、反馈和模型均采用 `model.joint_names()` 的顺序 |
| 关节单位 | 转动关节为 rad，移动关节为 m；对应速度为 rad/s、m/s |
| 位姿 | 本教程采用 base 到 TCP 的 `4 × 4` 齐次变换矩阵；若原 IK 用四元数或其他格式，在包装函数中转换 |
| base / TCP | IK 的基坐标系与工具偏置应和模型一致 |
| 成功返回值 | 长度为 `model.dof()` 的一维、有限浮点数组 |
| 无解 | 本教程的包装接口统一返回 `None`；原 IK 的成功标志、多解列表等由包装层处理 |
| 时间戳 | 指令、反馈、碰撞结果使用同一个单调时间域，单位为 ns |

有 URDF 时，可以继续用 `load_urdf()` 创建模型。即使 IK 来自外部，Servo 仍需要模型的关节名称、限位和 `difference()` / `integrate()` 语义。速度及位置限位来自模型，加速度限位需要显式提供。

例如，在项目根目录中使用自带的合成六轴模型：

```python
from servo_py import Servo, load_urdf

model = load_urdf(
    "examples/arm6.urdf",
    base="base",
    tip="tool",
    acceleration_limits=3.0,
)
servo = Servo(model)
```

接入自己的机械臂时，替换 URDF、base、tip 和加速度参数。默认 `ServoConfig.task_axes` 有六个分量；如果模型少于六个自由度，仍需为当前配置校验设置合适的 `task_axes`，例如二维示例使用 `ServoConfig(task_axes=(0, 1))`。这项设置不会使 JointJog 启用笛卡尔求解。

## 兼容旧版：将位姿 IK 转换成 JointJog

下面的应用层包装仍可用于 `0.1.0`；`0.2.0` 新项目推荐上面的适配器，避免重复实现限位、残差和失败检查。

先把自己的 IK 包装成以下接口：

```python
def solve_ik(target_pose, q_seed):
    # your_ik 是你已有的 Python 求解器，替换为真实调用方式。
    # 在此处理成功标志、关节重排、单位转换和多解选择。
    # 成功时返回 q_target；无解时返回 None。
    return your_ik.solve(target_pose, q_seed)
```

下面的函数是应用层示例，不是 servo-py 新增的 API。它用关节误差生成速度目标：

`dq_target = kp * model.difference(q_target, q_seed)`

`kp` 的单位为 `1/s`，应根据机器人和周期调整；它独立于 `ServoConfig.position_gain`。后者用于内置的 Pose 分支。

```python
import time
import numpy as np
from servo_py import JointJogCommand, StopCommand


def step_with_python_ik(
    servo,
    solve_ik,
    state,
    target_pose,
    *,
    previous_reference=None,
    dt=0.01,
    kp=2.0,
    joint_tolerance=1e-4,
    collision=None,
):
    """执行一次外部 IK 和 Servo 步进；设备调度与输出由调用方处理。"""
    if not np.isfinite(kp) or kp <= 0:
        raise ValueError("kp must be finite and positive")
    if not np.isfinite(joint_tolerance) or joint_tolerance < 0:
        raise ValueError("joint_tolerance must be finite and nonnegative")

    model = servo.model
    # 首周期使用真实反馈，之后使用上一周期接受的参考作为连续求解初值。
    q_seed = np.asarray(
        state.q if previous_reference is None else previous_reference.q,
        dtype=float,
    ).copy()

    # 在求解前记录时间，耗时过长的结果不会被重新标记为“刚收到”。
    request_stamp_ns = time.monotonic_ns()
    q_target = solve_ik(target_pose, q_seed.copy())
    command = StopCommand()

    if q_target is not None:
        q_target = np.asarray(q_target, dtype=float)
        if q_target.shape != (model.dof(),) or not np.isfinite(q_target).all():
            raise ValueError("IK must return a finite joint vector or None")
        # 本示例使用模型默认限位，与上面的 Servo(model) 构造方式对应。
        limits = model.limits
        if np.any(q_target < limits.lower) or np.any(q_target > limits.upper):
            raise ValueError("IK solution is outside the model joint limits")

        error = model.difference(q_target, q_seed)
        if np.max(np.abs(error)) > joint_tolerance:
            command = JointJogCommand(
                velocities=kp * error,
                stamp_ns=request_stamp_ns,
            )

    return servo.step(
        state,
        command,
        dt=dt,
        now_ns=time.monotonic_ns(),
        collision=collision,
    )
```

`joint_tolerance` 是关节空间阈值，转动和移动关节分别按 rad、m 解释。混合关节模型通常应改成逐关节阈值。若构造 `Servo` 时显式传入了自定义 `JointLimits`，包装层也应使用同一套限位，而不是示例中的 `model.limits`。

这段示例将无解和到达阈值转换成 `StopCommand`。受控制动可能持续多个周期，应继续调用 `step()` 并发送返回参考，直到 `HOLD`。`HOLD` 仅表示参考已静止；确认机器人到位还需检查真实反馈和目标误差。

## 接入已有控制循环

每个控制周期按顺序读取真实反馈、调用上面的函数、检查结果、发送参考。下面的 `controller` 是需要自行实现的设备适配对象，方法名仅用于说明，不属于 servo-py：

```python
from servo_py import Action

# 在进入控制循环前初始化一次；成功 reset 后也要重新设为 None。
previous_reference = None

# 以下代码放在已有周期调度器的单次回调中。
try:
    state = controller.read_joint_state()  # 返回带实际采样时间戳的 JointState
    result = step_with_python_ik(
        servo,
        solve_ik,
        state,
        target_pose,
        previous_reference=previous_reference,
        dt=0.01,
    )
    if result.action == Action.REJECT:
        raise RuntimeError(result.message)
    controller.send_reference(result.reference)
    previous_reference = result.reference
except Exception:
    controller.stop_and_clear_queue()  # 取消旧轨迹并调用设备的停止接口
    raise
```

调度器应跨周期保留 `previous_reference`，不要每次重置它，也不要用它覆盖真实的 `state.q` / `state.dq`。Servo 内部会独立检查真实反馈与生成参考的跟踪误差。

`dt` 是下一段参考的计划时长；下一次 `step()` 应在上一次参考终点附近调用。上面的 `dt=0.01` 对应 10 ms 周期，不是要求在本次计算结束后再固定休眠 10 ms。时序、反馈过期或跟踪误差故障可能返回 `REJECT` 并锁存；排除原因后，用新鲜反馈显式调用 `servo.reset()`，并清空应用层的旧参考。异常同样需要进入设备停止流程，不能仅停止发送新数据。

## 连续性、目标有效期与耗时

- **多解与初值**：优先选取接近 `q_seed` 的有效解。`model.difference()` 能处理当前模型中连续转动关节的最短角差，但不会自动识别或消除肘部、腕部等 IK 分支跳变。包装层应根据具体机械臂检查解的连续性和允许的变化量。
- **解的正确性**：形状、有限值与关节限位校验无法证明 IK 满足目标。应检查求解器的成功状态，必要时用 FK 校验目标任务上的位置、姿态残差。
- **目标有效期**：示例假定 `target_pose` 是上层已确认仍有效的控制目标。持续输入的遥操作/视觉目标需要单独检查来源时间戳；不能重复求解旧输入并刷新指令时间戳来掩盖断流。固定目标的保持与取消由上层任务显式管理。
- **求解预算**：同步 Python IK 的耗时属于整个周期预算。例如 100 Hz 周期只有 10 ms，还需容纳反馈读取、Servo 计算和驱动通信。超时检测发生在 `step()` 被调用之后，无法中断一个正在阻塞的 IK 调用。
- **较慢的 IK**：可以在较低频率的任务中求解，让 Servo 周期消费最新有效结果。应保留结果的请求时间、目标版本和有效期，丢弃迟到或被新目标取代的结果；不能每次消费都刷新同一份旧结果的时间戳。设备侧仍需处理控制进程断流。

## 外部 IK 关节指令路径的功能边界

| 能力 | 外部 IK → JointPosition / JointJog |
|---|---|
| 关节速度、加速度、位置约束 | 保留，作用于生成的参考 |
| 指令/反馈超时、跟踪误差、故障锁存 | 保留 |
| 外部碰撞结果 | 可通过 `collision` 传入；启用时仍需检查结果及其源状态的时间戳 |
| 笛卡尔线速度/角速度限制 | 两种关节分支均不执行，需在外部设计 |
| 内置 Jacobian 奇异性减速/离开策略 | 仅在内置 Pose/Twist 求解分支执行 |
| 末端路径形状 | 关节空间跟踪不保证末端走直线，也不保证中间路径无碰撞 |

如果现有求解器已经输出 `dq_target`，直接构造 `JointJogCommand`，不用再乘位置误差增益；仍需对齐关节顺序、单位、结果有效期，并处理上述输出状态。

若希望自己的微分 IK 直接接管内核 `PoseCommand` / `TwistCommand` 的求解，同时复用内置笛卡尔约束和奇异性策略，还需要独立的内核求解器接口。当前的位置 IK 适配器不接管该分支，剩余工作见 [功能清单](roadmap.md)。

如需提供 Python FK / Jacobian，可参考 [Pinocchio 后端](../src/servo_py/backends/pinocchio.py)。该接口要求 FK 为 base→TCP 的 `4 × 4` 变换，Jacobian 为 `6 × n`，前三行是 TCP 线速度、后三行是角速度，均用 base 坐标轴表达。

相关约定见 [执行契约](design.md)、[Python API](../src/servo_py/api.py) 和 [C++ 求解分支](../cpp/servo.cpp)。
