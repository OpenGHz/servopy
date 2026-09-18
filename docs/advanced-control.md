# 平滑、冗余控制、设备调度与记录

适用于 `servo-py 0.3.0`。基础依赖仍只有 NumPy；DLS/QP 在 C++ 中计算，Ruckig 是独立的可选 Python 后端。

## Ruckig 与参考采样

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install --upgrade '.[mujoco,ruckig]'
python examples/mujoco_panda.py --control-mode joint-position --smoothing ruckig --max-jerk 30
```

```python
from servo_py import Servo, RuckigSmoothing

servo = Servo(model, config, motion_generator=RuckigSmoothing(max_jerk=[30.] * model.dof()))
result = servo.step(state, command, dt=0.01, now_ns=now_ns)
# result 成功时，采样本次区间起点之后 2 ms 的实际参考。
sample = servo.sample_reference(0.002)
```

`max_jerk` 接受正有限标量或每关节向量，单位为 rad/s³ 或 m/s³。当前可选依赖固定为验证过的 `ruckig==0.12.2`，使用本地 state-to-state 计算，不调用 waypoint 云接口。上游接口说明见 [Ruckig 文档](https://docs.ruckig.com/)。

有效 JointPosition 目标使用位置轨迹规划，终点速度/加速度为零，不再使用比例增益来决定此模式的运动速度。目标连续关节仍使用最短角差、展开积分。Twist、Pose、JointJog、碰撞比例缩放及制动使用目标速度模式。Ruckig 的速度模式未直接限制位置，因此本实现解析计算各 jerk 段的位置和速度极值；每个输出区间都必须保留一个在限位内可执行的停止轨迹。

新目标不满足条件时，执行上一周期保留的停止轨迹；Stop/过期目标也沿该轨迹减速，保持加速度连续。初始状态连停止轨迹都不可行时返回 `REJECT | SMOOTHING_ERROR | INFEASIBLE`。生成器不改变实际反馈。复位清除平滑历史，初始参考加速度设为零；设备在恢复前必须停止，不能将 reset 当作运动中的平滑重规划。

采样周期可改变，停止轨迹按实际 dt 推进；`Servo` 的时序检查仍然生效。每个 Servo 必须独占一个 `RuckigSmoothing` 实例。`JERK_LIMIT` 表示启用了 jerk 受限生成；回退停止带 `SMOOTHING_FALLBACK`，其中由位置包络导致的回退另带 `POSITION_LIMIT`。这不是所有合法目标都可到达的全局规划器。

`sample_reference(t)` 只在最后一次成功生成的 `[0, dt]` 区间有效。默认后端为恒加速度；Ruckig 为分段恒 jerk，`ddq` 是端点加速度。设备不能继续套用 `q0 + v0*t + 0.5*result.ddq*t²` 来执行 Ruckig 轨迹。Panda 示例已经使用此采样接口。`REJECT` 或 reset 后不可继续采样旧参考。

## 微分 IK、QP 与零空间

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

## 周期调度与设备接口

```bash
python examples/periodic_servo.py --log periodic.jsonl
```

此示例使用理想参考跟随设备和真实单调时钟，执行一秒后制动；不是 MuJoCo 或真机。用 SDK 接入时提供 `Device` 协议实现，或用 `CallbackDevice` 绑定四个回调：

| 回调 | 契约 |
|---|---|
| `read_state(now_ns)` | 返回真实获取时间戳的 JointState，不能把过期数据刷新成新反馈 |
| `write_reference(reference, *, duration, sample)` | 发送有界区间；同步复制需要的采样，sample 在下一次 step 后失效 |
| `stop(reason)` | 取消设备旧缓冲，调用设备本身的停止机制 |
| `recover()` | 清除设备故障且不恢复旧缓冲；后续由 runner 获取新反馈并 reset |

```python
from servo_py import ServoRunner

runner = ServoRunner(servo, device, period=0.01)
# 在生产者线程更新目标；runner.run() 在唯一消费线程执行。
runner.commands.publish(command)
runner.run()  # 其他线程调用 runner.cancel() 请求受控停止。
```

邮箱始终只保留最新目标、复制数组、保留源时间戳，不排长队。断流使指令自然过期并制动；新的有效目标可以继续运动。`cancel()` 清除目标，忽略后续生产者输入并制动，参考 HOLD 且反馈速度接近零后调用设备 stop 并退出。`max_steps=N` 在 N 个周期后开始这一停止流程，实际总周期可能更多。完成或故障后使用 `recover()` 再运行；恢复不会重放目标。

调度基于绝对 deadline，避免相对 sleep 累积漂移。迟到预算不得大于 Servo 的 timing_tolerance。反馈 I/O、计算、输出或记录超预算，REJECT、异常及 Ctrl+C 都进入设备 stop，故障锁存；不补发过时周期。停止超过 `stop_timeout` 同样调用设备 stop。

Python 调度不是硬实时，无法中断阻塞驱动/求解器，也无法替代设备侧 watchdog。SDK 必须自行限定通信等待时间。这里未集成厂商通信协议，也未验证实际机器人制动性能。

## Panda 外部目标与回放

实时输入：

```bash
python examples/mujoco_panda.py --control-mode joint-position --target-stdin --log panda-live.jsonl
```

逐行发送 JSON；不带时间戳时使用接收时的仿真时间，有时间戳时保留源值。持续位控需定期发送新目标，默认 100 ms 超时。输入错误或 stdin EOF 清除目标并制动。无 viewer 时此模式也按墙钟节拍推进，且最后一秒仍执行停止。

```json
{"type":"joint_position","positions":[0.1,0,0,-1.57079,0,1.57079,-0.7853]}
{"type":"stop"}
```

`ik-position` 接收 `type: "pose"`、`pose: 4x4矩阵`，经过已有位置 IK 适配器；`torque` 接收 Pose 或 JointPosition；`joint-position` 接收 JointPosition。可用 `names` 指定完整关节名称排列。无法匹配控制模式的目标会制动并记录 `last_input_error`。

`--targets targets.jsonl` 从文件读相同目标，每行额外包含非负、递增的 `time`（仿真秒）。缺省 stamp_ns 取该 time；文件回放也保留超时，因此持续运动的目标需要周期更新。多个同一时刻目标只执行最后一个。例如：

```json
{"time":0.0,"type":"joint_position","positions":[0,0,0,-1.57079,0,1.57079,-0.7853]}
{"time":0.05,"type":"joint_position","positions":[0.01,0,0,-1.57079,0,1.57079,-0.7853]}
{"time":0.1,"type":"stop"}
```

```bash
python examples/mujoco_panda.py --headless --control-mode joint-position --targets targets.jsonl --log run.jsonl
```

`--log` 每周期记录实际反馈、进入 Servo 的命令（IK 模式是解算后的关节命令）、dt、时间戳、碰撞输入、动作、flags 和参考端点。它不包含 IK 内部迭代或完整 MuJoCo 状态。`replay(fresh_servo, path)` 重放记录的反馈与命令，需要使用相同模型、参数和后端；这是确定性控制回放，不是重新运行物理。

## 数值对照

```bash
python examples/compare_references.py actual.jsonl expected.jsonl --atol 1e-6
python examples/compare_references.py actual.jsonl moveit-trajectory.json \
  --format joint-trajectory --joint-names joint1 joint2 joint3 joint4 joint5 joint6 joint7
```

第一种同时比较动作、flags 及 q/dq/ddq，允许不同起始时钟 epoch，但要求间隔和参考端点时间一致。第二种读取 ROS 2 `trajectory_msgs/JointTrajectory` 的 JSON 导出：`joint_names`、`points`，每个 point 必须含 `positions`、`velocities`、`accelerations` 和 `time_from_start: {sec, nanosec}`。关节名称自动重排，时间从记录第一个 now_ns 起算，每个参考端点对应一个 point，不静默插值。输出最大绝对差和 RMSE；超差退出码为 1。

这些工具和合成对照数据已可测试。真实 MoveIt 结果需要由调用方在其 ROS 环境导出；没有该数据和机器人，本项目不能声明已经完成逐步 MoveIt 等价或真机验证。
