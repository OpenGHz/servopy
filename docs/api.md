# Python API 参考

本页对应 `servo_py.__version__ == "0.3.0"`。除可选 Pinocchio 后端外，下面的公共名称均可从 `servo_py` 导入。参数默认值见 [配置表](configuration.md)，按任务学习见 [文档目录](index.md)。

## 模型与限位

| 接口 | 输入与返回 |
|---|---|
| `load_urdf(path, *, base, tip, acceleration_limits, velocity_limits=None, margin=0.01)` | 返回原生 `SerialChainModel`；加速度上限必填，速度覆盖与 margin 均可选 |
| `Kinematics` | 可继承的模型接口；实现 `dof()`、`joint_names()`、`base_frame()`、`tip_frame()`、`fk(q)`、`jacobian(q)` |
| `SerialChainModel` | 原生串联模型，一般通过 `load_urdf` 创建；`limits` 给出原生限位副本 |
| `JointLimits(lower, upper, velocity, acceleration, margin=0.01)` | 前四项为 `(n,)`；margin 为标量或 `(n,)` |

`load_urdf` 的 `acceleration_limits`、`velocity_limits` 可按模型顺序给向量、按关节名给字典或给统一标量。前者不从 effort 推导；后者省略时采用 URDF 上限。单位和支持的关节类型见 [关键概念](concepts.md)。

`Kinematics.integrate(q, delta)` 和 `difference(q1, q0)` 默认使用标量关节加减。连续关节的自定义后端需要正确实现最短角差；FK 返回 `(4,4)`，Jacobian 返回 `(6,n)`。

可选后端从 `servo_py.backends.pinocchio` 导入 `PinocchioModel`，用 `PinocchioModel.from_urdf(path, base=..., tip=..., acceleration_limits=...)` 构建。需安装 `.[pinocchio]`。

## 控制器

```text
Servo(model, config=None, limits=None, *, differential_ik=None, motion_generator=None)
```

| 参数 | 说明 |
|---|---|
| `model` | Kinematics 实例；每步使用实际 q 做 FK/Jacobian |
| `config` | `ServoConfig`；省略时采用默认六维任务 |
| `limits` | 可选 `JointLimits` 覆盖模型限位；自定义模型无 `limits` 属性时必填 |
| `differential_ik` | `DifferentialIK` 实例；默认 DLS |
| `motion_generator` | `MotionGenerator` 实例；默认恒加速度参考生成 |

每个实例只有一个消费者；调用方不得并发 step/reset/sample。配置在构造时复制到内核，修改 Python config 不会动态调整运行中的控制器。

| 方法/属性 | 行为 |
|---|---|
| `step(state, command, *, dt, now_ns, collision=None) -> StepResult` | 计算下一周期参考；所有命令共用状态/时序检查 |
| `sample_reference(elapsed)` | 在上一成功区间 `[0, dt]` 内采样，elapsed 单位秒 |
| `reset(state, *, now_ns)` | 用新鲜有效反馈重置参考/故障历史；不能当作运动中的平滑切换 |
| `limits` | 有效 `JointLimits` 的独立副本 |
| `model`、`config` | Python 侧的模型与配置对象 |

`REJECT` 锁存故障；非法构造参数、Python 参数转换、并发调用和无效采样也可能直接抛异常。设备适配层须同时处理返回拒绝和异常。

## 状态与命令

以下为构造签名。数组接受 NumPy 数组或可转换的列表；除命令名称映射外均按模型关节顺序。

| 类型 | 构造签名与约束 |
|---|---|
| `JointState` | `(q, dq, stamp_ns)`；两个 `(n,)` 有限向量，时间是反馈获取时刻 |
| `JointPositionCommand` | `(positions, stamp_ns, names=None)`；完整目标，names 必须为完整唯一排列 |
| `JointJogCommand` | `(velocities, stamp_ns, names=None)`；命名子集允许，未列关节目标速度为零 |
| `TwistCommand` | `(linear, angular, stamp_ns, expressed_in="base", reference_point="tool")`；两个 `(3,)` |
| `PoseCommand` | `(pose, stamp_ns, expressed_in="base")`；刚体 `(4,4)`，仅 base 表达 |
| `StopCommand` | `()`；受约束停止，不需要源时间戳 |
| `CollisionSample` | `(velocity_scale, stamp_ns, state_stamp_ns)`；比例在 `[0,1]`，0 会拒绝并要求下游停止 |

CollisionSample 只是外部结果接口；本包不计算几何距离。结果时间和被检查状态的时间均需新鲜。

## 返回值

`StepResult(action, reference, flags, diagnostics, message)` 是只读 dataclass 包装。

| 字段 | 说明 |
|---|---|
| `action` | `Action.TRACK / BRAKE / HOLD / REJECT` |
| `reference` | 有效时含 `q`、`dq`、`ddq`、`stamp_ns`；REJECT 时为 None |
| `flags` | `SafetyFlag` 位集合，可同时包含多个原因 |
| `diagnostics` | 奇异值、缩放、任务误差等，见 [诊断表](status.md) |
| `message` | 人类可读说明，不应依赖其字符串做协议分支 |

参考数组读出时是副本；`stamp_ns` 为 `now_ns + round(dt * 1e9)`。默认 ddq 表示本段恒加速度；Ruckig ddq 表示端点加速度，执行时使用区间采样。

## 位置 IK

```text
PositionIKAdapter(servo, solver, *, max_joint_step=None,
                  position_tolerance=None, orientation_tolerance=None)
adapter.solve(command, q_seed, *, now_ns) -> PositionIKResult
```

`solver(target_pose, seed)` 返回关节解或 None。适配器使用创建时 Servo 的有效限位、任务轴和超时。两个容差为 None 时继承 Servo；max_joint_step 可为正标量/向量。完整语义见 [教程](python-ik.md)。

`PositionIKResult` 含 `command`、`message`、`position_error`、`orientation_error` 和计算属性 `success`。成功命令保留源时间戳；失败命令是 StopCommand。残差尚未计算时，误差字段为 None。

## 微分求解与轨迹后端

| 接口 | 契约 |
|---|---|
| `DifferentialIK.solve(request)` | 返回有限 `(n,)` 目标关节速度；Python 可继承 |
| `DifferentialIKRequest` | 字段为 `jacobian`、`task`、`q`、`lower`、`upper`、`preferred_velocity`、`damping` |
| `DampedLeastSquares()` | 默认阻尼最小二乘；内核随后执行限位 |
| `BoxQPSolver(max_iterations=200, tolerance=1e-9)` | 内置盒约束主动集 QP；无外部 QP 依赖 |
| `RuckigSmoothing(max_jerk)` | 可选 jerk 后端；正标量或 `(n,)`，每个 Servo 独占一个实例 |
| `MotionGenerator` | 自定义受信任后端，实现 reset/generate/sample，见 [C++ 扩展接口](cpp.md) |

请求的 J/task 已选轴、加权；bounds 已结合位置、速度、加速度。自定义求解器的异常或无效速度触发 SOLVER_ERROR；后端负责自己的整体轨迹契约。详见 [QP](solvers.md) 与 [Ruckig](smoothing.md)。

## 周期、设备与邮箱

```text
ServoRunner(servo, device, *, period=0.01, max_lateness=None,
            stop_timeout=5.0, commands=None, collision_source=None,
            recorder=None, clock_ns=time.monotonic_ns, sleep=time.sleep)
```

上式是签名参考，time 为标准库模块。`max_lateness=None` 默认取 Servo 的周期时序容差；显式值也不能超过它。

| 名称 | 行为 |
|---|---|
| `ServoRunner.run(max_steps=None)` | 阻塞到取消后静止或故障；max_steps 到期开始制动而非立即截断 |
| `ServoRunner.cancel()` | 清空目标并在后续周期受控停止 |
| `ServoRunner.recover()` | 设备恢复、读取停止反馈、重置 Servo；不重放旧目标 |
| `RunnerStats` | `cycles`、`deadline_misses`、`max_lateness_ns`、`max_cycle_ns`、`reason` |
| `LatestCommand` | `publish(command)`、`read()`、`clear()`；复制数据并保留源时间戳，`replaced` 统计替换 |
| `Device` | Protocol，要求 read_state / write_reference / stop / recover |
| `CallbackDevice` | 用同名关键字参数绑定四个 SDK 回调 |
| `SimulatedDevice(q)` | 理想参考跟随设备，无动力学，仅用于集成检查 |

`collision_source(state, now_ns)` 可返回每周期外部 CollisionSample。`recorder` 接受下述 record 调用。SDK 回调时间限制、写入采样与停止契约见 [运行指南](runtime.md)。

## 编码、记录与比较

| 接口 | 行为 |
|---|---|
| `command_to_dict(command)` | 转为 JSON 可序列化对象；拒绝非有限数值 |
| `command_from_dict(data, *, stamp_ns=None)` | 按 type 构造命令；缺省源时间戳时才采用传入时间 |
| `JsonlRecorder(path)` | 上下文管理器；以写模式打开文件，父目录需存在 |
| `record(now_ns, dt, state, command, result, collision=None)` | 向 recorder 写一条 schema 1 记录 |
| `read_records(path)` | 迭代 JSONL，检查 schema、基本字段和时间顺序 |
| `replay(servo, path)` | 向新建或已显式重置的 Servo 重放反馈与命令，迭代返回 StepResult |
| `compare_recordings(actual_path, expected_path, *, atol=1e-6)` | 比较对齐日志的动作、flags 和 q/dq/ddq |
| `compare_joint_trajectory(recording_path, trajectory_path, *, joint_names, atol=1e-6)` | 比较 ROS 2 JointTrajectory JSON 导出的对应端点 |

比较返回包含 `samples`、`errors`、`passed` 的字典；日志比较另含动作/flags 不匹配数。源码支持的导出字段、时间对齐和误差定义见 [记录教程](recording.md)。
