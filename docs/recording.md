# 外部目标、记录与数值对照

**目标：** 在 Panda 运行期间持续输入实时目标，记录每周期反馈与控制输出，并复现或比较一次运行。前置：完成 [Panda 教程](mujoco-panda.md)；目标更新、伺服频率与断流行为见[实时伺服控制](realtime-servo.md)。

## 生成可运行的目标文件

以下完整例子生成 6 秒的 Panda 关节目标，更新频率 50 Hz。幅值小于内置完整演示，适合先验证数据接入。

<!-- runnable: panda-target-file -->
```python
import json
import math
from pathlib import Path

directory = Path("outputs")
directory.mkdir(exist_ok=True)
with (directory / "targets.jsonl").open("w") as stream:
    for tick in range(300):
        t = tick * 0.02
        q = [0.08 * math.sin(2 * math.pi * t / 6), 0, 0, -1.57079, 0, 1.57079, -0.7853]
        stream.write(json.dumps({"time": t, "type": "joint_position", "positions": q}) + "\n")
print("Created outputs/targets.jsonl")
```

然后运行：

```bash
python examples/mujoco_panda.py --headless --duration 8 \
  --control-mode joint-position --targets outputs/targets.jsonl \
  --log outputs/run.jsonl --metrics outputs/metrics.json
```

预期生成 800 条控制记录并最终 HOLD。目标文件结束后，最后目标过期并触发制动；日志每行对应一次控制调用。

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

## 完整示例：记录并重放

下面将一段理想关节控制写入临时日志，再用新建 Servo 重放。比较的是相同反馈和命令下的控制输出，不是重新推进物理。

<!-- runnable: record-and-replay -->
```python
import tempfile
from pathlib import Path
import numpy as np
from servo_py import (
    Action, JointJogCommand, JointState, JsonlRecorder, Servo,
    ServoConfig, StopCommand, load_urdf, read_records, replay,
)

model = load_urdf("examples/planar2.urdf", base="base", tip="tool",
                  acceleration_limits=[3.0, 3.0])
config = ServoConfig(task_axes=(0, 1))
servo = Servo(model, config)
q, dq = np.array([0.5, -1.0]), np.zeros(2)
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / "run.jsonl"
    with JsonlRecorder(path) as recorder:
        for tick in range(60):
            now = tick * 10_000_000
            state = JointState(q, dq, now)
            command = JointJogCommand([0.1, 0.0], now) if tick < 30 else StopCommand()
            result = servo.step(state, command, dt=0.01, now_ns=now)
            if result.action == Action.REJECT:
                raise RuntimeError(result.message)
            recorder.record(now, 0.01, state, command, result)
            q, dq = result.reference.q, result.reference.dq
    original = list(read_records(path))
    repeated = list(replay(Servo(model, config), path))
    assert len(repeated) == 60
    for row, result in zip(original, repeated):
        np.testing.assert_allclose(row["reference"]["q"], result.reference.q, atol=1e-12)
    print("Replayed 60 matching references")
```

日志 schema 1 包含 now_ns、dt、state、command、可选 collision、action、flags、reference。Pose 经过位置 IK 后，Panda 记录的是实际送入 Servo 的关节命令；要复现 IK 内部行为需要应用另存原始位姿目标和求解器信息。

## 数值对照

```bash
python examples/compare_references.py actual.jsonl expected.jsonl --atol 1e-6
python examples/compare_references.py actual.jsonl moveit-trajectory.json \
  --format joint-trajectory --joint-names joint1 joint2 joint3 joint4 joint5 joint6 joint7
```

第一种同时比较动作、flags 及 q/dq/ddq，允许不同起始时钟 epoch，但要求间隔和参考端点时间一致。第二种读取 ROS 2 `trajectory_msgs/JointTrajectory` 的 JSON 导出：`joint_names`、`points`，每个 point 必须含 `positions`、`velocities`、`accelerations` 和 `time_from_start: {sec, nanosec}`。关节名称自动重排，时间从记录第一个 now_ns 起算，每个参考端点对应一个 point，不静默插值。输出最大绝对差和 RMSE；超差退出码为 1。

这些工具和合成对照数据已可测试。真实 MoveIt 结果需要由调用方在其 ROS 环境导出；没有该数据和机器人，本项目不能声明已经完成逐步 MoveIt 等价或真机验证。

---

[文档目录](index.md) · [API 参考](api.md) · [排障](troubleshooting.md)
