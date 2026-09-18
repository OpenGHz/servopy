# 实时控制循环与设备接入

**目标：** 用 `ServoRunner` 持续读取真实反馈、采用最新目标并发送控制输出，接入取消、超时和恢复行为。前置：[实时伺服控制](realtime-servo.md)。先用理想设备验证接口，再绑定自己的 SDK。

上游目标源和伺服循环可以独立运行：生产者在有新目标时更新邮箱，控制线程每周期读取最新命令与反馈。这样可以把遥操作、感知或上层任务接入正在运动的机器人。`ServoRunner` 提供 best-effort 周期调度，硬实时 deadline 仍不在当前保证范围内。

## 完整示例：可复现的调度回放

先用注入的仿真时钟验证接口。这个例子不会等待墙钟，也不驱动硬件；单次目标会自然过期，然后在步骤预算到期后结束。

<!-- runnable: runner-simulated-clock -->
```python
from servo_py import (
    JointPositionCommand, Servo, ServoConfig, ServoRunner,
    SimulatedDevice, load_urdf,
)

class Clock:
    now = 0
    def read(self):
        return self.now
    def sleep(self, seconds):
        self.now += round(seconds * 1e9)

model = load_urdf("examples/planar2.urdf", base="base", tip="tool",
                  acceleration_limits=[3.0, 3.0])
servo = Servo(model, ServoConfig(task_axes=(0, 1)))
clock = Clock()
device = SimulatedDevice([0.5, -1.0])
runner = ServoRunner(servo, device, clock_ns=clock.read, sleep=clock.sleep)
runner.commands.publish(JointPositionCommand([0.6, -0.8], clock.read()))
stats = runner.run(max_steps=100)
assert stats.deadline_misses == 0 and device.stopped
print(stats.cycles, stats.reason)
```

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

---

[文档目录](index.md) · [API 参考](api.md) · [排障](troubleshooting.md)
