# 命令行参考

以下源码脚本从仓库根目录运行，支持 `--help`。安装 wheel 后也可从任意目录用 `python -m servo_py.examples.<模块名>` 运行，例如 `python -m servo_py.examples.track_pose`；Panda 另提供 `servo-py-panda` 命令。以下默认值对应 0.3.0；参数组合示例见相应教程。

## Panda 仿真

`python examples/mujoco_panda.py [options]`，需安装 `.[mujoco]`。

发行包对应 `python -m pip install 'servo-py[mujoco]'` 和 `servo-py-panda [options]`。所有参数一致，URDF 和 Panda 资产随包提供。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--control-mode` | `torque` | `torque` / `joint-position` / `ik-position` |
| `--headless` | 关闭 | 不启动 viewer；无实时 stdin 时尽快运行 |
| `--duration` | `18` | 仿真秒，至少 6；内置轨迹会按时长变化 |
| `--smoothing` | `none` | `none` / `ruckig`；后者需 `.[ruckig]` |
| `--max-jerk` | `30.0` | Ruckig 每关节 jerk，rad/s³ |
| `--differential-ik` | `dls` | `dls` / `qp`；非默认设置仅用于 torque 模式 |
| `--nullspace-gain` | `0.0` | 朝 home 姿态的次级增益，仅 torque |
| `--joint-centering-gain` | `0.0` | 关节居中增益，仅 torque |
| `--target-stdin` | 关闭 | 实时 JSONL 输入，headless 也按墙钟推进 |
| `--targets` | 无 | 带 time 字段的 JSONL 文件，与 stdin 互斥 |
| `--log` | 无 | 写入每周期控制记录 |
| `--record` | 无 | 写 MP4，需可用渲染后端 |
| `--metrics` | 无 | 写跟踪汇总 JSON |
| `--width` | `960` | 录制宽度，偶数且至少 320 |
| `--height` | `640` | 录制高度，偶数且至少 240 |
| `--fps` | `30` | 录制帧率，1–100 |

viewer 的空格键暂停/继续。更多行为与退出解释见 [Panda 教程](mujoco-panda.md)；输入格式见 [记录教程](recording.md)。

## 位姿理想回放

`python examples/track_pose.py [--csv path]`

固定二维模型、1,200 步，无需图形环境。`--csv` 写时间、关节位置/速度、TCP 位置、误差和 flags。正常完成退出码 0；REJECT 会抛异常。

## 墙钟周期示例

`python examples/periodic_servo.py [--log path]`

`--log` 默认 `periodic-servo.jsonl`。使用理想设备、10 ms 周期；100 步后开始受控停止，总步数可能超过 100。宿主机迟到超预算时调用设备停止并报 TimeoutError。这验证调度策略，不承诺任意主机均满足 100 Hz。

## 参考数值对照

```bash
python examples/compare_references.py actual.jsonl expected.jsonl --atol 1e-6
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `actual` / `expected` | 必填 | 被检查记录与参考记录/导出文件 |
| `--format` | `recording` | `recording` / `joint-trajectory` |
| `--joint-names` | 无 | JointTrajectory 对照时必填，给出 Servo 的关节顺序 |
| `--atol` | `1e-6` | 每个 q/dq/ddq 数值的绝对差阈值，按各字段单位解释 |
| `--output` | 无 | 另存报告 JSON |

正常比较通过退出码为 0，超差为 1；格式或时间不对齐会抛异常，不会自动重采样。实际 MoveIt 对照需要先导出数据，见 [对照格式](recording.md#数值对照)。

## 计算耗时基准

`python examples/benchmark.py [--steps 5000] [--output path]`

先热身，再测 Python 到 C++ 的单次 Servo.step 耗时。`--steps` 为测量次数，`--output` 另存 JSON。结果不包括设备 I/O、周期等待或完整硬件控制链；不要把它等同于调度频率保证。

## 文档工具

```bash
python scripts/check_docs.py
python scripts/check_docs.py --run
python -m mkdocs build --strict
python -m mkdocs serve
```

检查脚本需 `.[docs]`；`--run` 还需 `.[ruckig]`，会在独立 Python 进程中执行标记的完整示例。默认检查本地链接、锚点、API/配置/flags 覆盖与 CLI 参数，不访问外部网页或驱动设备。
