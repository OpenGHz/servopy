# Panda / MuJoCo 仿真

**目标：** 通过真实动力学反馈比较三条控制路径，并学会查看跟踪结果。前置：安装 `.[mujoco]`。所有命令从仓库根目录执行。

通过 PyPI 安装后，无需源码仓库：安装 `servo-py[mujoco]`，将本页 `python examples/mujoco_panda.py` 替换为 **`servo-py-panda`** 即可，参数完全相同。Panda 模型单独下载，不包含在 wheel 或源码发行包中。首次上传状态见[安装指南](getting-started.md)。

## 先运行默认示例

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install '.[mujoco]'
python examples/mujoco_panda.py
```

默认打开 viewer，演示 18 秒，最后一秒发送 Stop。空格暂停/继续，关闭窗口或 Ctrl+C 退出。橙色为目标、青色为实际 TCP 路径。

![Panda 默认力矩模式](media/panda-servo.gif)

无桌面机器运行 `python examples/mujoco_panda.py --headless`。不录视频的 headless 模式不需要渲染上下文。

## 模型下载与离线运行

安装后的 Panda 示例首次启动时，从固定 Git 提交下载约 **5 MB** 的模型压缩包，验证 SHA-256 后存入 `~/.cache/servo-py/panda/<sha256>/panda.zip`。若设置了绝对路径的 `XDG_CACHE_HOME`，则使用该目录下的 `servo-py/panda/`。后续直接读取缓存；损坏的缓存会重新下载并校验。

`pip install`、`import servo_py`、基础 URDF 示例和 `servo-py-panda --help` 都不会下载模型。Git 克隆可直接使用 `examples/assets/panda.zip`，因此已有的源码示例仍可离线运行。

离线机器可从联网机器复制仓库中的压缩包，或者保存这个[固定版本的模型压缩包](https://raw.githubusercontent.com/OpenGHz/servopy/ede055d13e4c5f8a7475595ed90a2dc21bcdc2bb/examples/assets/panda.zip)，然后执行：

```bash
SERVO_PY_PANDA_ARCHIVE=/path/to/panda.zip servo-py-panda --headless
```

此路径优先于仓库文件和缓存，仍须通过内置 SHA-256 校验；若路径或校验错误，不会改为联网下载。来源清单和许可证随 Python 包提供；模型在内存中读取，不向安装目录写文件，也不解压网格到磁盘。

## 三种控制模式

| 模式 | 输入与求解 | 写给 MuJoCo 的量 | 是否调用位置 IK |
|---|---|---|---|
| `torque`（默认） | Pose → 微分 IK → 关节参考 | 力矩 PD 与模型 bias，单位 Nm | 否 |
| `joint-position` | 关节目标 → JointPosition 参考 | 原生位置执行器目标，单位 rad | 否 |
| `ik-position` | Pose → Python 位置 IK → JointPosition | 原生位置执行器目标，单位 rad | 是 |

```bash
python examples/mujoco_panda.py --control-mode joint-position
python examples/mujoco_panda.py --control-mode ik-position
```

`torque` 与 `ik-position` 跟踪固定朝向的空间八字位姿；直接关节模式使用关节正弦轨迹，画面中的目标路径由 FK 得到。因此不同模式的误差不能直接当作求解器优劣比较。

两种位控使用 Menagerie 模型原有 PD，不添加重力或期望速度前馈，负载下会保留静态误差。这里的 `JointPosition` 是 Servo 指令，`position` 执行器是 MuJoCo 的物理控制器，两者是不同层。

## 加入平滑与冗余控制

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install '.[mujoco,ruckig]'
python examples/mujoco_panda.py --control-mode ik-position --smoothing ruckig
python examples/mujoco_panda.py --differential-ik qp --nullspace-gain 0.1 --smoothing ruckig
```

`--max-jerk` 默认 30 rad/s³。三种模式均可用 Ruckig；`--differential-ik`、`--nullspace-gain` 和 `--joint-centering-gain` 仅用于 torque 模式的内核 Pose/Twist 路径。位置 IK 的替换方式见 [IK 教程](python-ik.md)。

## 外部目标与控制日志

```bash
python examples/mujoco_panda.py --control-mode joint-position --target-stdin --log run.jsonl
python examples/mujoco_panda.py --headless --control-mode joint-position --targets targets.jsonl --log run.jsonl
```

第一条接收实时 JSONL；第二条读取带仿真时间的目标文件。二者互斥。目标源默认 100 ms 超时；stdin EOF 或错误输入触发停止。可复制的目标生成例子、日志内容和回放见 [输入与记录](recording.md)。

## 录制视频与指标

```bash
mkdir -p outputs
MUJOCO_GL=egl python examples/mujoco_panda.py --headless \
  --control-mode ik-position --record outputs/panda.mp4 --metrics outputs/panda.json
```

这是 Linux EGL 录制方式，需要本机支持对应渲染后端。默认 960 × 640、30 fps；可用 `--width 640 --height 480 --fps 12` 调整。录制文件必须以 `.mp4` 结尾，尺寸必须为偶数且至少 320 × 240。输出到自己的目录，避免覆盖仓库基准录像。

`--record` 保存画面，`--metrics` 保存汇总，`--log` 保存每周期控制数据；它们用途不同，可以同时启用。[完整参数表](cli.md#panda-仿真)

## 如何读结果

| 字段 | 含义 |
|---|---|
| `completed` | 是否运行完计划时长；提前关窗时为 false |
| `position_rmse_m` / `position_max_error_m` | 全程实际 TCP 相对目标的误差，包括动态滞后 |
| `final_position_error_m` | 结束时的 TCP 误差，不能代表全程精度 |
| `max_joint_tracking_error_rad` | 实际关节反馈相对 Servo 参考的误差 |
| `ik_failures` / `last_ik_error` | 位置 IK 被拒绝的次数与最后原因 |
| `actions` / `flags` | Servo 行为与限制触发记录 |

默认配置的历史基线如下（Linux / Python 3.12 / MuJoCo 3.13.0、18 秒、未启用 Ruckig）：

| 指标 | torque | joint-position | ik-position |
|---|---:|---:|---:|
| TCP RMSE | 7.04 mm | 16.76 mm | 15.45 mm |
| 最终 TCP 误差 | 0.077 mm | 6.725 mm | 6.722 mm |
| 最终动作 | HOLD | HOLD | HOLD |

这些是特定配置的仿真观测，不是机器人精度规格。[原始默认模式指标](media/panda-servo.json) 与 [验证记录](validation.md) 保留了条件及限制。

`--duration` 最短为 6 秒，但缩短时长会加快内置轨迹。历史 6 秒 IK 位控运行曾拒绝 331 个解、最终误差约 180.5 mm；运行结束或 HOLD 不代表目标成功跟踪。首次体验请使用默认 18 秒。

## 控制与物理如何连接

Servo 以 100 Hz 使用 MuJoCo 的 qpos/qvel 反馈；物理积分为 500 Hz。执行器每个子步都通过 `sample_reference(t)` 读取参考，因此 Ruckig 不会被误当成恒加速度插值。初始化后不直接覆盖物理 qpos/qvel。

时间戳使用整数仿真时钟，渲染变慢不会伪造反馈超时。实时 stdin 模式会按墙钟节拍推进仿真，但不提供硬实时保证。MuJoCo 接触物理与 Servo 外部碰撞监控相互独立；本示例未提供 CollisionSample，因而会显示 `COLLISION_DISABLED`。

模型来自固定版本的 MuJoCo Menagerie，原始资产、许可证与校验信息保存在仓库的 [assets 目录](https://github.com/OpenGHz/servopy/tree/main/examples/assets)。夹爪保持张开；该示例不演示抓取任务或真机通信。

下一步：[记录与对照](recording.md) · [平滑原理](smoothing.md) · [显示与录制排障](troubleshooting.md)
