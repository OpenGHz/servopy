# Panda 的 MuJoCo Servo 示例

示例入口：[examples/mujoco_panda.py](../examples/mujoco_panda.py)。它用 MuJoCo 模拟 Panda 的实际运动，用 `servo-py` 生成关节参考，默认启动交互式 viewer。完整运行包含目标跟踪、回到起点、稳定保持和显式停止。

## 安装与启动

在项目源码根目录、已激活的 Python 环境中运行：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install '.[mujoco]'
python examples/mujoco_panda.py
```

Ubuntu 22.04 可使用系统 Python 3.10 从源码构建；如果尚未安装编译器和虚拟环境支持，先安装 `build-essential`、`python3-dev` 和 `python3-venv`。`CMAKE_BUILD_PARALLEL_LEVEL=2` 将本地编译并发限制为 2；它不改变仿真运行频率。项目此前交付的 Ubuntu 24.04 / CPython 3.12 wheel 不适用于此环境。

MuJoCo、图像与视频依赖属于可选的 `mujoco` extra。模型资源随源码提供，启动时校验并在内存中读取，不访问网络。只安装 wheel 的用户还需取得仓库中的 `examples/`；示例和模型没有放进 Python 运行时包。

默认运行 18 秒后关闭 viewer。空格暂停/继续；鼠标可操作 MuJoCo 相机；关闭窗口或按 Ctrl+C 提前结束。在 macOS 上，按照 [MuJoCo Python viewer 文档](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer) 使用：

```bash
mjpython examples/mujoco_panda.py
```

Linux 桌面运行需要可用的显示服务和 OpenGL 驱动。SSH/容器中没有显示服务时使用 `--headless`，不会尝试启动 viewer：

```bash
# 只计算物理与控制，不渲染，不需要图形上下文。
python examples/mujoco_panda.py --headless --metrics panda-metrics.json

# 无窗口录制，使用 EGL；需要系统支持 EGL 的驱动。
MUJOCO_GL=egl python examples/mujoco_panda.py --headless \
  --record panda-servo.mp4 --metrics panda-servo.json

# 在桌面上边看边录制。
python examples/mujoco_panda.py --record panda-servo.mp4
```

`--duration` 设置仿真时长，最短 6 秒；`--width`、`--height` 和 `--fps` 设置录像，默认 `960 × 640 / 30 fps`。缩短时长会加快目标运动，Servo 的速度和加速度约束仍有效，因此可能产生更大的跟踪滞后。`--headless` 按计算能力尽快推进；viewer 模式尝试按真实时间播放，机器负载较高时播放会变慢。

## 三种控制模式

`--control-mode` 在启动时选择模式，默认仍为 `torque`。以下命令都会打开 viewer，也均可组合 `--headless`、`--record` 和 `--metrics`。

```bash
# 原有模式：笛卡尔 Servo + 关节力矩 PD。
python examples/mujoco_panda.py --control-mode torque

# 直接给定七关节位置目标，无需 IK。
python examples/mujoco_panda.py --control-mode joint-position

# 先由 Python IK 解出关节角，再用相同的关节位控链路执行。
python examples/mujoco_panda.py --control-mode ik-position
```

| 模式 | 目标与 Servo 链路 | MuJoCo 执行器输入 |
|---|---|---|
| `torque` | 八字 TCP 位姿 → `PoseCommand` → 内置微分 IK → 关节参考 | PD 与偏置补偿计算的力矩，单位 Nm |
| `joint-position` | `target_joints()` → 关节误差转速度 → `JointJogCommand` → 关节参考 | 模型原有位置执行器，目标角度单位 rad |
| `ik-position` | 八字 TCP 位姿 → `PandaPositionIK` → 关节误差转速度 → `JointJogCommand` → 关节参考 | 与直接关节模式相同的位置执行器 |

两个位控模式的应用层使用 `dq_target = 10 * (q_target - q_reference)`，其中 `q_reference` 是上一周期接受的参考位置；它与真实反馈分别保存。Servo 继续检查真实反馈并生成受位置、速度和加速度约束的参考，再以 500 Hz 插值写入执行器 `data.ctrl`。这里的 `JointJogCommand` 是包内部用于参考生成的速度目标，下游实际发送给 MuJoCo 的仍然是关节位置。不会直接修改物理状态 `qpos/qvel`。

`joint-position` 的示范目标由七个关节的平滑周期函数给出，运动后回到初始角度。替换 `target_joints(home_q, t, duration)` 即可输入自己的关节目标，顺序是 `joint1` 到 `joint7`，单位 rad，并须处于含 margin 的 Servo 限位内。此模式只为显示目标路径与计算 TCP 误差调用 FK，不调用 IK/Jacobian；目标 TCP 朝向随关节运动变化，路径也不同于另两个模式的八字轨迹。

位控保留 Menagerie 模型原始位置执行器的 PD 增益、角度范围与力矩上限，初始化时把 `ctrl` 设为 home 角度。MuJoCo 根据位置误差和速度生成执行器力，再做动力学积分，参见 [MuJoCo 执行器模型](https://mujoco.readthedocs.io/en/stable/computation/index.html#actuation-model)。此路径未添加重力或期望速度前馈补偿，因此存在运动滞后和重力稳态偏差；`HOLD` 表示 Servo 参考停止，不能据此认定实际关节与目标完全一致。

### IK 求解与替换

`PandaPositionIK(target_pose, q_seed)` 是应用层 Python 位姿 IK：用 MuJoCo 的 FK/Jacobian 迭代阻尼最小二乘，最多 40 次更新，每次关节角更新不超过 0.15 rad，结果限制在带 margin 的关节范围内。位置、朝向求解阈值分别为 `1e-5 m` 与 `1e-4 rad`。每个周期用上一周期接受的关节参考作为初值。它适合演示这条连续轨迹，不保证任意目标都能求解。

示例接受已有 Python IK 的可调用对象，接口为 `solve_ik(world_T_tcp, q_seed) -> q_target | None`。在源码根目录可直接运行下面的控制循环（不启动 viewer）：

```python
from examples.mujoco_panda import PandaSimulation

def solve_ik(target_pose, q_seed):
    # 替换为已有 IK；须统一关节顺序、rad 单位以及 world/TCP 坐标系。
    return your_ik.solve(target_pose, q_seed)

simulation = PandaSimulation(control_mode="ik-position", ik_solver=solve_ik)
while simulation.time < simulation.duration:
    simulation.step()
print(simulation.summary())
```

要继续使用示例的 viewer/录制循环，可在 `main()` 构造 `PandaSimulation(...)` 时传入同一个 `ik_solver=solve_ik`，或将 `PandaPositionIK.__call__()` 替换为自己的求解调用。

外部解还会经过有限值、七关节维度、margin 限位、与初值最大角差 `0.35 rad`、FK 位置残差 `1e-4 m` 和朝向残差 `1e-3 rad` 检查。无解、求解异常或校验失败会发送 `StopCommand`，继续执行 Servo 的制动/保持参考，并增加 `ik_failures`、记录 `last_ik_error`；下一周期会重试新目标。`0.35 rad` 是本示例的连续性阈值，不是通用 IK 分支识别算法。若 Servo 返回 `REJECT`，仿真会终止，位置执行器不会收到全零关节角目标。

建议先使用默认 18 秒时长。把整条轨迹压缩到 6 秒等较短时间，可能让目标远超当前可执行参考，触发上述角差阈值并保持制动；此时仿真完成或最终 `HOLD` 不代表成功跟踪，应同时查看 `ik_failures` 与 TCP 误差。

这两个位控模式走 JointJog 分支，保留关节约束、反馈/指令检查和跟踪误差保护，但不执行内置的笛卡尔限速或 Jacobian 奇异性减速；IK 自身的阻尼不等同于这些 Servo 策略。接口细节、耗时与目标有效期约定见 [Python IK 教程](python-ik.md)。

## 控制链路

以下表格描述默认的 `torque` 模式；两种位置模式使用上节的目标与执行器链路。

| 环节 | 示例中的实现 |
|---|---|
| 目标 | `target_pose()` 生成平滑的空间八字位置轨迹，保持初始 TCP 朝向 |
| 实际反馈 | 每 10 ms 读取仿真的七个臂关节 `qpos` 和 `qvel` |
| 运动学 | `PandaKinematics` 用 MuJoCo 计算 TCP 的 FK 和 `6 × 7` Jacobian |
| Servo | `PoseCommand` → 内置阻尼微分 IK → 约束后的关节位置、速度、加速度参考 |
| 下游控制 | 以 2 ms 为步长插值参考，用关节 PD 加 `qfrc_bias` 补偿计算力矩，并限幅 |
| 物理积分 | `mj_step()` 计算由力矩驱动的下一时刻关节状态 |
| 结束 | 最后 1 秒发送 `StopCommand`；继续推进物理并执行返回的制动/保持参考 |

初始化后，控制循环不会把 Servo 参考直接写入仿真的 `qpos` 或 `qvel`。参考与实际状态分别保存，电机和动力学决定实际运动。因此这里能观察到跟踪滞后，并检查 Servo 的跟踪误差保护。

`PandaKinematics` 另建一份 `MjData` 进行 FK/Jacobian 查询，避免 IK 和奇异性探测修改正在运行的物理状态。Jacobian 的行顺序为 `[vx, vy, vz, wx, wy, wz]`，在世界坐标系表达，线速度参考点是 TCP。TCP 位于 `hand` 坐标系的 `[0, 0, 0.1034]` 米处；只控制七个臂关节，手指保持张开。

时间戳使用整数仿真时钟，Servo 固定 100 Hz，物理积分固定 500 Hz。渲染、窗口拖动或录制变慢不会产生虚假的反馈超时。本示例的 Python 回调、休眠和 viewer 不提供硬实时保证。

仅 `torque` 模式在加载时把模型原有的七个位置执行器改为力矩输入，保留相应的力矩边界；两种位控保留原位置执行器。所有模式的夹爪执行器都保持原样。示例关节加速度上限设置为 `3 rad/s²`，是演示配置。Servo 对生成的参考施加限制，实际反馈和执行器力矩在仿真中分别检查。

MuJoCo 的接触物理与 Servo 的外部碰撞监控是两个独立功能。此示例没有提供 `CollisionSample`，结果会包含 `COLLISION_DISABLED`；目标球与可视化轨迹不参与碰撞。它没有演示自动避障或真机驱动。

## 录像与验证

README 的 [GIF](media/panda-servo.gif) 和 [MP4](media/panda-servo.mp4) 来自本示例默认力矩模式的实际仿真，未用预置关节动画替代控制。橙色为目标轨迹与当前目标球，青色为实际 TCP 轨迹；画面标注仿真时间、当前笛卡尔位置误差和 Servo 动作。新录制的画面还会显示所选控制模式。

录制命令：

```bash
MUJOCO_GL=egl python examples/mujoco_panda.py --headless \
  --record docs/media/panda-servo.mp4 --metrics docs/media/panda-servo.json
```

[录制指标 JSON](media/panda-servo.json) 保存 MuJoCo 版本、控制频率、步数、位置误差、关节跟踪误差及最终动作。位置误差是同一仿真时刻实际 TCP 与移动目标的距离，包含位置闭环追踪移动目标的滞后；不能将最终静态误差当作全程误差。提前关窗的记录会标记 `completed: false`。

新运行还会记录 `control_mode`、`actuator_mode`、`actuator_command_units`、`ik_failures` 和 `last_ik_error`。`max_joint_tracking_error_rad` 是实际反馈与 Servo 参考的偏差，区别于目标轨迹的 TCP 误差。原有录制 JSON 保留当时的数据格式。

仓库中这次 18 秒录制使用 Linux / Python 3.12 / MuJoCo 3.13.0，以 EGL 渲染：

| 指标 | 本次结果 |
|---|---:|
| Servo 步数 | 1800 |
| 全程 TCP 位置 RMSE | 7.04 mm |
| 全程 TCP 最大位置误差 | 17.62 mm |
| 最终 TCP 位置误差 | 0.077 mm |
| 最大关节参考跟踪误差 | 0.001602 rad |
| 最终动作 | `HOLD` |

新增位控模式在同一 MuJoCo 3.13.0 / Python 3.12 环境下各运行 18 秒、1800 个 Servo 周期的结果：

| 指标 | `joint-position` | `ik-position` |
|---|---:|---:|
| 全程 TCP 位置 RMSE | 16.76 mm | 15.45 mm |
| 全程 TCP 最大位置误差 | 34.18 mm | 39.33 mm |
| 最终 TCP 位置误差 | 6.725 mm | 6.722 mm |
| 最大关节参考跟踪误差 | 0.02964 rad | 0.05251 rad |
| IK 失败次数 | 不使用 IK | 0 |
| 最终动作 | `HOLD` | `HOLD` |

两个模式的目标路径不同，这个表只记录各自结果，不构成求解器精度的直接比较。位控的最终偏差主要来自上述原始 PD 的重力负载。可用下面的命令复现实测或录制：

```bash
python examples/mujoco_panda.py --headless --control-mode joint-position \
  --metrics panda-joint-position.json
MUJOCO_GL=egl python examples/mujoco_panda.py --headless --control-mode ik-position \
  --record panda-ik-position.mp4 --metrics panda-ik-position.json
```

如需重新生成 README GIF，安装 FFmpeg 后运行：

```bash
ffmpeg -y -i docs/media/panda-servo.mp4 \
  -filter_complex '[0:v]fps=12,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3' \
  -loop 0 docs/media/panda-servo.gif
```

自动验证无需图形上下文：

```bash
python -m pip install '.[mujoco,test]'
python -m pytest -q tests/test_mujoco_panda.py
```

测试覆盖 TCP Jacobian 与位姿有限差分的一致性、运动学查询不污染物理状态、三种模式各自完整 18 秒的参考约束/执行器力限幅/实际运动/最终保持，以及位置执行器的输入语义、IK 位姿残差、不可达目标、外部无效解/异常时的受控制动和 `REJECT` 后不发送零角度。测试不替代 viewer 的交互检查。

## 模型来源

模型来自 Google DeepMind 的 [MuJoCo Menagerie / Franka Emika Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/71f066ad0be9cd271f7ed58c030243ef157af9f4/franka_emika_panda)，固定到提交 `71f066ad0be9cd271f7ed58c030243ef157af9f4`。原始 MJCF、mesh 和许可证保存在 [examples/assets](../examples/assets/README.md)，使用 Apache-2.0；加载时增加的场景、TCP 与执行器修改集中在示例的 `load_panda()` 中。
