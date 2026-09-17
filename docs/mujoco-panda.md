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

## 控制链路

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

模型原有的七个位置执行器在加载时改为力矩输入，保留相应的力矩边界；夹爪执行器保持原样。示例关节加速度上限设置为 `3 rad/s²`，是演示配置。Servo 对生成的参考施加限制，实际反馈和执行器力矩在仿真中分别检查。

MuJoCo 的接触物理与 Servo 的外部碰撞监控是两个独立功能。此示例没有提供 `CollisionSample`，结果会包含 `COLLISION_DISABLED`；目标球与可视化轨迹不参与碰撞。它没有演示自动避障或真机驱动。

如果希望把自己的 Python 位置 IK 放到上游，可保留这里的仿真反馈、力矩控制和 viewer，将生成 `PoseCommand` 的位置替换为 [Python IK 教程](python-ik.md) 中的 `JointJogCommand` 接法；注意该接法的笛卡尔约束边界。

## 录像与验证

README 的 [GIF](media/panda-servo.gif) 和 [MP4](media/panda-servo.mp4) 来自本示例实际仿真，未用预置关节动画替代控制。橙色为目标轨迹与当前目标球，青色为实际 TCP 轨迹；画面标注仿真时间、当前笛卡尔位置误差和 Servo 动作。

录制命令：

```bash
MUJOCO_GL=egl python examples/mujoco_panda.py --headless \
  --record docs/media/panda-servo.mp4 --metrics docs/media/panda-servo.json
```

[录制指标 JSON](media/panda-servo.json) 保存 MuJoCo 版本、控制频率、步数、位置误差、关节跟踪误差及最终动作。位置误差是同一仿真时刻实际 TCP 与移动目标的距离，包含位置闭环追踪移动目标的滞后；不能将最终静态误差当作全程误差。提前关窗的记录会标记 `completed: false`。

仓库中这次 18 秒录制使用 Linux / Python 3.12 / MuJoCo 3.13.0，以 EGL 渲染：

| 指标 | 本次结果 |
|---|---:|
| Servo 步数 | 1800 |
| 全程 TCP 位置 RMSE | 7.04 mm |
| 全程 TCP 最大位置误差 | 17.62 mm |
| 最终 TCP 位置误差 | 0.077 mm |
| 最大关节参考跟踪误差 | 0.001602 rad |
| 最终动作 | `HOLD` |

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

测试覆盖 TCP Jacobian 与位姿有限差分的一致性、运动学查询不污染物理状态，以及完整 18 秒动力学过程中的参考约束、实际运动和最终保持。测试不替代 viewer 的交互检查。

## 模型来源

模型来自 Google DeepMind 的 [MuJoCo Menagerie / Franka Emika Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/71f066ad0be9cd271f7ed58c030243ef157af9f4/franka_emika_panda)，固定到提交 `71f066ad0be9cd271f7ed58c030243ef157af9f4`。原始 MJCF、mesh 和许可证保存在 [examples/assets](../examples/assets/README.md)，使用 Apache-2.0；加载时增加的场景、TCP 与执行器修改集中在示例的 `load_panda()` 中。
