# 排障指南

先记录命令类型、`Action`、flags、message，再检查单位、关节顺序与时间域。以下处理应针对根因；不要先通过放大限位或反复 reset 让故障暂时消失。

## 安装与版本

| 症状 | 检查与处理 |
|---|---|
| 编译器不存在 | 检查本机 GCC/Clang 以及 CC/CXX 环境变量；若它们指向已删除的路径，改用实际编译器 |
| `std::optional` / `std::clamp` 不可用 | 确认编译器及标准库支持 C++17，并使用 C++17 或更新标准编译；完整版本范围见[安装要求](getting-started.md#1-准备环境) |
| 编译进程被杀死 | 限制并行：`CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install .` |
| `No module named servo_py` | 确认激活正确虚拟环境，并使用 `python -m pip` 安装 |
| 新源码却缺少 JointPosition/QP 等 API | 重新安装 C++ 扩展，检查实际导入路径 |
| Ruckig 导入失败 | 源码安装用 `.[ruckig]`；发行包用 `servo-py[ruckig]`，当前固定 0.19.4；ARM64 上游没有预编译包 |
| 可选测试被跳过 | 对应依赖未装，见 [安装选项](getting-started.md#3-选择需要的扩展) |

定位当前环境：

```bash
python -c "import sys, servo_py; print(sys.executable); print(servo_py.__version__); print(servo_py.__file__)"
python -m pip show servo-py
```

## Servo 拒绝或不运动

| 症状 | 优先排查 |
|---|---|
| `INVALID_TIMING` | now_ns 是否单调，下一次调用是否对应前一参考端点，dt 是否误用了毫秒 |
| `STALE_COMMAND` | 上游是否断流、IK 是否耗时过久；源消息时间戳应保留 |
| `STALE_STATE` / `FUTURE_TIMESTAMP` | 反馈与调用时钟是否同一时间域；不要混用墙钟与单调时钟 |
| `TRACKING_ERROR` | 设备是否实际执行了参考、插值是否正确、关节顺序/单位是否一致 |
| `FAULT_LATCHED` | 先停止设备、解决原始故障，再用新鲜停止反馈恢复 |
| `HOLD` 但没到目标 | 查看目标误差及 STALE/INVALID 标志；HOLD 只代表参考静止 |
| `SINGULARITY_HALT` | 检查任务维数、轴、模型与初始姿态；局部 IK 可能无法到达目标 |
| `INFEASIBLE` / `SMOOTHING_ERROR` | 初始速度与剩余距离可能不足以停止；核对状态和约束，而非删除限位 |

命令无效通常制动；反馈、模型或时序失效通常拒绝。完整表见 [状态参考](status.md)。

## IK 和零空间

| 症状 | 优先排查 |
|---|---|
| 位置 IK 始终无效 | 对齐 base/TCP、关节顺序、rad/m 与 FK；打印适配器 message |
| `max_joint_step` 被拒绝 | 检查分支、种子、目标跳变；连续性阈值不是速度上限 |
| IK 数值看似合理但残差失败 | IK 和 FK 是否用了同一工具偏置、同一模型、同一启用任务轴 |
| 换了 `Kinematics.solve()` 但行为没变 | 位置 IK 用 PositionIKAdapter；微分 IK 用 differential_ik 参数 |
| 零空间没有运动 | 任务是否已占满自由度、投影后是否为零、关节约束是否激活 |
| Pose 已到位仍 TRACK | 开启了次级姿态目标时，这是允许的行为 |

不要给 `Servo` 传 `ik_solver=`；这个参数仅出现在 Panda 示例包装层。

## Panda 显示、录制与偏差

| 症状 | 优先排查 |
|---|---|
| 提示无显示器 | 使用 `--headless`；需要 viewer 时在真实桌面环境运行 |
| EGL/OpenGL 初始化失败 | 检查本机图形驱动与 EGL；先去掉 `--record` 验证控制逻辑 |
| 输出视频被拒绝 | `.mp4` 后缀、偶数尺寸、最低 320×240、fps 1–100 |
| 位控最终仍有毫米级偏差 | 原始位置 PD 在重力负载下有稳态误差，参考到达不等于真实 TCP 精确到达 |
| 6 秒 IK 模式失败很多 | 恢复默认 18 秒；短时长使内置轨迹显著加快 |
| stdin 只发一行就停止 | 默认目标超时 100 ms，EOF 也停止；持续目标需周期发布，或使用定时目标文件 |
| 目标类型和模式不符 | joint-position 接 JointPosition；ik-position 接 Pose；torque 接两者 |

## 墙钟调度超时

`ServoRunner` 会在迟到、计算或 I/O 超出预算时请求设备停止，不补发过时周期。先记录 `RunnerStats`，把阻塞 I/O、复杂 IK 或同步日志耗时定位清楚。Python sleep 与回调没有硬实时保证，设备侧仍需要有界通信和独立 watchdog。

## 提供一个可复现的问题

请包含版本/导入路径、操作系统和 Python 版本、最小模型或关节限位、ServoConfig、命令与反馈时间戳、flags/message，以及最后几条控制记录。明确是理想回放、MuJoCo 还是实际设备。提交流程见 [贡献指南](contributing.md)。
