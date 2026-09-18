# 快速上手

**目标：** 安装 ServoPy，运行一个控制循环，再选择是否进入 Panda 仿真。无需 ROS、MoveIt 或物理机器人。

## 1. 准备环境

已验证环境为 Linux x86_64、Python 3.12、GCC 13、Eigen 3.4。项目声明 Python 3.10+；其他 Python/平台组合尚未逐一验证。源码构建需要 C++17 编译器，pip 会安装隔离构建所需的 CMake、pybind11 和 Eigen 头文件依赖。

```bash
git clone https://github.com/OpenGHz/servopy.git
cd servopy
python -m venv .venv
source .venv/bin/activate
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install .
python -c "import servo_py; print(servo_py.__version__)"
```

预期打印 `0.3.0`。后续命令均在这个已激活的环境、仓库根目录中执行。若已有克隆，先进入目录；更新 C++ 源码后必须重新安装，`git pull` 本身不会更新已安装的扩展。

## 2. 跑通基础示例

```bash
python examples/track_pose.py
```

该脚本用二维 URDF 模型跟踪一个末端位置目标，按 100 Hz 的整数仿真时钟计算 1,200 步。下一周期直接采用上一参考作为反馈，因此通常很快运行完成，不需要等待 12 秒。

预期结果的关键字段：

```json
{
  "model": "planar2",
  "feedback": "ideal",
  "steps": 1200,
  "final_position_error_m": 0.000098,
  "final_action": "HOLD",
  "collision": "disabled"
}
```

误差末位允许浮点差异。这是理想反馈示例；它验证接口与参考生成，不包含动力学、传感噪声或设备通信。保存曲线数据可运行 `python examples/track_pose.py --csv trajectory.csv`。

## 3. 选择需要的扩展

| 安装命令 | 增加的能力 |
|---|---|
| `python -m pip install '.[mujoco]'` | Panda 动力学、viewer、视频录制 |
| `python -m pip install '.[ruckig]'` | Ruckig 0.12.2 jerk 受限生成 |
| `python -m pip install '.[pinocchio]'` | Pinocchio 运动学后端 |
| `python -m pip install '.[test]'` | pytest 功能测试 |
| `python -m pip install '.[docs]'` | 本地文档预览与校验 |

可以组合，例如 `CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install '.[mujoco,ruckig]'`。MuJoCo、Pinocchio 和 Ruckig 都不是基础运行依赖。

## 4. 运行 Panda

有桌面显示时：

```bash
python -m pip install '.[mujoco]'
python examples/mujoco_panda.py --control-mode joint-position
```

无显示器时：

```bash
python examples/mujoco_panda.py --headless --control-mode joint-position
```

默认 18 秒仿真，最后进入停止阶段。viewer 中橙色是目标路径，青色是实际 TCP；按空格暂停。切换模式与记录视频见 [Panda 教程](mujoco-panda.md)。

## 5. 写自己的循环

先完成 [关节位置教程](joint-position.md)，再学习 [模型、坐标与时间约定](concepts.md)。需要实际硬件时，通过 [设备协议](runtime.md) 接入 SDK，并使用真实反馈替换理想回放状态。

## 安装失败时

| 现象 | 首先检查 |
|---|---|
| 找不到 C++ 编译器 | 本机是否安装 GCC/Clang；`CC`、`CXX` 是否指向存在的程序 |
| 编译进程被终止 | 用 `CMAKE_BUILD_PARALLEL_LEVEL=2` 限制并行度 |
| 更新后仍没有新 API | 当前 Python 路径、`servo_py.__file__`、是否重新安装 |
| 无法打开 viewer | 改用 `--headless`，或在有显示器的机器启动 |

更多诊断命令见 [排障指南](troubleshooting.md)。当前仓库没有承诺可直接下载的通用 wheel；不要把本地 CPython 3.12 / Linux wheel 当作跨平台分发包。
