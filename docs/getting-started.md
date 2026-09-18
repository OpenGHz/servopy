# 快速上手

**目标：** 安装 ServoPy，运行一个控制循环，再选择是否进入 Panda 仿真。无需 ROS、MoveIt 或物理机器人。

## 1. 准备环境

项目声明的安装与构建要求如下，版本范围以 [pyproject.toml](https://github.com/OpenGHz/servopy/blob/main/pyproject.toml) 和 [CMakeLists.txt](https://github.com/OpenGHz/servopy/blob/main/CMakeLists.txt) 为准：

| 组件 | 版本要求 | 用途 |
|---|---|---|
| Python | `>=3.10` | Python 接口与示例 |
| C++ 编译器及标准库 | 支持 C++17 或更新标准 | 编译原生内核和 Python 扩展；C++17 是语言标准下限 |
| CMake | `>=3.20` | 配置与构建 C++ 目标 |
| Eigen | Python 隔离构建使用 `cmeel-eigen>=3.4,<4`；原生 CMake 构建请求 3.4 或更新的兼容版本 | 编译期头文件依赖 |
| NumPy | `>=1.23` | 必需的第三方 Python 运行依赖 |

C++17 中的 17 指语言标准年份，GCC 13 等数字指编译器自身的版本。C++20/C++23 满足项目声明的标准下限；具体工具链和依赖组合仍需验证。

已验证环境包括 Linux x86_64、Python 3.12、GCC 13 和 Eigen 3.4，详细版本及覆盖范围见[验证记录](validation.md)。这些是用于复现实验的环境记录，不是额外的版本限制；其他 Python、平台与依赖组合尚未逐一验证。

默认的 pip 隔离构建会准备 scikit-build-core、pybind11 和 Eigen 头文件依赖，并按需获取符合要求的 CMake。本机仍需提供 C++ 编译器和系统开发工具。仅链接原生内核时无需 Python，见[原生 C++ 接入](cpp.md)。

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

可选依赖有各自的版本范围：MuJoCo 为 `>=3.2,<4`，Pinocchio 的 Python 发行包 `pin` 为 `>=3.0`，Ruckig 则固定为 `==0.12.2`。Ruckig 的等号表示实际的版本锁定；验证记录中的 MuJoCo 3.13.0 等具体版本只描述已测试环境。

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
