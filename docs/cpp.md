# 原生 C++ 接入

**目标：** 不依赖 Python 构建 ServoCore，并在自己的 CMake 工程中链接。要求 C++17、CMake 3.20+ 和 Eigen 3.4。

## 1. 构建与验证

```bash
cmake -S . -B build-native \
  -DSERVO_PY_BUILD_PYTHON=OFF \
  -DSERVO_PY_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-native -j2
ctest --test-dir build-native --output-on-failure
```

若 Eigen 不在系统查找路径，配置时追加 `-DCMAKE_PREFIX_PATH=/path/to/eigen/prefix`，换成本机实际目录。原生测试覆盖关节目标收敛、停止、QP 和采样；源码见 [cpp_smoke.cpp](https://github.com/OpenGHz/servopy/blob/main/tests/cpp_smoke.cpp)。

## 2. 安装到用户目录

```bash
cmake --install build-native --prefix "$PWD/dist/native"
```

在消费工程的 CMakeLists.txt 中添加以下接入片段，`app` 为自己的可执行目标：

```cmake
find_package(servo_py CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE servo_py::core)
```

配置消费工程时把 `dist/native` 的绝对路径加入 `CMAKE_PREFIX_PATH`。导出目标传递 Eigen 与 C++17 要求；头文件为 `servo_py/servo.hpp`。

## 3. 对齐 Python 与 C++ 名称

| Python | C++ |
|---|---|
| `Servo` | `ServoCore` |
| `ServoConfig` | `Config` |
| `JointLimits` | `Limits` |
| `JointState` | `State` |
| 各命令 dataclass | `Command`，由 `CommandType` 选择 |
| `StepResult` | `Result`，含 optional Reference |
| `SerialChainModel` | `SerialChain` |

C++ 不提供 Python URDF 加载器或命名命令映射。可以直接构建 Joint/SerialChain，或实现 Kinematics 连接自己的机器人模型。输入、坐标、时序和故障契约与 Python 保持一致。

## 扩展接口

头文件中的核心构造签名如下；这是接口参考，具体默认值与类型定义以头文件为准：

```cpp
ServoCore(std::shared_ptr<Kinematics> model, Limits limits, Config config = {},
          std::shared_ptr<DifferentialIK> differential_ik = nullptr,
          std::shared_ptr<MotionGenerator> motion_generator = nullptr);
```

`DifferentialIK::solve(const DifferentialIKRequest&)` 返回目标关节速度。请求已带加权任务和可行速度区间；内置 DLS、BoxQPSolver 都可直接使用。

`MotionGenerator` 必须实现以下签名：

```cpp
void reset();
MotionOutput generate(const Reference& start, const Vector& velocity,
                      const std::optional<Vector>& position,
                      double dt, const Limits& limits);
Reference sample(double elapsed) const;
```

`MotionOutput` 含 `reference`、`flags`、`braking`。生成器必须保证整个区间与后续停止轨迹满足自身声明的限位和连续性契约；内核会再校验端点，但端点检查不能替代连续轨迹验证。位置目标仅在合法、未受外部比例缩放的 JointPosition 分支提供。

仓库中的 RuckigSmoothing 是 Python 可选实现，不会自动成为独立 C++ 构建的依赖。需要纯 C++ 的 jerk 后端时应实现上述接口。

下一步：[执行契约](design.md) · [求解器行为](solvers.md) · [源码与贡献](contributing.md)
