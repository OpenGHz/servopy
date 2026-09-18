# 更新与迁移

此处按功能版本记录变化；验证环境与数值结果集中保留在 [验证记录](validation.md)。这些条目描述源码版本，不表示已向 PyPI 发布或提供跨平台 wheel。

## 首次 PyPI 发布准备

- 增加 manylinux_2_28 的 CPython 3.10–3.14 / x86_64、ARM64 wheel 流水线，从 sdist 构建并验证完整产物。
- wheel 包含示例和小型 URDF，新增 `servo-py-panda` 入口及 `python -m servo_py.examples.track_pose`。
- 从 wheel 和 sdist 排除 Panda 模型压缩包；示例首次使用时下载固定模型，经 SHA-256 校验后缓存，支持离线档案路径与缓存复用。
- 可选 Ruckig 从 0.12.2 升至 0.19.4，覆盖较新 Python 的上游 x86_64 wheel；ARM64 不承诺该可选项免编译。
- 增加 Ubuntu 二进制安装检查、独立 PyPI README，以及 TestPyPI / PyPI Trusted Publishing 流程。
- 首次上传尚未完成，步骤见 [PyPI 发布](publishing.md)。

## 文档更新

- 项目展示名称统一为 ServoPy，加入适配浅色/深色背景的 SVG Logo。
- README 默认展示英文，中文移至 README.zh-CN.md；语言栏独立于文档导航。
- 同步 Panda 录制画面的项目名，重新生成真实仿真的 GIF、视频与对应指标。
- 重写中文 README，增加英文入口和按任务组织的文档首页。
- 补充快速上手、完整控制/IK 示例、参数/API/诊断/CLI 参考与排障。
- 将高级控制长文拆成独立教程，保留原入口。
- 增加 MkDocs Material 本地预览、中文搜索分词、文档一致性检查及 GitHub Actions 工作流。
- 明确 C++17 为最低语言标准，区分安装依赖范围、精确版本锁定与已测试环境。

## 0.3.0 — 2026-09-18

- 增加可选 RuckigSmoothing、连续轨迹采样及保留可行停止的回退策略。
- 增加 C++/Python DifferentialIK、盒约束 QP、零空间姿态与关节居中。
- 增加周期调度、设备回调协议、最新目标邮箱、取消与恢复。
- Panda 支持外部目标、控制记录；提供 JSONL 回放及 JointTrajectory 数值比较。
- 增加 SOLVER_ERROR、SMOOTHING_ERROR、SMOOTHING_FALLBACK 等诊断信息。

### 从 0.2.0 升级

重新安装本项目以更新 C++ 绑定。现有构造和默认恒加速度行为保持；高级后端均需显式启用。Ruckig 用户应改用 `sample_reference(t)` 执行区间，不可用端点 ddq 假定整段恒加速度。开启零空间后，Pose 的 GOAL_REACHED 可与 TRACK 同时出现。

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install --upgrade '.[mujoco,ruckig]'
```

## 0.2.0 — 2026-09-18

- 增加原生 JointPositionCommand、完整关节名称映射与 joint_position_error。
- 增加 PositionIKAdapter，对位置 IK 解做限位、连续性和 FK 残差校验。
- Panda 两种位置执行器模式改用包级关节命令和共享适配器。

### 从旧版 JointJog 包装升级

已有 `q_target` 时直接发送 `JointPositionCommand(q_target, source_stamp_ns)`，无需在 Python 手写位置误差到 JointJog 的比例律。已有位置 IK 时使用 PositionIKAdapter；保留源时间戳，失败命令仍交给 Servo 制动。

## 0.1.0 — 初始实现

- 独立 C++/Eigen 内核（最低语言标准为 C++17）、Python 绑定、JointJog/Twist/Pose/Stop。
- DLS、奇异性策略、关节约束、时序检查与故障锁存。
- URDF/串联模型、Pinocchio 后端及理想回放/基准脚本。
- 后续同版本示例更新加入 Panda 动力学、viewer/录制与两种位控路径。

下一步：[功能状态](roadmap.md) · [参数默认值](configuration.md) · [贡献指南](contributing.md)
