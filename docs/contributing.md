# 贡献与文档维护

建议先用最小模型或现有示例复现问题，再修改对应层。控制行为改变需要说明对时序、限位、参考采样与停止行为的影响；文档修改需要保证示例与当前接口一致。

## 开发环境

```bash
python -m venv .venv
source .venv/bin/activate
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m pip install -e '.[test,docs,ruckig]'
```

需要完整可选测试时安装 `.[test,mujoco,pinocchio,ruckig,docs]`。修改 C++ 或绑定后重新安装，避免误测旧扩展。

## 源码导航

| 路径 | 职责 |
|---|---|
| `include/servo_py/servo.hpp` | C++ 公共类型与扩展接口 |
| `cpp/servo.cpp` / `cpp/solvers.cpp` | 控制、约束、DLS 与 QP |
| `cpp/kinematics.cpp` | 原生串联运动学 |
| `cpp/bindings.cpp` | Python 绑定与回调 |
| `src/servo_py/api.py` | Python 类型与调用入口 |
| `src/servo_py/ik.py` / `smoothing.py` | 位置 IK 适配与可选 jerk 后端 |
| `src/servo_py/runtime.py` / `recording.py` | 周期/设备与记录工具 |
| `tests/` / `examples/` | 行为验证与可运行示例 |
| `docs/` / `scripts/check_docs.py` | 文档及一致性检查 |

## 选择相关验证

| 改动 | 检查 |
|---|---|
| 内核或模型 | `python -m pytest -q`，以及 [独立 C++ 构建](cpp.md) |
| Panda 控制 | `python -m pytest -q tests/test_mujoco_panda.py` |
| 可选 IK/平滑/设备 | 对应 tests 模块，并安装所需 extras |
| 教程、导航、参数说明 | 以下文档验证流程 |
| 发布/打包元数据 | 构建 wheel/sdist，确认源文件与许可证包含情况 |

测试应检查有意义的行为、数值基准或失败路径；避免只重复实现细节。历史性能或仿真结果不要直接改写成新结果，补充新的条件和记录。

## 本地预览与文档验证

```bash
python scripts/check_docs.py
python scripts/check_docs.py --run
python -m mkdocs build --strict
python -m mkdocs serve
```

打开 serve 输出的本地地址，检查导航、搜索与代码复制。静态输出在 `site/`，不提交构建产物。本仓库只配置本地构建与文档检查，没有声明一个已经部署的公开文档站。

检查脚本会核对 Markdown 本地路径/锚点、公共 API、配置默认值、flags 与 CLI 参数，并可执行明确标注的完整 Python 示例。它不启动真实设备、不自动执行文档中的任意 shell 命令，也不把接入片段当作完整程序。

[Documentation 工作流](https://github.com/OpenGHz/servopy/blob/main/.github/workflows/docs.yml) 在 PR 和 main 推送时执行示例检查与严格构建，使用 Linux / Python 3.12；也可在 Actions 手动触发。该工作流不发布站点。

## 文档写法

1. README 负责项目定位、第一条可运行命令和任务入口。参数细节放参考页。
2. 教程说明目标、前置环境、完整操作、预期结果与下一步。需要外部对象的代码明确标为接入片段。
3. 默认值只在配置参考集中维护；新增公共名称时同步 API 和诊断页。
4. 完整可运行 Python 块前加 `<!-- runnable: unique-name -->`，用检查脚本执行。示例默认从仓库根目录运行。
5. 保持现有页面入口可访问；需要拆分时保留导览，避免旧链接失效。
6. 写清观测环境、参考/反馈、仿真/实机区别；不增加未经验证的性能、兼容性、发布或 CI 状态徽章。

## 文档结构参考

本轮结构整理参考了 [Ruckig](https://github.com/pantor/ruckig) 的入门示例、[Pink](https://github.com/pink-kinematics/pink) 的任务与示例入口、[Pinocchio](https://github.com/stack-of-tasks/pinocchio) 的学习资源导航，以及 [MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html) 的概念与接口分层。内容按 [Diátaxis](https://diataxis.fr/) 的学习、操作、参考、解释四种需求组织；技术说明与代码均以本仓库实际实现为准。

## 提交变更

问题报告应包含最小复现、环境、期望与实际结果。PR 描述先说明问题与行为变化，再列相关验证及未覆盖范围。文档与代码更新应在同一变更中完成；不要把未执行的验证写成已通过。

项目许可证与上游资产来源保留在仓库根目录的 LICENSE、NOTICE 和 LICENSES 中。新增依赖或模型资产时一并说明来源与许可证。
