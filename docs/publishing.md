# PyPI 发布

发行包名为 **`servo-py`**，Python 导入名为 **`servo_py`**，GitHub 仓库名为 **`servopy`**。当前版本 `0.3.0` 的发布流程已加入仓库；首次上传尚未完成。构建成功不表示已经公开发布。

## 发行范围

| 项目 | 首批范围 |
|---|---|
| Python | 常规 CPython 3.10、3.11、3.12、3.13、3.14 |
| CPU | Linux x86_64、aarch64（ARM64） |
| 二进制基线 | `manylinux_2_28`，glibc 2.28 或更新 |
| 产物 | 10 个 wheel、1 个 sdist |
| Ubuntu | 22.04 / Python 3.10、24.04 / Python 3.12 的 x86_64 安装检查；24.04 / Python 3.12 的 ARM64 安装检查 |
| wheel 内容 | C++ 扩展、Python API、示例、小型 URDF、Panda 来源清单和许可证；模型压缩包不包含在 wheel 或 sdist 中 |

Ubuntu 20.04 的 glibc 满足二进制基线，但默认 Python 3.8 不满足项目的 Python 要求，需另外准备 Python 3.10 或更新环境。其他 glibc / Python 组合是否可用，还取决于运行依赖；不将所有 Ubuntu 版本或其他 Linux 发行版写为已验证。

基础包运行时只依赖 NumPy。Ruckig 0.19.4 的上游 Linux wheel 目前仅覆盖 x86_64；ARM64 上安装 `servo-py[ruckig]` 可能触发 Ruckig 源码构建，`--only-binary=:all:` 会直接报无匹配版本。MuJoCo viewer / 视频录制另需桌面或可用的图形后端。Windows、macOS、32 位、PyPy、free-threaded Python 不在本次 wheel 矩阵内。

## 构建与检查

[Python distributions 工作流](https://github.com/OpenGHz/servopy/actions/workflows/release.yml) 在 pull request、main 推送、手动运行和 GitHub Release 发布时执行：

当前[验证运行](https://github.com/OpenGHz/servopy/actions/runs/35327133523)已通过，生成 10 个不含 Panda 模型的 wheel 和 1 个 sdist。wheel 约为 0.38–0.42 MB，测试条件及模型下载验证见[验证记录](validation.md#panda-model-download-and-smaller-distributions)。这次 main 构建没有向 TestPyPI 或 PyPI 上传。

1. 检查 pyproject、CMake 和 C++ 绑定中的版本一致，构建 sdist 并检查 PyPI 元数据。
2. 在原生 x86_64 / ARM64 runner 的 manylinux_2_28 容器内，从该 sdist 构建所有 wheel，并由 auditwheel 检查、修复依赖。
3. 在各 Python 版本安装生成的 wheel，运行基础测试；x86_64 还运行 Ruckig 回归测试。
4. 在上述 Ubuntu 环境强制只安装二进制包，运行 MuJoCo / Pinocchio 测试，x86_64 加入 Ruckig。另在临时空目录运行包内示例和 `servo-py-panda --headless`，验证模型按需下载、校验、缓存及断网后复用。基础示例和 `--help` 不应下载模型。
5. 确认全部 10 个 wheel 和 sdist 齐全、版本和平台标签正确、许可证与来源清单存在、Panda 压缩包已排除，生成 `publish-distributions` artifact。

main 推送和普通手动构建只生成产物。**手动选择 `testpypi`** 才上传测试站；**发布 GitHub Release** 才上传正式 PyPI。两种上传都依赖所有检查成功，且仅发布作业有 `id-token: write` 权限。

## 一次性账号配置

TestPyPI 与 PyPI 是两个独立服务，需要分别登录并完成各自的账号验证 / 双因素认证。在 GitHub 仓库 **Settings → Environments** 建立 `testpypi` 和 `pypi` 两个 environment；可以为正式 `pypi` 环境设置 required reviewers，把审核保留为最后一步。

打开 [TestPyPI publishing](https://test.pypi.org/manage/account/publishing/) 和 [PyPI publishing](https://pypi.org/manage/account/publishing/)，分别增加 pending publisher：

| 字段 | TestPyPI | PyPI |
|---|---|---|
| PyPI Project Name | `servo-py` | `servo-py` |
| Owner | `OpenGHz` | `OpenGHz` |
| Repository name | `servopy` | `servopy` |
| Workflow name | `release.yml` | `release.yml` |
| Environment name | `testpypi` | `pypi` |

Workflow name 是文件名 `release.yml`，不是界面上显示的 `Python distributions`，也不是完整路径。若项目已由当前账号创建，应在该项目 Publishing 设置里添加 publisher。若名称被其他人占用，先处理名称问题，不要重试上传。

Trusted Publishing 使用 GitHub OIDC，不需要把 PyPI token 放进 GitHub Secrets 或发送给协作者。pending publisher 也不会预留包名，首次成功上传才会创建项目。参见 [PyPI 官方说明](https://docs.pypi.org/trusted-publishers/creating-a-project-through-oidc/)。

## 首次测试发布

1. 在 **Actions → Python distributions → Run workflow** 选择 `main`，把 `destination` 设为 `testpypi`。
2. 等全部构建、安装验证和 `testpypi` 作业成功，打开 [TestPyPI 项目](https://test.pypi.org/project/servo-py/0.3.0/) 确认文件。
3. 在匹配平台的新虚拟环境运行：

```bash
python3 -m venv .venv-testpypi
source .venv-testpypi/bin/activate
python -m pip install --upgrade pip
python -m pip install --only-binary=:all: 'numpy>=1.23'
python -m pip install --only-binary=:all: --no-deps \
  --index-url https://test.pypi.org/simple/ 'servo-py==0.3.0'
python -m servo_py.examples.track_pose
python -c "import servo_py; print(servo_py.__version__)"
```

依赖从正式 PyPI 安装，仅本项目使用 TestPyPI 索引；不把两个索引混合查找依赖。再体验 Panda：

```bash
python -m pip install --only-binary=:all: 'mujoco>=3.2,<4' 'imageio>=2.34' 'imageio-ffmpeg>=0.4.9'
servo-py-panda --headless
```

若相同版本已经上传，不能覆盖既有发行文件；代码有变更时同步调整 pyproject.toml、CMakeLists.txt 与 cpp/bindings.cpp 中的版本，再重新测试。工作流不启用 `skip-existing`，避免把旧文件与新构建混为一次成功发布。

## 正式发布

确认 TestPyPI 安装通过、正式 PyPI publisher 已配置后，在 GitHub **Releases → Draft a new release**：

1. 从通过验证的 main 提交创建 tag `v0.3.0`。
2. 填写版本说明，然后 **Publish release**。仅创建草稿或推送 tag 不会上传 PyPI。
3. 等工作流全部检查与 `pypi` 作业成功，查看 [PyPI 项目](https://pypi.org/project/servo-py/0.3.0/)。

随后 Ubuntu 用户即可在虚拟环境执行：

```bash
python -m pip install --only-binary=:all: 'servo-py==0.3.0'
python -m servo_py.examples.track_pose
python -m pip install --only-binary=:all: 'servo-py[mujoco]==0.3.0'
servo-py-panda --headless
```

最后更新 README 和快速上手中的“首次上传待完成”提示，并记录实际发布链接与验证运行。后续版本重复 TestPyPI → GitHub Release 流程。

## 本地排查

```bash
python -m pip install build twine
python scripts/check_release.py
CMAKE_BUILD_PARALLEL_LEVEL=2 python -m build
python -m twine check --strict dist/*
```

本机 `python -m build` 生成的 Linux wheel 只用于本机检查；不能通过改文件名得到 manylinux 兼容性。正式 Linux 产物来自上述容器流水线。若需要本地复现完整构建，安装 Docker 后按 [cibuildwheel 文档](https://cibuildwheel.pypa.io/en/stable/options/) 使用仓库中的配置。
