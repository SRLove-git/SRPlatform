# 贡献规范

感谢你考虑为 SRPlatform 贡献代码、文档、模型、电路或示例场景。

本项目目前处于早期阶段，优先保持架构清晰和可运行性，其次才是功能数量。

## 1. 开始之前

在提交贡献前，请先阅读：

- [README.md](README.md)
- [总体架构](docs/architecture.md)
- [分阶段路线图](docs/roadmap.md)
- [项目规范](docs/project-specification.md)

如果你计划做较大改动，建议先创建 Issue 说明动机和方案，避免和其他工作冲突。

## 2. 行为准则

- 尊重不同背景和经验的贡献者。
- 讨论技术问题时聚焦证据和可复现结果。
- 不提交恶意代码、不加入不必要的远程请求或数据收集。
- 遇到分歧时以项目文档和可验证行为为准。

## 3. Issue 规范

提交 Issue 时尽量包含：

- 操作系统和构建环境。
- C++ 编译器和 CMake 版本。
- 复现步骤。
- 期望行为和实际行为。
- 相关日志、截图或最小复现场景。

功能建议需要说明：

- 它解决什么问题。
- 预期使用者是谁。
- 大致涉及哪些模块。
- 是否影响 Mod、脚本或公共 API。

## 4. 开发环境

项目使用：

- C++20
- CMake 3.24+
- vcpkg
- Visual Studio 2022 或兼容编译器

具体构建命令和依赖说明以 [项目规范](docs/project-specification.md) 为准。

## 5. 分支与提交

### 5.1 分支

建议从最新主分支创建短期分支：

```text
feature/描述
fix/描述
docs/描述
refactor/描述
```

### 5.2 提交信息

使用 Conventional Commits：

```text
feat(physics): add sphere-box collision
fix(circuit): correct capacitor transient step
docs(readme): update C++ build requirements
refactor(bridge): split motor and sensor buses
test(physics): add friction regression scene
```

每个提交只做一件可解释的事。

## 6. 代码要求

- 遵循 [项目规范](docs/project-specification.md) 中的命名和风格要求。
- 仿真核心使用 `double`，不使用 `float`。
- 物理、电路、渲染之间保持模块边界，不在仿真核心中引入渲染代码。
- 新增公共接口必须写清楚单位、线程假设和生命周期。
- 提交前运行 clang-format 和本地测试。

## 7. 测试要求

- 修复 Bug 时优先添加能复现该 Bug 的测试。
- 新增物理或电路能力时，提供已知结果或数值对照。
- 修改求解器时，说明对确定性或精度的影响。
- 测试应能在本地使用 CTest 运行。

## 8. Pull Request 流程

提交 PR 前确认：

- 分支已同步主分支。
- 代码已格式化。
- 本地测试通过。
- 文档已同步更新。
- PR 描述写清改动原因、影响范围和验证方式。

PR 合并门槛：

- 通过构建和测试。
- 无明显架构边界破坏。
- 至少经过一次维护者审查。
- 公共 API 或 Mod 格式变更已明确记录。

## 9. 文档与资源贡献

- 文档使用 Markdown，保持中文为主，术语可保留英文。
- 模型优先使用 GLTF。
- 电路、Mod、场景使用 JSON，并记录字段含义。
- 示例资源应说明用途、单位约定和来源。

## 10. 兼容性与版本

- 不要为了短期功能破坏现有 Mod API，除非有明确迁移说明。
- 资源格式变更需要版本号或兼容说明。
- 保持主分支可构建，避免提交半成品状态。
