# SRPlatform

![CI](https://github.com/SRLove-git/SRPlatform/actions/workflows/ci.yml/badge.svg)

SRPlatform 是一个面向工程学习的统一仿真平台：设计电路、搭建小车或无人机、
编写 Lua 控制代码，然后在同一个世界里观察它们如何一起工作。物理、电路、
脚本、传感器/执行器通过机电桥连接，所有状态都可记录成时间序列用于教学。

## 功能状态

- Phase 0-6 已完成：工程骨架、自研刚体物理、电路仿真、可控制小车、
  无人机与传感器、Mod 系统、编辑器与学习体验（场景/电路/脚本编辑、
  观测面板、仿真控制、录制回放、四门示例课程、发布打包）。
- Phase 7 进行中：确定性验证、性能基准与热点优化已完成，跨平台构建、
  文档与清理收尾中。

详细进度见 [进度表](docs/progress.md)。

## 快速开始

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Windows 运行编辑器：

```text
build/debug/bin/srp_window.exe --course rc_car
```

构建依赖、运行方式、发布打包见 [安装与快速开始](docs/getting-started.md)。

## 文档

- [总体架构](docs/architecture.md)
- [安装与快速开始](docs/getting-started.md)
- [Mod 开发指南](docs/mod-development.md)
- [性能基准](docs/benchmarks.md)
- [分阶段路线图](docs/roadmap.md)
- [项目规范](docs/project-specification.md)
- [进度表](docs/progress.md)

## 设计要点

- 统一使用 SI 单位与 `double`。
- 固定步长仿真，渲染与仿真解耦，结果可确定性回放。
- 仿真核心（物理、电路、桥、脚本）与展示层（渲染、编辑器）分离。
- 一切皆可观测：电压、电流、受力、能耗、传感器值都可记录成波形。
- Mod 优先：车辆、电路、元件、脚本都是可加载资源，而不是硬编码进引擎。
