# 安装与快速开始

## 环境要求

- CMake 3.24+
- C++20 编译器（GCC 12+ / Clang 16+ / MSVC 2022）
- Ninja（可选，推荐）

Windows 下可直接使用 [w64devkit](https://github.com/skeeto/w64devkit)
（自带 GCC、CMake、Ninja），或用 MSYS2 MinGW64。Linux 需要 g++ 与 ninja：

```bash
sudo apt-get install -y ninja-build cmake g++ git
```

依赖（spdlog、nlohmann/json、glm、Lua、sol2、GoogleTest、Dear ImGui）
由 CMake FetchContent 自动下载，首次配置需要联网。

## 构建

```bash
cmake --preset debug
cmake --build --preset debug
```

也可以手动指定目录：

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

Linux 只构建无头核心（库、`srp_cli`、`srp_bench`、单元测试）；
`srp_window` 编辑器窗口目前仅支持 Windows。

## 运行

### 编辑器窗口（Windows）

```text
build/debug/bin/srp_window.exe
```

可选参数：

```text
srp_window.exe --course quadcopter
```

`--course` 可选：`rc_car`、`line_follower`、`quadcopter`、`robotic_arm`。

操作：

- 左键点击 3D 视口：选中物体；Move mode 下拖动放置到地面。
- 右键拖动：环绕视角；滚轮：缩放。
- Delete：删除选中物体；Esc：取消选中/取消连线。
- 空格：暂停/继续仿真；右方向键：单步。
- Scene / Circuit / Script / Observations / Recording / Courses / Simulation
  面板分别提供场景编辑、电路编辑、脚本编辑、观测、录制回放、课程加载与仿真控制。

### 命令行工具

```text
srp_cli
```

打印版本信息。

### 单元测试

```bash
ctest --preset debug
```

### 性能基准

```bash
build/debug/srp_bench
```

结果写入 `logs/benchmark.json`，基线见 [docs/benchmarks.md](benchmarks.md)。

## 发布打包（Windows）

```powershell
./scripts/package.ps1
```

生成 `dist/SRPlatform-<version>-win64.zip`，解压后运行 `bin\srp_window.exe`
即可，无需安装依赖。

## 学习资源

- [Mod 开发指南](mod-development.md)
- [示例课程](../assets/courses/README.md)（RC 小车、循迹车、四旋翼、机械臂）
- [总体架构](architecture.md)
- [路线图](roadmap.md)
