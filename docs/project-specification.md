# SRPlatform 项目规范

版本：0.1  
状态：草案  
更新日期：2026-08-16  
适用语言：C++

## 1. 项目目标与边界

### 1.1 目标

SRPlatform 是一个面向工程学习的统一仿真平台，将电路仿真、机械运动、传感器、执行器和用户控制代码放进同一个可运行世界。

核心能力：

- 自研刚体物理引擎，后续可扩展到关节、轮子约束、无人机气动力等。
- 自研基础电路仿真，支持模拟电路和数字逻辑。
- 提供机电转换桥，让电信号驱动电机、舵机、螺旋桨等执行器。
- 允许用户加载模型、电路、元件和脚本，形成类似 Minecraft Mod 的扩展生态。
- 提供波形、传感器读数、受力、能耗等可观测数据。

### 1.2 当前范围

- Windows 桌面应用优先。
- 单机、本地仿真。
- 固定步长仿真与教学回放。
- 内置基础元件、车辆和无人机示例。

### 1.3 非目标

以下内容不进入当前阶段：

- 完整 Minecraft 客户端或地图兼容。
- 流体、布料、软体、破坏等高级物理。
- 大规模多人在线联机。
- 对标专业 SPICE、MATLAB/Simulink 或商业 CAD 的完整能力。

## 2. 技术选型

| 领域 | 选择 | 说明 |
| --- | --- | --- |
| 语言 | C++20 | 不启用 C++ Modules，先保持头文件/源文件结构 |
| 构建系统 | CMake 3.24+ | 跨平台，支持 Visual Studio 和 Ninja |
| 包管理 | vcpkg manifest mode | 依赖在 `vcpkg.json` 中声明 |
| 数学库 | GLM | 向量、矩阵、四元数 |
| ECS | EnTT | 仿真世界实体组件系统 |
| 渲染 | GLFW + OpenGL 4.6 | 渲染层必须通过接口隔离，后续可迁移 bgfx/Vulkan |
| 工具 UI | Dear ImGui | 编辑器、调试面板 |
| 模型导入 | Assimp / tinygltf | GLTF 为第一优先格式 |
| 序列化 | nlohmann/json | 场景、Mod、配置 |
| 脚本 | Lua 5.4 + sol2 | 嵌入式和可热重载 |
| 日志 | spdlog | 统一日志输出 |
| 测试 | GoogleTest | 单元、集成、回归 |
| 格式化 | clang-format | 提交前强制 |
| 静态检查 | clang-tidy | CI 和本地可选 |

## 3. 仓库结构

```text
SRPlatform/
├── CMakeLists.txt
├── vcpkg.json
├── cmake/
├── docs/
├── assets/
│   ├── models/
│   ├── circuits/
│   ├── mods/
│   └── shaders/
├── src/
│   ├── app/
│   ├── core/
│   │   ├── loop/
│   │   ├── ecs/
│   │   ├── math/
│   │   └── time/
│   ├── physics/
│   ├── circuit/
│   ├── bridge/
│   ├── scripting/
│   ├── assets/
│   ├── rendering/
│   ├── editor/
│   └── util/
├── tests/
│   ├── unit/
│   ├── integration/
│   └── benchmarks/
└── tools/
```

模块依赖规则：

- `core` 只依赖 `util` 和数学库，不依赖具体渲染或编辑器。
- `physics` 不依赖 `rendering` 或 `editor`。
- `circuit` 不依赖 `physics`。
- `bridge` 依赖 `physics` 和 `circuit`。
- `scripting` 依赖 `core` 和 `bridge`。
- `rendering` 依赖 `core` 和资源层，但不反向依赖 `physics` 内部实现。
- `app` 负责组装所有模块。

## 4. 编码规范

### 4.1 命名

统一命名空间：`srp`。子模块使用子命名空间，例如 `srp::physics`。

| 元素 | 风格 | 示例 |
| --- | --- | --- |
| 类、结构体、枚举 | PascalCase | `RigidBody`, `ContactSolver` |
| 函数、方法 | camelCase | `stepWorld()`, `addBody()` |
| 局部变量、参数 | snake_case | `delta_time`, `body_id` |
| 成员变量 | snake_case + 尾下划线 | `mass_`, `position_` |
| 常量 | kPascalCase | `kDefaultGravity` |
| 枚举成员 | kPascalCase | `kBox`, `kSphere` |
| 宏 | SRP_ 前缀全大写 | `SRP_ASSERT` |

### 4.2 代码风格

- 缩进：4 个空格，不使用 Tab。
- 头文件使用 `#pragma once`。
- 不在头文件中 `using namespace std`。
- 包含顺序：本文件头、项目内头、第三方库、标准库。
- 公开头文件保持最小包含，能用前置声明就不用完整包含。
- 使用 RAII，禁止裸 `new`/`delete` 拥有对象。
- 优先使用值语义和 `std::unique_ptr`；`std::shared_ptr` 只在真正共享所有权时使用。
- 跨模块引用优先使用稳定 ID，而不是长期持有的裸指针。
- 公开 API 尽量标记 `const`、`noexcept`，适合处使用 `constexpr`。

### 4.3 错误处理

- 构造函数、配置加载、资源加载等不可恢复错误可以抛异常。
- 仿真运行时的可恢复错误使用 `std::expected` 或自定义 `Result<T>`。
- 禁止吞掉异常后继续执行而不记录原因。
- 不依赖异常作为正常控制流。

### 4.4 数值类型

- 仿真核心使用 `double`。
- 渲染层可使用 `float`，在边界处显式转换。
- 禁止在物理、电路求解中使用 `float`。
- 时间、长度、质量等物理量使用明确的单位约定，不使用隐式魔法数字。

## 5. 仿真规范

### 5.1 单位

内部统一使用 SI 单位：

| 物理量 | 单位 |
| --- | --- |
| 长度 | 米 |
| 质量 | 千克 |
| 时间 | 秒 |
| 力 | 牛顿 |
| 扭矩 | 牛·米 |
| 电压 | 伏特 |
| 电流 | 安培 |
| 电阻 | 欧姆 |
| 电容 | 法拉 |
| 电感 | 亨利 |
| 角度 | 弧度 |

### 5.2 时间推进

- 使用固定仿真步长，默认 `dt = 1/240s`。
- 电路仿真允许独立子步长，解决 PWM、开关电源等高频问题。
- 渲染帧率与仿真步长解耦，渲染使用插值。
- 仿真结果应尽量可复现：固定遍历顺序、固定种子、避免依赖无序容器迭代顺序。

### 5.3 模块边界

仿真核心不得包含窗口、OpenGL、ImGui 等渲染相关代码。物理模块不得直接包含电路模块，反之亦然。两者只能通过 `bridge` 交换数据。

## 6. 核心接口草案

接口命名以 `I` 开头，实现类放在对应模块中。

```cpp
class IPhysicsWorld
{
public:
    virtual ~IPhysicsWorld() = default;
    virtual void step(double dt) = 0;
    virtual PhysicsBodyId addBody(const BodyDefinition& def) = 0;
    virtual PhysicsJointId addJoint(const JointDefinition& def) = 0;
    virtual std::vector<ContactPoint> contacts(const PhysicsBodyId& id) const = 0;
};

class ICircuitWorld
{
public:
    virtual ~ICircuitWorld() = default;
    virtual void step(double dt) = 0;
    virtual ComponentId addComponent(const ComponentDefinition& def) = 0;
    virtual void connect(ComponentId a, PortId a_port, ComponentId b, PortId b_port) = 0;
    virtual double nodeVoltage(NodeId id) const = 0;
    virtual double componentCurrent(ComponentId id) const = 0;
};

class IActuatorBus
{
public:
    virtual ~IActuatorBus() = default;
    virtual void setMotor(MotorId id, double value) = 0;
    virtual void setServo(ServoId id, double angle) = 0;
};

class ISensorBus
{
public:
    virtual ~ISensorBus() = default;
    virtual double read(SensorId id) const = 0;
};

class IScriptHost
{
public:
    virtual ~IScriptHost() = default;
    virtual void load(const std::string& id, const std::string& source) = 0;
    virtual void runOnce(double dt) = 0;
    virtual void reload(const std::string& id) = 0;
};
```

这些接口不是最终实现，但它们是后续开发中的稳定边界。

## 7. Mod 与资源格式

### 7.1 主格式

- 3D 模型：GLTF 或 GLB。
- 场景/实体蓝图：JSON。
- 电路网表：JSON，后续兼容 SPICE 子集。
- Mod 清单：JSON。

### 7.2 Mod 包结构

```text
my_robot_mod/
├── mod.json
├── models/
├── circuits/
├── components/
├── scripts/
└── assets/
```

`mod.json` 至少包含：

- 名称与 ID。
- 版本。
- 依赖。
- 实体蓝图列表。
- 电路和元件列表。
- 脚本入口。

## 8. 构建与开发流程

### 8.1 本地构建

Windows 建议使用 Visual Studio 2022 和 vcpkg。

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Debug
ctest --test-dir build -C Debug
```

### 8.2 Git 工作流

- 主分支保持可构建。
- 提交信息使用 Conventional Commits，例如 `feat(physics): add sphere-box collision`。
- 提交前执行格式化和单元测试。
- PR 必须包含测试说明和验证结果。

### 8.3 质量门槛

合并到主分支前应满足：

- 代码通过编译，无新增警告。
- 通过 clang-format 检查。
- 单元测试通过。
- 新增物理或电路能力提供对照测试或可复现验证场景。
- 公共接口变更已更新文档。

## 9. 测试策略

### 9.1 单元测试

覆盖：

- 数学和碰撞算法。
- 约束求解。
- 电路节点分析。
- 电机、舵机等转换模型。
- 脚本 API。

### 9.2 集成测试

覆盖：

- 电池、电机、轮子联动。
- 传感器到脚本再到执行器的闭环。
- Mod 加载和场景创建。

### 9.3 回归与基准

- 保留已知结果的场景，防止物理和电路数值退化。
- 逐步加入性能基准，记录刚体数量和电路节点数量。

## 10. 文档规范

- 架构变更先改文档，再改代码。
- 公共头文件必须写清楚单位、约束、线程假设。
- 复杂算法在模块内写简短设计说明。
- 后续引入 Doxygen，但当前阶段以 Markdown 为主。

## 11. 当前里程碑

按 [路线图](roadmap.md) 执行，首个可运行里程碑是：

> Phase 0 + Phase 1：C++ 工程骨架、固定步长循环、地面与盒体、重力、碰撞、摩擦。

完成该里程碑后，再进入电路模块和第一个“电池驱动电机驱动轮子”的完整闭环。
