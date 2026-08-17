# Mod 清单格式（mod.json）

每个 Mod 包根目录必须包含 `mod.json`，描述 Mod 的元信息、入口脚本和依赖。

## 字段定义

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `id` | string | 是 | 全局唯一 ID。只能包含字母、数字、`.`、`_`、`-`，不能以 `.`、`_`、`-` 开头。建议使用反向域名，例如 `com.example.rc_car`。 |
| `name` | string | 是 | 显示名称，例如 `RC Car Pack`。 |
| `version` | string | 是 | 版本号，建议使用语义化版本，例如 `1.0.0`。 |
| `description` | string | 否 | 简短描述。 |
| `author` | string | 否 | 作者。 |
| `entry` | string | 是 | 入口脚本路径，相对于 Mod 根目录，例如 `scripts/main.lua`。 |
| `requires` | string[] | 否 | 依赖的 Mod ID 列表，例如 `["com.example.core"]`。 |

## 示例

```json
{
  "id": "com.example.rc_car",
  "name": "RC Car Pack",
  "version": "1.0.0",
  "description": "An example RC car mod.",
  "author": "SRPlatform",
  "entry": "scripts/main.lua",
  "requires": []
}
```

解析规则：

- 所有必填字段缺失、为空或类型错误时，清单解析失败。
- `requires` 中每一项都必须是合法的 Mod ID。
- 其他字段目前被忽略，后续阶段会扩展模型、电路、蓝图等资源字段。

## 打包与热加载

Mod 目录结构：

```text
my_mod/
  mod.json
  scripts/
    main.lua
```

- `loadModPackage()` 加载目录中的 `mod.json` 并校验入口脚本存在。
- 入口脚本可以通过 `HotReloadScriptHost` 按文件加载；脚本文件被修改后，
  `pollReloads()` 会检测到变化并自动重新编译，无需重启仿真。
