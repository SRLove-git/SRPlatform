# Mod 开发指南

Mod 让用户不需要改引擎源码就能加入自己的车辆、机器人、电路与脚本。
参考示例：`assets/mods/rc_car_demo`。

## 目录结构

```text
my_mod/
├── mod.json            # 必需，Mod 清单
├── blueprint.json      # 可选，实体蓝图（缺省时按此文件名约定）
└── scripts/
    └── main.lua        # 入口脚本（mod.json 的 entry 字段指向）
```

## 1. 清单（mod.json）

字段定义见 [Mod 清单格式](mod-manifest.md)。

```json
{
  "id": "com.example.rc_car",
  "name": "RC Car Pack",
  "version": "1.0.0",
  "description": "An example RC car mod.",
  "entry": "scripts/main.lua",
  "blueprint": "blueprint.json"
}
```

## 2. 实体蓝图（blueprint.json）

蓝图描述要创建什么实体。目前支持 `car` 和 `drone`，字段说明见
[实体蓝图](entity-blueprint.md)。

```json
{
  "id": "temp_car",
  "kind": "car",
  "parameters": {
    "battery": {
      "full_charge_voltage_v": 12.0,
      "empty_charge_voltage_v": 9.0
    }
  }
}
```

## 3. 脚本 API

入口脚本每固定步调用一次 `update(dt)`，可以使用的受限 API：

| 函数 | 说明 |
| --- | --- |
| `read_sensor(id)` | 读取传感器数值；无传感器时返回 NaN。 |
| `set_motor(id, value)` | 设置电机指令（-1..1）。 |
| `set_servo(id, angle)` | 设置舵机指令（-1..1，机械臂按比例映射到关节限位）。 |

常用传感器 ID：

- 小车：1=速度、2=位置 x、3=航向。
- 无人机：1=高度、2=垂直速度。
- 机械臂：1..n=各关节角度（弧度）。

## 4. 电路网表

Mod 可附带电路定义，格式见 [电路网表格式](netlist-format.md)；
编辑器“Circuit”面板导出的文件可直接复用。

## 5. 热重载

Mod 加载后，脚本文件被监视；修改保存后下一次仿真步会自动重新编译加载
（`ModEntityDemo` / `HotReloadScriptHost`）。

## 6. 示例课程扩展

课程不是 Mod，但同样通过 JSON 描述，放在 `assets/courses/*.json`，
字段说明见 [课程文件格式](course-format.md)。课程脚本可读取编辑器注入的
外部传感器（例如循迹小车课程的左右距离传感器 id=11/12）。
