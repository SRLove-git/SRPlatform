# 电路网表格式（netlist.json）

外部 JSON 电路网表可加载为 `CircuitModel`。根对象包含 `nodes`（可选）和 `components`（必填）。

## 示例：12V 分压电路

```json
{
  "nodes": ["vcc", "out"],
  "components": [
    {
      "type": "voltage_source",
      "name": "V1",
      "ports": ["vcc", "gnd"],
      "parameters": { "voltage": 12.0 }
    },
    {
      "type": "resistor",
      "name": "R1",
      "ports": ["vcc", "out"],
      "parameters": { "resistance": 1000.0 }
    },
    {
      "type": "resistor",
      "name": "R2",
      "ports": ["out", "gnd"],
      "parameters": { "resistance": 2000.0 }
    }
  ]
}
```

## 节点

- `"gnd"`、`"ground"`、`"0"` 都指向内置地节点。
- `nodes` 数组用于提前声明节点；未声明但被元件引用的节点会自动创建。
- 节点名不能重复（地节点别名除外）。

## 元件

每个元件对象：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `type` | string | 必填，元件类型（见下表）。 |
| `name` | string | 可选，显示名。 |
| `ports` | string[] | 必填，各端口连接的节点名，数量必须与类型匹配。 |
| `parameters` | object | 可选，元件参数。 |

端口顺序：

- 两端口元件（电阻/电容/电感/二极管/开关）：`[terminal_a, terminal_b]`
- 电压源/电流源：`[positive, negative]`
- 数字源/PWM 源：`[output]`
- 逻辑门：`[input_a, input_b, output]`
- D 触发器：`[d, clock, q, qbar]`

### 元件类型与参数

| type | 参数 |
| --- | --- |
| `resistor` | `resistance`（Ω） |
| `capacitor` | `capacitance`（F）、`initial_voltage`（V） |
| `inductor` | `inductance`（H）、`initial_current`（A） |
| `voltage_source` | `voltage`（V） |
| `current_source` | `current`（A） |
| `diode` | `forward_voltage`、`on_resistance`、`off_resistance` |
| `switch` | `closed`（bool）、`on_resistance`、`off_resistance` |
| `digital_source` | `initial_value`（bool）、`frequency_hz` |
| `logic_gate` | `type`：`not`/`and`/`or`/`nand`/`nor`/`xor`/`xnor` |
| `d_flip_flop` | `initial_q`（bool） |
| `pwm_source` | `frequency_hz`、`duty_cycle` |

未写出的参数使用默认值。

## 自定义元件

`ComponentLibrary` 允许用户注册自定义元件模板，例如把二极管封装成“白光 LED”：

```cpp
srp::circuit::ComponentDefinition white_led;
white_led.type = srp::circuit::ComponentType::kDiode;
white_led.name = "white_led";
white_led.port_names =
    srp::circuit::defaultPortNames(srp::circuit::ComponentType::kDiode);
srp::circuit::DiodeParameters diode;
diode.forward_voltage = 2.8;
white_led.parameters = diode;
library.registerComponent("white_led", white_led);
```

之后网表里可以直接写 `"type": "white_led"`，其默认参数来自模板；网表中的
`parameters` 可以覆盖模板默认值。重复注册同名元件会被拒绝，模板参数类型与元件
类型不匹配也会被拒绝。
