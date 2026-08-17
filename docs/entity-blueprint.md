# 实体蓝图格式

实体蓝图用 JSON 描述一个可实例化的车辆或机器人，由
`parseEntityBlueprint()` 解析、`createEntity()` 实例化。

## 基本结构

```json
{
  "id": "my_rc_car",
  "kind": "car",
  "parameters": {
    "chassis_mass": 1.2,
    "wheel_radius": 0.25
  }
}
```

- `id`：必填，非空字符串。
- `kind`：必填，当前支持 `car` 和 `drone`。
- `parameters`：可选，覆盖对应实体的默认参数。

## car 参数

顶层：`wheel_radius`、`chassis_mass`、`wheel_mass`、
`rolling_resistance_torque_nm`、`viscous_load_nm_per_rad_s`；
嵌套对象 `battery`（`full_charge_voltage_v`、`empty_charge_voltage_v`、
`internal_resistance_ohm`、`capacity_coulombs`、`initial_state_of_charge`）
和 `motor`（`armature_resistance_ohm`、`armature_inductance_h`、
`torque_constant_nm_per_a`、`back_emf_constant_v_per_rad_s`、
`rotor_inertia_kg_m2`、`viscous_friction_nm_per_rad_s`）。

## drone 参数

顶层：`chassis_mass`、`max_rotor_angular_velocity_rad_s`；
嵌套对象 `quadcopter`（`arm_length_m`、`propeller`：
`diameter_m`、`thrust_coefficient`、`torque_coefficient`、
`air_density_kg_m3`）。

未写出的参数使用对应实体默认值。
