# 课程文件格式（course.json）

每门示例课程是 `assets/courses/*.json` 下的一个 JSON 对象：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `id` | string | 必填，唯一标识。 |
| `title` | string | 课程标题。 |
| `description` | string | 课程简介。 |
| `entity` | string | `car` / `drone` / `arm`。 |
| `script` | string | Lua 控制脚本路径。 |
| `objective` | string | 教学目标。 |
| `initial_bodies` | array | 初始场景物体：`{kind, position:[x,y,z]}`。 |
| `channels` | string[] | 建议观测的通道名。 |

脚本可通过受限 API `read_sensor` / `set_motor` / `set_servo` 控制实体。
循迹小车课程额外注入 id=11（左侧距离）与 id=12（右侧距离）传感器。
