# 录制文件格式（recording.json）

仿真录制文件把每个固定仿真步的刚体位姿保存为 JSON，用于回放。根对象：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `version` | int | 当前为 1。 |
| `snapshots` | array | 按时间升序的帧数组。 |

每一帧：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `t` | number | 仿真时间（秒）。 |
| `bodies` | array | 该时刻所有可见刚体。 |

每个刚体：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `name` | string | 显示名（与场景编辑器条目一致）。 |
| `kind` | string | 形状或角色，如 `box`/`sphere`/`cylinder`/`plane`/`chassis`/`wheel`。 |
| `position` | [x, y, z] | 世界坐标（米）。 |
| `orientation` | [w, x, y, z] | 四元数。 |

回放时在相邻两帧之间做位置线性插值和朝向 slerp。
