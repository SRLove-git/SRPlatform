# 性能基准

`srp_bench` 记录刚体物理与电路求解的关键路径性能，输出到
`logs/benchmark.json`（该文件不提交，基线数字记录在本页）。

## 运行

```text
cmake -S . -B build/ninja-debug
cmake --build build/ninja-debug --target srp_bench
./build/ninja-debug/srp_bench
```

场景说明：

- `physics N bodies`：N 个动态盒体/球体从空中落下并与地面碰撞，测量每个固定步（1/60 s）耗时。
- `dc solve N nodes`：N 节点电阻梯形网络的直流工作点求解耗时。
- `transient N nodes`：同一网络的瞬态仿真每步耗时。
- `digital 16 gates`：16 级与非门链的数字仿真每步耗时。

## 基线（2026-08-18，Debug / w64devkit GCC 16.1）

| 场景 | 单位 | ms/操作 | 操作/秒 |
| --- | --- | --- | --- |
| physics 32 bodies | 32 | 0.4631 | 2159.6 |
| physics 64 bodies | 64 | 1.6077 | 622.0 |
| physics 128 bodies | 128 | 6.2833 | 159.2 |
| dc solve 16 nodes | 16 | 0.0412 | 24283.6 |
| dc solve 32 nodes | 32 | 0.1879 | 5323.1 |
| dc solve 64 nodes | 64 | 1.1549 | 865.9 |
| transient 16 nodes | 16 | 0.0367 | 27229.0 |
| transient 32 nodes | 32 | 0.1739 | 5750.4 |
| transient 64 nodes | 64 | 1.0885 | 918.7 |
| digital 16 gates | 16 | 0.0115 | 87001.9 |

物理场景随刚体数量接近二次增长（32→128 刚体，单步耗时约 ×13.6），
主要来自宽相阶段的全部两两检测，是 7.3 优化热点路径的目标。
