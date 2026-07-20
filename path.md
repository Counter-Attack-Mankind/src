2026-07-13：将单车/多车任务路径接入 A1-cycle 模式：新增 `/forklift_planner/multi_vehicle/use_a1_cycle` 开关，打开后普通 B->B 任务在 `TaskAllocator` 内部拆成 `B->A1` 与 `A1->B` 两段，分别使用显式 `B_TO_A1`、`A1_TO_B` 生成器并拼接成完整 track；`single_vehicle_patrol.launch` 默认改为单车 `B17->A1->B24` 实验入口，后续可通过扩展 start/target 数组推广到多车。

2026-07-13：将 A1-cycle 的运行时生成改为两张腿路径字典：启动/首次使用时只生成一次 `B->A1[id]` 和 `A1->B[id]`，普通任务 `B_i->B_j` 直接查 `B_i->A1` 与 `A1->B_j` 后拼接；单车 launch 默认关闭 `precompute_task_filter`，避免枚举全部 B->B pair，后续若需要多车全量筛查可用参数重新打开。

2026-07-13：为 A1-cycle 路径缓存增加磁盘目录机制：启动时优先读取 `a1_cycle_path_catalog.yaml` 并填充 `B->A1`、`A1->B` 两张内存字典；若文件不存在、格式版本不匹配、slot 数或 A1 虚拟点不匹配、leg 缺失，则回退为现场生成，生成完成后按 `save_a1_cycle_catalog` 开关保存，下一次启动可直接加载以减少等待。
2026-07-20：针对 `A1->B` 第 7 行内侧库位 `B57..B64`，末端进库不再使用 terminal lane shift 拼接，保留走廊第一车道上的横向到位点，再由普通转弯模型进入库位，避免因末端直线段过短导致车身斜着进库；同时将 A1-cycle catalog 格式号升到 v2，使旧离线路径自动失效并重新生成保存。
2026-07-20：继续修正 `A1->B` 第 7 行中间内侧库位 `B59..B62`，在 `3->4` 走廊过渡处显式构造与外侧一致的 staged parallel-shift 骨架，固定先接到第四走廊第一车道，再进行末端入库；catalog 格式号升到 v3，确保旧缓存路径自动重建。
2026-07-20：将 `A1->B` 的 clothoid 失败处理从全局 `build_arc_path` 改为局部 arc fallback：已接受的 lane shift 与 clothoid 保留，只在失败角点插入 `local_arc_fallback`，避免 row7 中间入库末端失败覆盖整条路径；catalog 格式号升到 v4。
2026-07-20：为 `A1->B` 第 7 行 `B59..B62` 的末端入库增加横向辅助裕度：`3->4` 过渡时根据目标库位中心动态选择辅助竖线 x，使进入库位前的第一车道横向段至少约 0.78m，缓解 S6->S7 过短导致 clothoid 不可行；catalog 格式号升到 v5。
2026-07-20：按实车入库意图重做 `A1->B` 第 7 行 `B59..B62`：撤销动态移动 `3/4` 竖线的做法，恢复先进入第四走廊第一车道，再复用 `center_detour` 机制前进到辅助点、倒车拉开距离、最后正向转弯入库；catalog 格式号升到 v6。
2026-07-20：继续调整 `A1->B` 第 7 行 `B59..B62` 的 center detour，增大第一车道辅助点与 `S6` 的横向距离，避免 `S6->S7` 只有三十多厘米导致局部转弯过急；同时加大倒车 stage 裕度，catalog 格式号升到 v7。
