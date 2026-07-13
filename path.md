2026-07-13：将单车/多车任务路径接入 A1-cycle 模式：新增 `/forklift_planner/multi_vehicle/use_a1_cycle` 开关，打开后普通 B->B 任务在 `TaskAllocator` 内部拆成 `B->A1` 与 `A1->B` 两段，分别使用显式 `B_TO_A1`、`A1_TO_B` 生成器并拼接成完整 track；`single_vehicle_patrol.launch` 默认改为单车 `B17->A1->B24` 实验入口，后续可通过扩展 start/target 数组推广到多车。

2026-07-13：将 A1-cycle 的运行时生成改为两张腿路径字典：启动/首次使用时只生成一次 `B->A1[id]` 和 `A1->B[id]`，普通任务 `B_i->B_j` 直接查 `B_i->A1` 与 `A1->B_j` 后拼接；单车 launch 默认关闭 `precompute_task_filter`，避免枚举全部 B->B pair，后续若需要多车全量筛查可用参数重新打开。
2026-07-13：为 A1-cycle 路径缓存增加磁盘目录机制：启动时优先读取 `a1_cycle_path_catalog.yaml` 并填充 `B->A1`、`A1->B` 两张内存字典；若文件不存在、格式版本不匹配、slot 数或 A1 虚拟点不匹配、leg 缺失，则回退为现场生成，生成完成后按 `save_a1_cycle_catalog` 开关保存，下一次启动可直接加载以减少等待。
