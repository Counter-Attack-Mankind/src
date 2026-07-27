2026-07-24：将 A1-cycle 从“预先选定目标 B 并拼接 B->A1->B 单条 track”改为真实两航段运输状态机：车辆从 B 空载执行独立 B->A1 航段，到 A1 后进入取货驻留，取货完成才选择/接收卸货 B 并安装独立 A1->B 航段；到 B 后保持满载完成卸货驻留，再置空载进入下一轮。A1 不写入 B 库位 ID，增加独占预约并在车辆离开 A1 0.30m 后释放；新航段会立即触发滚动轨迹刷新。

2026-07-13：将单车/多车任务路径接入 A1-cycle 模式：新增 `/forklift_planner/multi_vehicle/use_a1_cycle` 开关，打开后普通 B->B 任务在 `TaskAllocator` 内部拆成 `B->A1` 与 `A1->B` 两段，分别使用显式 `B_TO_A1`、`A1_TO_B` 生成器并拼接成完整 track；`single_vehicle_patrol.launch` 默认改为单车 `B17->A1->B24` 实验入口，后续可通过扩展 start/target 数组推广到多车。

2026-07-13：将 A1-cycle 的运行时生成改为两张腿路径字典：启动/首次使用时只生成一次 `B->A1[id]` 和 `A1->B[id]`，普通任务 `B_i->B_j` 直接查 `B_i->A1` 与 `A1->B_j` 后拼接；单车 launch 默认关闭 `precompute_task_filter`，避免枚举全部 B->B pair，后续若需要多车全量筛查可用参数重新打开。

2026-07-13：为 A1-cycle 路径缓存增加磁盘目录机制：启动时优先读取 `a1_cycle_path_catalog.yaml` 并填充 `B->A1`、`A1->B` 两张内存字典；若文件不存在、格式版本不匹配、slot 数或 A1 虚拟点不匹配、leg 缺失，则回退为现场生成，生成完成后按 `save_a1_cycle_catalog` 开关保存，下一次启动可直接加载以减少等待。
2026-07-20：针对 `A1->B` 第 7 行内侧库位 `B57..B64`，末端进库不再使用 terminal lane shift 拼接，保留走廊第一车道上的横向到位点，再由普通转弯模型进入库位，避免因末端直线段过短导致车身斜着进库；同时将 A1-cycle catalog 格式号升到 v2，使旧离线路径自动失效并重新生成保存。
2026-07-20：继续修正 `A1->B` 第 7 行中间内侧库位 `B59..B62`，在 `3->4` 走廊过渡处显式构造与外侧一致的 staged parallel-shift 骨架，固定先接到第四走廊第一车道，再进行末端入库；catalog 格式号升到 v3，确保旧缓存路径自动重建。
2026-07-20：将 `A1->B` 的 clothoid 失败处理从全局 `build_arc_path` 改为局部 arc fallback：已接受的 lane shift 与 clothoid 保留，只在失败角点插入 `local_arc_fallback`，避免 row7 中间入库末端失败覆盖整条路径；catalog 格式号升到 v4。
2026-07-20：为 `A1->B` 第 7 行 `B59..B62` 的末端入库增加横向辅助裕度：`3->4` 过渡时根据目标库位中心动态选择辅助竖线 x，使进入库位前的第一车道横向段至少约 0.78m，缓解 S6->S7 过短导致 clothoid 不可行；catalog 格式号升到 v5。
2026-07-20：按实车入库意图重做 `A1->B` 第 7 行 `B59..B62`：撤销动态移动 `3/4` 竖线的做法，恢复先进入第四走廊第一车道，再复用 `center_detour` 机制前进到辅助点、倒车拉开距离、最后正向转弯入库；catalog 格式号升到 v6。
2026-07-20：继续调整 `A1->B` 第 7 行 `B59..B62` 的 center detour，增大第一车道辅助点与 `S6` 的横向距离，避免 `S6->S7` 只有三十多厘米导致局部转弯过急；同时加大倒车 stage 裕度，catalog 格式号升到 v7。
2026-07-20: 针对 `A1->B` 第 3 行库位，末端进库不再使用 terminal lane shift；左侧库位如 B28 改为先向左倒车并走 1/2 右边内侧竖直通道，右侧库位如 B30 改为先向右倒车并走 1/2 左边内侧竖直通道，再由普通转弯进入库位；catalog 格式号升到 v8，确保旧离线路径失效重建。
2026-07-20: 任务指派阶段允许使用路径生成器确认可行的局部 `arc_fallback/G1` 路径，将 `skip_arc_fallback_paths` 改为 `false`；这样 132 条 A1-cycle 离线路径不会因为 `arc=1` 在调度评分第一轮被跳过，后续行轮换不会被无 fallback 偏好绑死。
2026-07-20: 任务指派回到最朴素的 66 个库位随机分配：`chooseNextTarget` 只过滤当前不可用路径、无出库能力目标和被其他车占用/预约的库位，然后用 `random_seed + vehicle.id + task_count + target` 的确定性随机数选目标；删除 row0/row7 偏置、跨行距离、访问次数和最近行记忆等综合评分对目标选择的影响。
2026-07-20: 将朴素随机任务指派从 `deterministicJitter` 伪随机改为运行时随机：`chooseNextTarget` 先收集所有当前可用候选库位，再用 `std::random_device` 均匀抽取一个目标；因此不再受 `random_seed` 固定复现影响。
2026-07-20: 为单车/多车巡逻增加已访问库位可视化：车辆到达目标并进入 `DWELL` 时记录该库位，`MarkerPublisher` 在 `/forklift_planner/markers` 上叠加黄色 `visited_slots` 方块覆盖原绿色库位；单车时即表现为跑完一个库位后该库位由绿色变黄。
2026-07-20: 在随机任务指派前增加候选库位日志：`chooseNextTarget` 完成路径可用、出库能力和占用预约筛查后，会用星号分隔打印当前车辆可随机选择的库位列表，便于检查随机池是否符合预期。
2026-07-20: 扩展随机任务指派候选日志：`chooseNextTarget` 不再只打印可行候选，也会列出 66 个库位中被排除的库位及原因，包括 `same_slot`、路径验证失败原因、`arc_fallback_filtered`、`no_valid_outbound` 和 `reserved_by_other`，便于定位某一整行为什么不能被随机选中。
2026-07-21: 重做 `A1->B` 第 1 行上侧库位策略，抛弃原来的 row1 辅助车道 `aux_y`；B10/B12/B14 从 A1 先向右倒到 1/2 右边内侧竖直通道，再沿第一走廊第一车道前进转弯入库，B16/B18 镜像为先向左倒到 1/2 左边内侧竖直通道；catalog 格式号升到 v9 触发离线路径重建。
2026-07-21: 继续修正 `A1->B` 第 1 行上侧库位：B10/B12/B14 的倒车终点先落在 1/2 右边内侧竖直通道的上侧车道，再沿竖直通道下到第一走廊第一车道后横向入库；B16/B18 镜像落在左边内侧竖直通道，避免 S0 直接位于第一车道导致缺少竖直调整段；catalog 格式号升到 v10。
2026-07-21: Fix A1->row1 upper-slot exit cusp: keep the requested vertical connector before the first-lane terminal turn, skip terminal lane_shift for this row1 case, and make the reverse prefix approach the forward start along the same vertical line so the gear-change point is a valid vertical cusp instead of a horizontal-to-vertical kink.
2026-07-21: Correct A1->row1 upper-slot skeleton coordinates: S0 is now placed at the center of the 1/2 vertical connector below the row1 lanes, S1/S2 stay on row1 first lane y=4.071, and the A1 reverse bridge reaches S0 with a vertical tangent before the forward S0->S1->S2->slot approach; catalog format bumped to v12.
2026-07-21: Replace the A1->row1 upper-slot reverse bridge with a road-topology reverse prefix: A1 rear point reverses to row1 second lane, follows that lane laterally, then uses a second arc into the 1/2 vertical connector S0; this removes the previous lane-shift-like direct bridge from A1 to S0. Catalog format bumped to v13.
2026-07-21: Apply the same road-topology rule to B->A1 row1 upper slots: reverse out to row1 first lane, then drive along first lane to the 1/2 vertical connector, descend to row1 second lane, and enter A1 from the second lane with the existing terminal turn. Catalog format bumped to v14.
2026-07-21: Refine B->A1 row1 upper-left slots B10/B12/B14 to reuse the A1->B corner set in reverse order: reverse exits to the slot-x first-lane point, drives right to the 1/2 right-inner connector, descends to the second lane, and enters A1 from the second lane with the terminal curve direction chosen from the connector side. Catalog format bumped to v15.
2026-07-21: Rebuild B->A1 row1 upper-slot paths as a full typed road skeleton: S0 is the slot rear-axle point, S1 is the slot-x row1 first-lane point, S2 is the 1/2 inner vertical connector on the first lane, S3 is the connector midpoint, then the path changes direction and continues through the second lane into A1. Catalog format bumped to v16.
2026-07-24: Add an 8-vehicle transparent stress-test setup: `single_vehicle_patrol.launch` now defaults to `vehicle_count=8` and continuous dispatch, while `multi_vehicle::overlaps()` always returns false so vehicle-vehicle conflict zones, following/brake checks, hard guards, and start-clear checks that depend on body overlap treat all forklifts as pass-through.

2026-07-24：移除 A1-cycle 的单车独占准入：不再用 `a1_owner` 将 B→A1 串行化，所有车辆可在初始化时独立领取各自的 B→A1 航段，并在各自完成 A1 取货驻留后独立分配 A1→B 航段。`single_vehicle_patrol.launch` 改为显式传入 8 个起点和 8 个首轮卸货目标；A1-cycle 初始化会校验每个起点确实存在有效 B→A1 路径，避免车辆长期停留在 `NEED_TASK` 而不在 RViz 中显示。

2026-07-25：重新启用车辆 OBB-SAT 重叠检测，使固定路径冲突、跟车、前向净空和硬碰撞保护恢复生效；RViz 将当前参与仲裁的路径冲突弧段显示为浅色区域。同向跟车使用浅蓝色 `conflict_same_direction`，交叉与对向因共用相同的整片互斥区 holder 仲裁而合并为浅橙色 `conflict_crossing_or_opposing`。

2026-07-25：修复 A1 装载车被后续 B→A1 车辆逼近后无法倒退离场的问题。新增仅作用于 A1 最后一段的持久服务权：所有车辆仍可并发执行各自 B→A1 航段，非持有车只在 A1 前约 1.10m 停止线排队；持有权跨越 `TO_A1→PICKUP_DWELL→TO_B` 换轨保持，载货车驶出至少 0.75m 且车尾清除候车末段的精确 OBB 冲突区后才释放。候车日志原因为 `wait_a1_exit_Vx`。

2026-07-27：将 A1 服务过程提升为不可被普通路径仲裁打断的事务。A1 持有车在最后进场及载货离场期间优先于 `commit_owner` 和整片路径外包区的 `committed` 判定，候车不能再凭远处冲突块反向刹停服务车；仅当候车车身真实侵入某个独立 OBB 冲突块时才先让其安全清出。普通路径预约同时绑定车辆 `path_gen`，任一车辆在 A1 换轨后会清除涉及旧路径的预约，避免 B→A1 holder 污染新的 A1→B 航段。
