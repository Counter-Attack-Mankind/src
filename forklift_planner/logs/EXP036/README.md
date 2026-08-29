# EXP-036：A1 waiter 裸车身边界前置停车余量

- 分支：`fix`
- 基线提交：`e448851`
- 日期：2026-08-29
- 修改目的：保持 A1 ConflictZone 的裸车身物理边界不变，仅将 waiter 控制停车点从边界前 0.01 m 调整为前 0.10 m。
- 参数：新增 `forklift_planner/multi_vehicle/a1_stop_margin=0.10 m`。

## 构建与单测

- `catkin_make -DCMAKE_BUILD_TYPE=Release --pkg forklift_planner`：通过。
- `future_a1_policy_test`：通过。

## 30 min 回归

- launch：`multi_vehicle_phase2_batch.launch`
- `random_seed=2025`
- `vehicle_count=2`
- `start_slots=[38,20]`（保持 launch 既有值）
- `simulation_time=30 min`，完成 18000 tick / 1800 s。

结果：

- 9 条 `FUTURE_A1_ADMISSION` 与 5978 条含边界的 `DEPARTURE_CLUSTER` 日志中，`boundary - stop_s` 的最小值、最大值、平均值均为 0.100 m。
- 典型首次 admission：`selected_stop_boundary_s=2.500, stop_s=2.400, other_s=2.320`；对应 protected rollout 中 waiter 最终约为 `other_s=2.401`，距物理边界 0.099 m。
- 全局最接近物理边界且仍在边界上游的 protected rollout 样本：`boundary=0.800, stop_s=0.700, other_s=0.702`，距物理边界 0.098 m。
- 实际记录到 32 次 `TO_A1 -> PICKUP_DWELL` 和 32 次 `PICKUP_DWELL -> TO_B`，V0/V1 最终任务数分别为 15/16。
- A1 service：create=32、hold=699、release=31、invalidate=0。
- `hard_guard_events=1`，首次在 tick 11739 / sim 1173 s。V0 已在 A1 路径终点进入 DWELL；V1 以 NOMINAL/CLEAR 从 s=1.660 接近，随后触发 hard collision guard。该失败不是日志中的正常 waiter 在 `stop_s` 附近停车案例。
- 结尾 V0/V1 wait 分别约 624.3/626.2 s，deadlock ticks=1198，存在永久停滞。

## 结论

0.10 m 控制停车余量已按要求生效，但 30 min 验收因一次 hard guard 和后续永久停滞失败。本实验不扩展 A1 owner、reservation 或 DepartureCluster 修复。
