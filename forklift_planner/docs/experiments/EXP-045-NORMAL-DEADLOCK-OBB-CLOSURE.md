# EXP-045 普通 Deadlock 裸 OBB 二维连通闭包

- 分支/基线：`fix` / `255972b`，包含实验时工作区修改。
- 目的：移除普通 deadlock recovery 对 `PotentialConflictZone` 的依赖，以 0.01 m 裸车身 OBB 参数网格的当前 8 邻域连通分量冻结 retreat/pass 边界。
- 范围：仅普通 deadlock 几何与 RViz retreat target；A1 late-owner、bridge TTC、普通 timed TTC 不变。
- 参数变化：无。继续使用既有 `deadlock_retreat_clearance` 作为 retreat 清空边界后的额外后退余量。
- 构建：`catkin_make -DCMAKE_BUILD_TYPE=Release --pkg forklift_planner`。
- 仿真：双车，seed=2024，240 仿真分钟，`dt=0.1 s`，60 s ACTIVE no-progress watchdog；覆盖并越过约 3 h 15 min。
- 产物：`forklift_planner/logs/EXP-20260908-NORMAL-DEADLOCK-OBB-CLOSURE/`。

## 结果

- 构建：PASS。`multi_vehicle_patrol_node` 与 `deadlock_manager_test` 等目标成功链接；未运行 CTest。
- 60 s watchdog 运行：在 `10737.3 s`（178min57.3s）因 `HARD_GUARD` 提前失败；失败前 V0/V1 最大连续受限动作时间分别为 `49.2 s / 39.5 s`，不是 no-progress 失败。
- 不因首次 hard guard 提前退出的完整运行：完成 `144000/144000 ticks`（240min），V0/V1 最终完成 `180/179` 个任务；累计 3 次 hard guard，因此整体安全回归结论仍为 FAIL。
- 完整运行最大连续受限动作时间：V0 `49.2 s`，V1 `39.5 s`；未出现超过 60 s 的连续 STOP/CREEP/YIELD。
- 3h15min 附近普通 deadlock：
  - `193min43.3s`：`CONFIRMED`，pair V0/V1，path_gen `292/293`。
  - 闭包评估：V0 retreat infeasible；选择 V1 retreat、V0 pass，冻结 `retreat_target_s=0.337479 m`、`pass_clear_s=7.35791 m`、`retreat_distance=0.665449 m`。
  - `193min50.1s`：V1 实际退至约 `0.343 m`，通过真实 corridor 复核后 `RETREAT_DONE/PASS_START`。
  - `193min59.1s`：V0 实际到达 `7.37117 m >= pass_clear_s`，事务 `CLEAR`。
  - 195min 后任务继续推进；运行终点 V0/V1 task count 为 `180/179`，未观察到该 pair 持续失活。
- 结论：本次指定的约 3h15min 持续 deadlock/no-progress 已解除；但不能判定整体回归通过，原因是更早及后续共有 3 次普通行驶 hard collision guard，首个发生在 178min57.3s，属于另一个安全问题。
