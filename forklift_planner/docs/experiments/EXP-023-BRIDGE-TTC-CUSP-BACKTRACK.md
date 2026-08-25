# EXP-023：bridge TTC 换向点回溯语义

## 1. 基线、范围与结论

- 日期：2026-08-25
- 基线 HEAD：`fbdee63970a7397f5a828d1f8973acd9e72298ce`
- 工作区起始状态：clean；本轮未创建 Git commit。
- 场景：`random_seed=2025`、`start_slots=[42,50]`、`vehicle_count=2`、9000 ticks / 900.0 s 真实仿真时间。
- 范围：只修改 ordinary bridge-aware TTC correction 的回溯终止、other 最近点局部跟踪、诊断和对应测试。
- 总结论：**问题判断部分成立，定向安全回归 FAIL**。

成立部分：旧实现确实以 self `WpType` 变化直接终止回溯；旧 `NearestCursor` 也把 other 搜索永久限制在 collision seed 所属 traversal。这两个语义均不符合“distance 或 actual-motion direction 丢失才结束 relation”。

不成立部分：B42/B50 的 `boundary_s_b≈0.735~0.737` 虽非常接近换向区，但修改后仍由 `relation_distance_lost` 得到同一 boundary，动作序列和首次 hard guard 均未改变。因此现有证据不支持“该碰撞由 cusp break 单独造成”。

## 2. 源码核查

修改前 `bridge_ttc_correction.cpp::evaluateVehicle()` 保存 `self_seed_type`，且回溯中执行：

```cpp
if (self.track.typeAtS(query_s) != self_seed_type) break;
```

修改前 `NearestCursor` 同时保存 collision seed 的 `traversal_type`、`first_index`、`last_index`，`nearestOnTraversal()` 只能在该连续 traversal 内 hill-climb。因此只删除 self break 会留下 other traversal 锁定问题。

当前生产路径生成器以路径顺序表示 `s` 增大方向，并在 reverse 段保存 `body heading = motion heading + pi`；例如 `path_generator_routes/b_to_a1/path_generator_route.cpp` 的 `append_motion_line()`。执行链也以 `theta + pi` 得到 REVERSE 实际运动方向。故本轮保留既有 `motionHeading()`，避免引入另一套切线插值；但明确取消 `WpType` 作为回溯边界的职责。

## 3. 最小修改

- `bridge_ttc_correction.cpp`
  - 删除 self seed traversal break。
  - `nearestOnTraversal()` 改为 collision-seeded `nearestOnPathLocal()`：cursor 只在相邻路径顺序内以固定 `±2 waypoint/segment` 窗口推进，可跨邻接/重复 cusp，但不做全路径扫描，也不根据 self type 强制 other type。
  - relation 只由 `distance <= vehicle_width + conflict_margin` 且 `direction_dot < bridge_opposing_threshold` 延续。
  - 完整记录 self/other traversal change；终止原因区分 distance、direction、path start、invalid geometry。
- `bridge_ttc_correction.h`：增加终止枚举和诊断字段。
- `rule_engine.cpp`：增强 `[BRIDGE-TTC]`，不改变 priority 或 action 选择。
- `multi_vehicle_config.h`：修正注释，不改参数值。
- `bridge_ttc_correction_test.cpp`：加入三类回归：跨 self cusp 后 relation 持续、跨 cusp 后方向真实丢失、other 最近点独立跨邻接 cusp。

未修改 `priorityWinner()`、`unifiedPriority()`、A1 service/reservation/FutureA1/departure cluster、SlotDepartureAdmission、FAR/MID/NEAR、STOP threshold、rolling period、15 s prediction、hard guard 或 ordinary reservation 原则。

## 4. 修改前后算法差异

| 项目 | 修改前 | 修改后 |
|---|---|---|
| self traversal 变化 | 立即 `break` | 仅计数并用新阶段实际运动 heading 继续判断 |
| other 最近点 | 永久锁在 collision traversal | collision-seeded、路径顺序连续的固定局部窗口可跨邻接 traversal |
| other traversal 选择 | seed type 决定 | 当前 self 点的局部欧氏最近关系决定 |
| relation 终止 | type / distance / direction 混合 | 仅 distance、direction、path start 或 invalid geometry |
| 复杂度 | 局部 hill-climb | 仍为常量局部窗口；未恢复 `N×M` 或 BFS |

## 5. 验证命令与结果

构建与测试：WSL Ubuntu 20.04、ROS Noetic、Release。

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
cd build
ctest --output-on-failure
```

结果：构建成功，8/8 CTest PASS（bridge、conflict closure、FutureA1、spatiotemporal、dynamic coordination、RuleEngine、prediction consistency、rolling timing）。

定向命令：

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=15 vehicle_count:=2 random_seed:=2025 \
  start_slot_a:=42 start_slot_b:=50 \
  coord_log_file:=.../EXP-023-B42-B50-seed2025-15min-coord.log \
  node_output:=log
```

完成 9000/9000 ticks、900.0 s，但 **FAIL**：hard guard 1 次，首次 tick 8948 / sim 894 s；deadlock ticks=0，wedge episodes=0，reciprocal STOP cycles=0；汇总最大 wait=18.0 s（V1）。因为定向测试未通过，按任务顺序未运行 seed 2024/2025/2026 的 120 min 回归。

## 6. 14min42s 附近修改前后对照

修改前和修改后控制结果相同：

| plan | original TTC | collision sB | boundary sB | corrected TTC B | action |
|---:|---:|---:|---:|---:|---|
| 474 | 8.800 | 1.412 | 0.737 | 5.373 | YIELD |
| 475 | 7.200 | 1.362 | 0.737 | 4.025 | CREEP |
| 476 | 6.100 | 1.261 | 0.736 | 3.417 | CREEP |
| 477 | 4.900 | 1.160 | 0.735 | 2.714 | CREEP |
| 478 | 3.700 | 1.060 | 0.735 | 2.011 | STOP |

增强日志显示 plan 474--478 的 `self_traversal_changes_b=0`，失败 query 为 `sB=0.710~0.712`，match distance 为 `0.237~0.239 m`。实际 proximity limit 为 `vehicle_width 0.191 + conflict_margin 0.040 = 0.231 m`，因此均由 `relation_distance_lost` 结束，而不是 type break。

plan 479 从 `collision_s_b=0.876` 回溯到有效 boundary `0.726`，下一 query `0.701` 时 `self_traversal_changes_b=1`，证明新代码实际尝试跨过 traversal；该点 distance=`0.238 m`、direction dot=`+0.620`，两个 relation 条件都已失败。实现按 predicate 顺序报告 `relation_distance_lost`，日志仍保留 dot 供审查。由 0.025 m 采样只能确认 type transition 位于 `(0.701,0.726]`，所以原 `0.735~0.737` 距该区约 0.009--0.036 m，属于“非常接近”，但不是由 traversal 变化决定的 boundary。

## 7. 全部 traversal-change `[BRIDGE-TTC]` 样本

本次 15 min 共 6 条；other traversal change 在真实场景中为 0，独立跨越行为由单元测试覆盖。

| plan | self changes A/B | other changes A/B | end reason A/B | end dot A/B | selected |
|---:|---|---|---|---|---|
| 371 | 0/1 | 0/0 | distance/direction | -0.993/+0.877 | NOMINAL/NOMINAL |
| 372 | 0/1 | 0/0 | distance/direction | -0.993/+0.899 | NOMINAL/YIELD |
| 373 | 0/1 | 0/0 | distance/direction | -0.984/+0.886 | NOMINAL/YIELD |
| 374 | 0/1 | 0/0 | distance/direction | -0.984/+0.877 | NOMINAL/YIELD |
| 375 | 0/1 | 0/0 | distance/direction | -0.984/+0.893 | NOMINAL/CREEP |
| 479 | 0/1 | 0/0 | distance/distance | -0.597/+0.620 | NOMINAL/STOP |

完整原始行见 coordination log 第 12422、12426、12430、12434、12438、16274 行。

## 8. hard guard、deadlock、reservation 与 A1

- hard guard：1；首次 tick 8948 / sim 894 s，V0-V1。
- 首碰前 tick 8939--8947：V0 从 `s=2.804` 前进至 `2.942`；V1 在 `s=0.484` 因 `dynamic_speed_STOP_V0` 停止，wait 从 11.9 s 增至 12.7 s。
- 首碰 geometry：zone A `[0.800,3.575]`、B `[0.150,3.200]`，双方 committed，位置约 `(1.35,2.73)`。
- deadlock：0 ticks；wedge=0；reciprocal STOP cycle=0。
- ordinary reservation：evaluated/executed 均 `ordinary_create=0`。
- A1 指标与修改前完全一致：service `create=23 hold=287 release=23`；launch `allow=22 hold=4 a1_prefix_hold=2 ordinary_road_hold=2 released_after_hold=4 max_hold=8.4`。
- 修改前后首碰时间、任务/路径代次和 14:42 动作均一致，说明本次 cusp 语义修复没有解决该安全失败，也没有证据表明 A1 行为发生变化。

## 9. 第一处失败与证据

第一处失败：`tick=8948`、`sim_t=894s`（14min54s），V0-V1 触发 `hard_collision_guard`：

```text
[FIRST-COLLISION] @tick=8948 sim_t=894s 涉及对=[V0-V1]
V0 ... s=2.942 ... reason=hard_collision_guard
V1 ... s=0.484 ... wait=12.8 ... reason=hard_collision_guard
[CONFLICT] zone0 ... A[0.800,3.575] committedA=1 |
                   B[0.150,3.200] committedB=1 | @(1.35,2.73)
```

证据文件：

- `EXP-023-B42-B50-seed2025-15min-coord.log`：完整 `[BRIDGE-TTC]`、14:42 rolling decisions、首碰历史与 geometry。
- `EXP-023-B42-B50-seed2025-15min-rosout.log`：固定参数、运行汇总、hard guard、deadlock、reservation、A1 和计算量指标。

本实验不能宣称 PASS。B42/B50 的实际碰撞根因与后续修复方向仍为“未知，需要确认”；继续处理会涉及当前任务明确排除的动态调速/冲突准入或其他协调链，需另立范围审查。
