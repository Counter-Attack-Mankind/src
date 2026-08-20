# EXP-012：Slot Departure Admission 核查与最小修复

## 1. 结论

**PARTIAL PASS**

源码和 EXP-011 seed=2026 日志确认了四个问题：`UNLOAD_DWELL -> ACTIVE` 前没有普通道路
准入；A1 launch gate 使用完整未来路径而可能过度 HOLD；OPPOSING shared segment 从 `s=0`
开始时会把刚激活车辆误判为 `actually_inside`；普通动态候选即使重新预测仍在 `t=0` 冲突，
也会被写入执行并记录 `accepted=true`。

最小修复已实现并通过 7/7 CTest。seed=2026 的 30 min 自然回归中 hard guard 从 EXP-011
同 seed 的 861.1 s 首次事件降为 0；所有 `after_action=CONFLICT@0 accepted=true` 消失。

但该自然回归从 203.6 s 起出现稳定 deadlock，因此不能判 PASS。此 deadlock 的 launch 首个
物理冲突发生在候选已经清出 slot sweep 之后，按本阶段明确规则必须 ALLOW；随后既有动态协调
无法安全清除冲突，unresolved-immediate 兜底将双方 STOP。没有把 gate 扩大成“15 s 内任意冲突
都 HOLD”，也没有越界修改 deadlock、priority 或 reservation。

基线 commit：`a39e4c0`（EXP-011）。本轮未提交 Git commit。

## 2. 根因确认

1. `MultiVehiclePatrolNode::updateDwellAndTasks()` 的 `UNLOAD_DWELL` 分支只调用原
   `launchPickupLegWithA1Admission()`。原 gate 只读取 Future A1 service owner；没有遍历当前
   ACTIVE 道路车辆。通过后 `TaskAllocator::assignPickupLeg()` 立即写入 `ACTIVE/NOMINAL`。
2. 原 `RuleEngine::checkA1LaunchAdmission()` 对完整候选路径的全部 protected closure zones
   计数，没有候选离库边界。远距离交集也会在 owner 生命周期内阻塞发车。
3. `predictOccupancyInterval()` 用 shared segment 的原始 enter_s 判断初始占用。B19 日志中
   segment 覆盖候选 `s=0`，因此 V1 在刚激活时取得 `actually_inside`，进而赢得 OPPOSING
   occupancy priority。
4. `RuleEngine::resolvePairwiseConflicts()` 在 `evaluation.conflict_free=false` 时仍无条件应用
   `selected_action_a/b`；日志中的 `accepted=true` 是硬编码。原日志精确记录
   `STOP/NOMINAL after_action=CONFLICT@0.000 accepted=true`。

需要区分：OPPOSING occupancy violation 是共享道路的时序语义，不等于同一时刻已有物理 OBB
重叠。同步 OBB detector 始终保留。

## 3. 实际修改

| 文件 | 函数/结构 | 修改 |
| --- | --- | --- |
| `vehicle_agent.h` | `VehicleAgent` | 增加 `slot_departure_clear_s`，只表示 source slot sweep 清出弧长 |
| `task_allocator.h/.cpp` | `slotDepartureClearS()`、`assignPickupLeg()` | 复用现有 `poseInSlotSweep()`，沿真实 B→A1 track 找首次完整清出边界；正式/clone candidate 得到同一值 |
| `rule_engine.h/.cpp` | `SlotDepartureAdmission`、`checkSlotDepartureAdmission()` | 统一执行 A1 离库前缀检查和对 ACTIVE 车辆的完整 15 s NOMINAL 同步 OBB 预测 |
| `rule_engine.cpp` | `checkA1LaunchAdmission()` | protected zone 只有 candidate-side enter 不晚于 `slot_departure_clear_s` 才 HOLD |
| `spatiotemporal_interaction.cpp` | `detectSharedSegmentInteraction()` | public-road occupancy enter 改为 `max(shared_enter_s, slot_departure_clear_s)`；未清出时不设 `actually_inside` |
| `multi_vehicle_patrol_node.cpp` | 原 launch helper / `updateDwellAndTasks()` | clone 准备后先统一 gate；HOLD 保持真实车辆 DWELL/STOP/0，ALLOW 后才正式 assign 一次；新增 `[SLOT_DEPARTURE]` 日志 |
| `dynamic_speed_coordination.cpp`、`rule_engine.cpp` | immediate fallback / 日志 | 非 SAME_DIRECTION 的 unresolved `t<=prediction_step` 改为 STOP/STOP、winner=-1、`accepted=false` |
| 三个既有测试文件 | 原测试内追加最小 case | 覆盖离库内 HOLD、离库后 ALLOW、A1 近/远 closure、occupancy 边界和 unresolved immediate STOP |

首次把 STOP/STOP 兜底泛化到 SAME_DIRECTION 时，既有测试证明会破坏“前车继续清出、后车
停止”的稳定权威；最终实现明确排除 SAME_DIRECTION。这是基于测试证据的必要收窄。

## 4. 新调用链

```text
UNLOAD_DWELL complete
  -> clone VehicleAgent
  -> assignPickupLeg(clone, emit_log=false)
  -> calculate slot_departure_clear_s
  -> RuleEngine::checkSlotDepartureAdmission(
       service owner, clone, current vehicles, 15 s)
       -> A1 protected closure: only clone prefix [0, clear_s]
       -> every existing ACTIVE vehicle: NOMINAL/NOMINAL predictor + OBB
       -> first physical conflict candidate_s <= clear_s ? HOLD : ALLOW
  -> HOLD: real vehicle remains DWELL + STOP + speed=0; retry next tick
  -> ALLOW: assignPickupLeg(real) once -> ACTIVE
  -> existing rolling dynamic coordination
```

没有增加 MissionPhase、commit 区、长期 owner、普通 reservation 或第二套动态协调器。

## 5. B19 场景为何不再按旧路径失败

旧日志在 14:17.1 直接 ALLOW V1，下一 plan 立即得到
`OPPOSING first_t=0 s_a=3.332 s_b=0`，V1 因 shared segment 从 0 开始被当作 actual occupant，
随后 `STOP/NOMINAL ... accepted=true`。

修改后：

- clone 在真实车辆仍为 DWELL 时参加 admission；若同步 OBB overlap 发生于
  `candidate_s <= slot_departure_clear_s`，existing ACTIVE 车辆直接成为 blocker，candidate
  不进入普通 priorityWinner。
- 即使 safe launch 已发生，在 candidate 尚未清出 slot sweep 时，OPPOSING occupancy 的进入
  弧长不会早于 `slot_departure_clear_s`，也不会取得 `actually_inside`。
- 即使后续出现 unresolved immediate 冲突，执行结果为 STOP/STOP、winner=-1、
  `accepted=false`，不再让名义 winner NOMINAL 前进。

本轮自然任务序列因 A1 远距离 closure 不再 HOLD 而在早期已发生变化，未精确重放 14:17.1
B19 原轨迹；上述三条分别由现有测试和自然日志覆盖，不能声称旧时间线被逐帧完全复现。

## 6. A1 远距离 closure 不再过度 HOLD

`computeConflictZonesFull()` 和 `selectFutureA1ProtectedZones()` 仍完整运行，owner-side protected
closure 定义未变。变化仅是 launch admission 的 candidate-side filter：

```text
zone.s_other_enter <= candidate.slot_departure_clear_s -> immediate prefix
zone.s_other_enter >  candidate.slot_departure_clear_s -> allow launch
```

后者在车辆进入道路后继续由现有 DepartureClusterCommitment、waiter stop boundary、braking
与普通 rolling dynamic 处理。单元测试分别覆盖近端 HOLD 和远端 ALLOW。30 min 自然回归中
`a1_prefix_hold=0`，而 EXP-011 同 seed 曾有多次完整路径 A1 HOLD；这说明远距离 gate 已明显
收窄，但该次自然运行没有恰好产生近端 A1 prefix HOLD，近端分支证据来自测试。

## 7. `after_action=CONFLICT@0 accepted=true`

最终代码中该组合不再成立：非 SAME_DIRECTION candidate 若完整重预测仍在当前/立即样本冲突，
输出 `STOP/STOP`、`winner=V-1`、reason=`rolling_unresolved_immediate_stop`，日志
`accepted=false`。FAR 的有意 defer 不受影响；SAME_DIRECTION 继续由既有前后车权威处理。

最终 30 min coord log 统计：

- `after_action=CONFLICT@0.000 accepted=true`：0。
- `after_action=CONFLICT@0.000 accepted=false`：811（稳定 deadlock 后的重复 rolling refresh）。

这证明错误“成功接受”语义已消失，但也如实显示后续 deadlock 未解决。

## 8. 测试与回归

环境：WSL `Ubuntu-20.04-ros`、ROS Noetic、`~/stage32_ws`。

- Release build：成功。
- CTest：7/7 passed。
- 修改既有测试，没有新建大规模构造测试：
  - `spatiotemporal_interaction_test`：slot vehicle 的 public occupancy enter/actually_inside；
  - `dynamic_speed_rule_engine_test`：离库内 OBB HOLD、离库后冲突 ALLOW、A1 near/far；
  - `dynamic_speed_coordination_test`：unresolved immediate 不保留 NOMINAL winner。

seed=2026、2 车、reproducible、30 min / 18,000 ticks 自然回归：

| 指标 | 结果 |
| --- | ---: |
| completed / sim time | 18000 / 1800.0 s |
| hard guard | 0 |
| deadlock | 3193 ticks，首次 203.6 s |
| tasks V0/V1 | 2 / 2 |
| max wait V0/V1 | 1621.6 / 1635.6 s |
| slot ALLOW / HOLD / released | 4 / 1 / 1 |
| ordinary-road HOLD / A1-prefix HOLD | 1 / 0 |
| max slot HOLD / active at end | 2.8 s / 0 |
| ordinary reservation create | 0 |

自然 HOLD 时间线：1:11.4，V1 仍为 `DWELL speed=0`，`clear_s=0.320`，预测在
`first_t=1.2 candidate_s=0.145` 与 V0 CROSSING，因此 HOLD；1:14.2 冲突清除后以
`ACTIVE path_s=0` ALLOW。它证明 gate 在真实移动前生效且可正常重试释放。

自然失败时间线：2:44.5 V1 的首个物理冲突不在离库前缀，因此按规则 ALLOW；后续冲突从
MID/NEAR 演进到 current conflict，2:50.5 起 STOP/STOP accepted=false，203.6 s 被 deadlock
检测。修复阻止了碰撞式继续前进，但未提供新的脱困策略。

## 9. 日志与验收

日志目录：`D:/desktop/forklift_slot_departure_20260820`。

- `seed2026_30_v2_console.log` SHA-256：
  `69D6144615EBD7CA648BFA3EF7086028801F2311CEF145EF82541DA9F567C2DD`
- `seed2026_30_v2_coord.log` SHA-256：
  `D722FDFBC1532D50B23853F511149A8E7CDCA7BE50BBB576C1AD86E832009CC5`

| 验收项 | 结果 |
| --- | --- |
| 普通道路统一离库 gate 存在且在 ACTIVE 前执行 | PASS |
| 只按 candidate path-space 离库前缀 HOLD | PASS |
| A1 远 closure ALLOW、近 prefix HOLD | PASS（测试）；自然近端未覆盖 |
| slot vehicle 不取得 shared-road actual occupancy | PASS |
| 库位附近真实 OBB detector 保留 | PASS |
| unresolved immediate 不再 NOMINAL winner / accepted=true | PASS |
| FAR/MID/NEAR、priority、service owner、reservation 边界未扩展 | PASS |
| Release / CTest | PASS |
| 30 min hard guard=0 | PASS |
| 无新增稳定 deadlock / 任务持续完成 | **FAIL** |

因此结论为 **PARTIAL PASS**。当前 diff 是可独立回退的离库准入与安全语义修复；若继续处理
203.6 s deadlock，需要另行审查“离库后冲突在进入 immediate 前为何没有可执行清出顺序”，不应
在本阶段把 SlotDepartureAdmission 扩大为全 horizon 互斥或暗中恢复普通 reservation。
