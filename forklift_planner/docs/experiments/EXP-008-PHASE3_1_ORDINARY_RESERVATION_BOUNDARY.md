# EXP-008：阶段3.1 普通道路与旧资源预约边界

## 1. 最终结论

**FAIL**

阶段3.1的结构目标已经实现：所有非A1 pair 均进入 rolling dynamic motion coordination；普通道路 `ConflictReservation CREATE=0`；三车以上不再进入 `multi_vehicle_legacy`；未来 rollout 中也不再由 `already_inside` reservation 覆盖 frame 0 的普通动作。A1 的 `FutureA1Commitment`、`DepartureClusterCommitment` 和 A1 reservation 路径仍保留。

但是正式回归发现新的稳定运动死锁，不能判 PASS：

- 2车 seed=2026、120 min：hard guard=0，但 deadlock ticks=7303，最终连续等待约3678 s，任务44/45。
- 3车 seed=2026、10 min：hard guard=0，但 deadlock ticks=804，最大等待约428 s，任务1/2/1。

最终现场不是普通 reservation 残留，而是车辆已经物理进入交互区后，rolling dynamic winner 的 NOMINAL 被既有 forward-clearance STOP 覆盖，形成 STOP/STOP。安全解决需要修改 forward-clearance 的覆盖关系或引入近端物理运动 commitment；前者超出本阶段明确保留范围，后者属于后续阶段，因此本阶段只记录，不越界实现。

## 2. 修改前源码审计

基线 commit：`9f639cc`。

### 2.1 ConflictReservation 全生命周期

`RuleEngine::resolvePairwiseConflicts()` 中只有一处实际创建赋值：

```cpp
conflict_reservations_[key] = r;
```

修改前该入口按 `create_reason` 区分：

| 触发条件 | 修改前类别 | 普通道路/A1 |
|---|---|---|
| 普通动态未提前 continue | `ordinary_dynamic` | 普通道路 |
| 任一车辆进入任一 pair zone | `already_inside` | 普通道路 |
| 制动余量不足 | `braking_safety` | 普通道路安全条件 |
| terminal docking | `terminal` | 非A1 |
| deadlock breaker | `deadlock_breaker` | 非A1 |
| `vehicles.size()>2` 且动态关闭 | `multi_vehicle_legacy` | 普通道路 |
| 其它 legacy special | `other_special` | 非A1 |
| Future/Departure/A1 departure owner | `a1_related` | A1 |

existing reuse 在重新计算本周期 `a1_related` 之前执行，因此 map 中任何 reservation 都会先跳过动态选择。update 有两个入口：A1 protected owner 替换尚未物理进入的软 owner；waiter 已物理进入时将 owner 改为 waiter。delete 有三类：车辆/路径 generation 失效、pair zones 变空、owner 越过 reservation exit。`SimSnapshot::reservations` 完整保存和恢复该 map。

### 2.2 双车限制

修改前：

```cpp
const bool dynamic_speed_enabled = vehicles.size() == 2;
```

该值控制 baseline/dynamic 分支。三车时 pair 循环和 TimedConflict 检测仍运行，但不进入 `evaluatePairSpeedCoordination()`，最终落到唯一 create 点并标记 `multi_vehicle_legacy`。

### 2.3 rollout 污染根因

frame 0 动态动作成功后通过 `continue` 避免 reservation。未来 frame 中车辆推进越过 zone enter 后，`a_inside_any_zone=true` 使旧 `ordinary=false`；因此 `ordinary && reuse_ordinary_coordination` 保护失效，控制流落到 legacy holder/create，生成 `already_inside`。随后 future frames 在 existing reuse 分支跳过动态并应用 holder/waiter STOP。问题不是 snapshot/restore 泄漏，而是污染已经写进同一 sandbox plan 的未来 `SimPlanFrame`，随后被真实执行。

## 3. 最小实现

### 3.1 RuleEngine 管辖边界

- `dynamic_speed_enabled` 改为 `vehicles.size() >= 2`。
- 非A1 rolling 管辖条件改为 `ordinary = !a1_related`；inside、terminal、deadlock 和 braking 都只作为动作/紧急度输入，不再切换到资源所有权。
- 每次 pairwise 开始前删除所有 `create_reason != a1_related` 的历史 reservation，防止旧 snapshot 继续跳过动态。
- 非A1制动不足直接选择 STOP，但应用动作后始终 `continue`，不创建 `braking_safety` reservation。
- 唯一 create 点只写入 `create_reason=a1_related`；普通、inside、terminal、braking、deadlock、multi 和 other 分类在当前代码中均无法到达该赋值。
- 若仅一车已进入任一相关 zone，本 rolling period 选择已进入者为 winner；若双方已进入当前 event，选择预测更快清出者。该结果不写 map，约2 s后重新判断，不构成 LocalMotionCommitment。
- 三车多pair通过现有 `applyActionRequest()` 的 `moreRestrictive()` 顺序聚合。`RollingDynamicDecision::targets` 改为按 vehicle/path generation upsert，确保 future frame 复用的是最终聚合动作，而不是同一车辆较早、较弱的 pair 建议。

### 3.2 rolling executor

`rollWorldModel()`、`buildSimulationHorizonPlan()` 和 `executeSimulationPlanSample()` 的结构无需改动：frame 0 保存 period decision，frame 1..149 继续推进任务/DWELL/A1世界状态，并以 `reuse_ordinary_coordination=true` 复用非A1聚合动作。由于 RuleEngine 的非A1定义不再排除 inside/terminal/deadlock，future frame 无法再绕过 reuse 去创建普通 reservation。

### 3.3 A1明确保留

未修改：

- `FutureA1Commitment` 生成、刷新和 admission；
- `DepartureClusterCommitment`、handoff、owner/waiter stop boundary 和 release；
- prepared A1->B exit path；
- pickup dwell、departure 和 `owner_release_exit_s`；
- `a1_related` reservation 的 create/update/reuse/delete。

## 4. 修改文件

- `forklift_planner/src/multi_vehicle/rule_engine.cpp`：阶段3.1边界、非A1历史 reservation 清退、多车pairwise启用、当前周期物理进度 winner、聚合 target upsert。
- `forklift_planner/include/forklift_planner/multi_vehicle/rule_engine.h`：更新 reuse 和 reservation 的权威注释，明确 reservation 仅用于A1事务。
- `forklift_planner/test/dynamic_speed_rule_engine_test.cpp`：更新 braking/inside 断言，新增三车三pair动态和唯一聚合 target 验证，保留A1 reservation测试。
- `forklift_planner/test/rolling_decision_timing_test.cpp`：增加 frame 0 NEAR 后未来帧进入旧 inside 条件仍无 reservation、动作不被污染的回归；existing reservation测试改为明确A1。
- `forklift_planner/launch/multi_vehicle_phase2_batch.launch`：新增默认值为2的 `vehicle_count` 参数，允许相同正式主链运行3车；其余参数和默认行为不变。
- 本实验报告。

未修改 dynamic speed `.h/.cpp`：现有 `evaluatePairSpeedCoordination()`、`evaluateSelectedAction()`、band 分类和 braking helper 本身均为纯 pair 输入，能够直接复用，无需复制第二套协调器。

## 5. 测试

环境：WSL `Ubuntu-20.04-ros`，ROS1 Noetic。

```text
catkin_make --pkg forklift_planner
cd build && ctest --output-on-failure
```

结果：构建成功，7/7 tests passed。

覆盖结果：

| Case | 结果 |
|---|---|
| 双车普通 crossing | 动态仲裁正常，reservation为空 |
| ordinary already-inside | 旧 injected reservation 被删除；未来 reuse frame 不创建新 reservation |
| braking safety | 选择 STOP，reservation为空 |
| 三车 | 三个pair均进入动态入口，目标按车辆唯一聚合，`multi_vehicle_legacy=0` |
| A1 | `future_a1_policy_test` 通过；A1 reservation create/reuse保留 |

## 6. 2车短时验证

最终实现的10 min短测：6000/6000真实tick，`sim_t=600.0 s`。

| 指标 | 结果 |
|---|---:|
| hard guard | 0 |
| deadlock ticks | 0 |
| tasks V0/V1 | 8 / 6 |
| max wait V0/V1 | 15.5 / 19.5 s |
| FAR/MID/NEAR | 2 / 3 / 6 |
| ordinary reservation create | 0 |
| A1 reservation create/delete | 12 / 12 |

## 7. 2车 seed=2026 正式120 min

命令：

```text
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=120 vehicle_count:=2 \
  coord_log_file:=/mnt/d/desktop/叉车/Testing/EXP008/stage3_1_2v_120_coord.log \
  debug_log_dir:=/mnt/d/desktop/叉车/Testing/EXP008/2v_120_debug
```

| 指标 | 结果 |
|---|---:|
| requested/completed ticks | 72000 / 72000 |
| reached real sim time | 7200.0 s |
| dt | 0.100 s |
| hard guard | 0 |
| deadlock ticks/recovery | 7303 / 0 |
| wedge episodes | 5 |
| reciprocal STOP cycles | 0 |
| tasks V0/V1 | 44 / 45 |
| max wait V0/V1 | 3676.4 / 3678.5 s |
| FAR/MID/NEAR | 11 / 22 / 1874 |
| evaluated YIELD/CREEP/STOP | 12 / 16 / 1868 |
| executed target YIELD/CREEP/STOP | 11 / 11 / 4574 |
| nominal recoveries | 4 |
| ordinary reservation create | **0** |
| A1 reservation create/update/delete | 74 / 2 / 74 |
| existing A1 reservation holds | 7141 |
| nonA1 inside/braking/terminal/deadlock/multi/other create | **全部0** |

动作时间：

| 车辆 | NOMINAL | YIELD | CREEP | STOP |
|---|---:|---:|---:|---:|
| V0 | 2635.5 s | 2.0 s | 37.7 s | 4093.6 s |
| V1 | 2627.9 s | 20.6 s | 40.6 s | 4069.9 s |

最终稳定现场：V0 `path_s=3.308`、V1 `path_s=0.098`，双方都在同一 crossing zone 内。dynamic 每周期选择 V0 NOMINAL / V1 STOP，但 forward-clearance 将 V0 覆盖为 `clear_block_V1`，故真实目标为 STOP/STOP。根据最终连续等待反推，稳定停滞约从 `sim_t≈3521.5 s` 开始；这是基于最终 wait 的推断，不是独立时间戳事件。

## 8. 3车验证

命令使用同一正式主链，仅设置 `vehicle_count:=3`，运行6000真实tick/600 s。

| 指标 | 结果 |
|---|---:|
| hard guard | 0 |
| deadlock ticks/recovery | 804 / 0 |
| wedge episodes | 3 |
| tasks V0/V1/V2 | 1 / 2 / 1 |
| max wait V0/V1/V2 | 427.2 / 427.8 / 412.8 s |
| FAR/MID/NEAR | 4 / 21 / 223 |
| ordinary reservation create | **0** |
| multi_vehicle_legacy create | **0** |
| A1 reservation create | 16 |

日志在同一 plan frame 真实记录多个普通 pair，例如 plan 3 同时出现 V0-V1 FAR 和 V0-V2 NEAR，证明三车不再因车辆数退出动态入口。后续出现多pair严格动作聚合和 forward-clearance 共同形成的全停。本阶段按要求记录，未增加 coordination graph、联合优化、priority hysteresis 或 LocalMotionCommitment。

## 9. 验收与尚未解决问题

| 验收项 | 结论 |
|---|---|
| 普通道路 ConflictReservation CREATE=0 | PASS |
| frame 0 普通动作不被future already-inside reservation污染 | PASS |
| braking safety保留STOP且不建普通reservation | PASS |
| 3车pairwise dynamic已启用 | PASS |
| A1现有机制未被删除，focused tests通过 | PASS |
| 2车120 min hard guard=0 | PASS |
| 2车120 min无稳定deadlock、任务能力不回归 | **FAIL** |
| 3车无all-stop/长期等待 | **FAIL** |

未解决问题：

1. 双方物理进入同一 interaction 后，动态 winner NOMINAL 会被 forward-clearance STOP 覆盖；纯“更严格动作优先”无法表达“安全地允许其中一辆先清出”。
2. 三车多pair各自正确，但没有全局一致性，多个 STOP/紧急建议可形成循环等待。
3. A1 rollout 仍每0.1 s运行现有 commitment/reservation 规则，本阶段按要求保留。
4. `RollingDynamicDecision` 的单个 band/after-action 摘要在多pair场景只代表最后写入的pair；聚合目标正确，但后续应增加显式per-pair诊断结构。该结构未在本阶段扩展。

## 10. 是否误做阶段3.2及以后内容

**没有。**

未实现或引入：`SharedSegment`、`OccupancyInterval`、`FollowingConstraint`重构、`LocalMotionCommitment`、coordination graph、多车联合优化、A1 Service Envelope，也未删除/重写 `ConflictZone`、`computeConflictZonesFull()` 或 `findConflictZones()`。未修改 forward-clearance、hard guard、路径生成、任务分配或 deadlock策略。

## 11. 实验产物

日志保存在仓库外同级目录 `Testing/EXP008`：

| 文件 | bytes | SHA-256 |
|---|---:|---|
| `stage3_1_2v_120_console.log` | 1,786,128 | `6427A98C949F421CFCC8372DC7F63720A81B9956A84BA225DC84A4AB4947D6CE` |
| `stage3_1_2v_120_coord.log` | 14,656,563 | `46A58A4DEB651ED2E41BCFA718D4D3687F0DC46D03F31A4B1B75B6E7522D4DBD` |
| `stage3_1_3v_10_v2_console.log` | 377,158 | `0F069229E7EBB44B98C41792B34F51DA1DA181F63536913ADF7D97D513D6376C` |
| `stage3_1_3v_10_v2_coord.log` | 8,267,458 | `99A4433AF7CB2435163EEAE853EBF1919BCC3230FBF5B3718146612D56F0E847` |

## 12. 最终判定

普通道路与旧 reservation 的管辖边界及多车pairwise入口已经按阶段3.1实现并通过focused验证；但正式双车和三车回归出现稳定死锁，整体阶段不能通过。

**FAIL**
