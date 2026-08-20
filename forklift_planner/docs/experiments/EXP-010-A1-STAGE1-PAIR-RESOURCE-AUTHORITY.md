# EXP-010：A1 修复阶段一——Pair/Resource 管辖收窄

## 1. 结论

**PARTIAL PASS**

阶段一的代码边界目标已经落实：`RuleEngine::resolvePairwiseConflicts()` 不再因为 Future A1
owner 身份、inactive staged `DepartureClusterCommitment` 或单车
`a1_departure_committed` 状态，把整个 pair 无条件划为 A1 special。A1 pair authority 现在只来自：

1. `futureA1OwnerForPair(a, b) >= 0`，即 synthetic future TO_B 与另一辆车当前 TO_A1
   确认存在 departure resource conflict；
2. `departureClusterOwnerForPair(a, b) >= 0`，即有效且 active 的 pair-level departure
   cluster。

focused tests、Release 构建、7/7 CTest、2 车 seed=2026 的 10 min 和 30 min 自然回归均通过。
原基线约 2213 s 的关键情形也在新 120 min 中自然复现：`first_t=9.600 s` 时先进入普通
MID 动态协调，选择 `YIELD/NOMINAL` 后完整重预测为 CLEAR，不再因 owner 身份直接
`a1_protected`；下一周期确认真实 future departure conflict 后，A1 才接管。

但 120 min 在 `sim_t=2759.3 s` 出现 2 次 hard guard，随后形成长期等待，因此不能给出完整
PASS。现场是 V1 从 `PICKUP_DWELL` 切入新 TO_A1 航段时，Future A1 仍因 horizon exceeded
未接管；普通动态协调虽令 V1 CREEP，仍在 V0 已进入交叉区后发生物理重叠。这与需求中明确保留
给阶段三的 DWELL→ACTIVE admission/launch gap 一致，本阶段未修改该链，也没有证据表明它由
被删除的三项身份式 A1 条件直接造成。为避免把未完成的 120 min 安全闭环写成 PASS，本报告保守
判定 **PARTIAL PASS**。

## 2. 基线、环境与范围

- Git 基线：`079608d`。
- 环境：WSL `Ubuntu-20.04-ros`，Ubuntu 20.04.6，ROS Noetic。
- 工作空间：`~/stage32_ws`，源码链接到当前 Windows 工作区。
- 参数：`vehicle_count=2`、`random_seed=2026`、`reproducible_task_random=true`，A1 cycle
  保持启用。
- 用户原有未提交修改：`planner_param.yaml` 中 `vehicle_count: 2`、`show_paths: false`；本轮未改动。
- 未修改 Future owner 选择/锁定、任务分配、DWELL launch gate、forward clearance、hard guard、
  ConflictReservation 生命周期、路径生成或任何安全参数。
- 按用户要求，本轮不创建 Git commit。

## 3. 修改前问题与审查结论

| 原条件 | 实际语义 | 结论 |
|---|---|---|
| `future_a1_pair` | pair 中任一车只是 Future owner 即为 true | 身份式过宽，删除其独立 authority |
| `departure_cluster_commitments_.count(key)` | staged/inactive 与 active 均命中 | 生命周期式过宽，改由 active/valid 查询函数决定 |
| `a1_departure_committed && path_s < priority_until` | 仅证明单车处于 departure prefix | 不能证明当前 pair 存在 departure conflict，删除其独立 authority |
| `futureA1OwnerForPair()` | 用 pending TO_B preview 与 other 当前 TO_A1 验证真实 protected conflict | 保留，作为 future pair authority |
| `departureClusterOwnerForPair()` | 验证 pair、path generation、active 状态和 release 条件 | 保留，作为 actual/active pair authority |

`a1_departure_committed` 没有从车辆生命周期删除；它仍被 active cluster 的生成、刷新与释放链使用。
删除的只是“单车 flag 足以接管任意 pair”这一用途。实际 TO_B departure 与特定 TO_A1 车辆形成
冲突时，`refreshDepartureClusterCommitments()` 会在 pair resolution 前生成/刷新 active cluster，
因此实际资源保护仍由 pair-level 证据承接。

## 4. 实际修改

### 4.1 `forklift_planner/src/multi_vehicle/rule_engine.cpp`

函数：`RuleEngine::resolvePairwiseConflicts()`。

原逻辑：

```text
a1_related = map 中存在 commitment
          OR pair 含 Future owner
          OR 任一车处于 departure prefix
          OR active cluster confirmed
          OR future departure conflict confirmed
```

新逻辑：

```text
a1_related = active departure cluster confirmed
          OR future departure conflict confirmed
```

同时删除 reservation holder 对单车 departure flag 的兜底选择，避免已收窄的 pair 又在 holder
阶段被身份式逻辑重新接管。代码旁保留简短注释：A1 ownership 是 pair/resource scoped，而不是
owner-identity scoped。

### 4.2 `forklift_planner/test/dynamic_speed_rule_engine_test.cpp`

新增 focused 覆盖：

- 单车 departure flag + 无实际 pair resource conflict：普通动态，无 reservation；
- Future owner + pending exit 远离当前 pair：普通 MID 动态，无 A1 fallback；
- inactive staged cluster：普通 MID 动态，无 A1 fallback；
- valid active cluster：保留 A1 reservation；
- owner 位于 `PICKUP_DWELL`：future admission 仍生成 inactive staged cluster，保护链不断档。

## 5. 修改后的 authority 真值表

| 场景 | `futureA1OwnerForPair` | active cluster | A1 接管 | 普通 rolling dynamic |
|---|---:|---:|---:|---:|
| 仅 Future owner 身份 | -1 | 否 | 否 | 是 |
| 仅 inactive staged cluster | -1 | 否 | 否 | 是 |
| 仅单车 `a1_departure_committed` | -1 | 否 | 否 | 是 |
| 普通 TO_A1 CROSSING/OPPOSING/SAME_DIRECTION | -1 | 否 | 否 | 是 |
| synthetic future TO_B 与 other TO_A1 确认冲突 | owner | 可暂未 active | 是 | 否 |
| valid active `DepartureClusterCommitment` | 任意 | 是 | 是 | 否 |

不存在第三种未经 pair/resource 验证的 actual-departure 独立充分条件。

## 6. PICKUP_DWELL 连续保护

当前调用链并非依赖 DWELL 车辆参与 pairwise motion：

```text
FutureA1Commitment + pending_dropoff_track
  -> enforceFutureA1Admission()
  -> synthetic exit preview (mission_phase=TO_B)
  -> staged DepartureClusterCommitment(active=false)
  -> activatePreparedDropoffLeg()
  -> refreshDepartureClusterCommitments()
  -> active=true
  -> enforceDepartureClusterCommitments()
```

focused test 确认 owner 在 `PICKUP_DWELL` 时仍能通过 admission 链保留 inactive staged
commitment；自然日志也记录了 future handoff 的 CREATE，随后 TO_B activation 后 active cluster
生效。因此本次收窄没有在 5 s 装货阶段制造新的 future-departure protection 空档。

本结论不等于 DWELL→ACTIVE launch admission 已完整解决；后者仍是明确留给阶段三的问题。

## 7. 构建与测试

```bash
cd ~/stage32_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
cd build
ctest --output-on-failure
```

结果：Release 构建成功，**7/7 tests passed**：

- conflict_zone_closure
- future_a1_policy
- spatiotemporal_interaction
- dynamic_speed_coordination
- dynamic_speed_rule_engine
- prediction_execution_consistency
- rolling_decision_timing

## 8. 2 车 seed=2026 自然回归

命令模板：

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=MINUTES vehicle_count:=2 \
  coord_log_file:=/mnt/d/desktop/forklift_a1_stage1_20260820/coord_MINUTES.log
```

| 时长 | completed ticks / sim time | hard guard | deadlock ticks | tasks V0/V1 | max wait V0/V1 |
|---|---:|---:|---:|---:|---:|
| 10 min | 6000 / 600.0 s | 0 | 0 | 8 / 7 | 15.6 / 19.5 s |
| 30 min | 18000 / 1800.0 s | 0 | 0 | 24 / 22 | 34.2 / 23.8 s |
| 120 min | 72000 / 7200.0 s | 2 | 8831 | 36 / 34 | 4440.8 / 4447.6 s |

### 8.1 动态协调与 reservation

| 时长 | CROSSING / OPPOSING / SAME | FAR / MID / NEAR（executed） | A1 skips（evaluated） | reservation ordinary / A1（executed） |
|---|---:|---:|---:|---:|
| 10 min | 1 / 8 / 1 | 2 / 1 / 7 | 58 | 0 / 11 |
| 30 min | 13 / 25 / 1 | 8 / 8 / 23 | 217 | 0 / 41 |
| 120 min | 2242 / 30 / 2 | 12 / 15 / 2247 | 280 | 0 / 55 |

120 min 后 44418 次 duplicate-authority 计数均发生在车辆已经形成物理重叠/长期等待之后，
是 forward-clearance 与普通动态 STOP 的后果统计，不是 ordinary reservation 被恢复；ordinary
reservation create 始终为 0。

### 8.2 Future/active departure activity

120 min 协调日志中，REAL snapshot 的 active DepartureCluster 生命周期为：

- CREATE 49；
- RELEASE 49；
- 其中 `future_handoff=true` CREATE 43，代表由 future pair protection 形成并移交的 cluster；
- `future_handoff=false` CREATE 6，代表直接从实际 TO_B/TO_A1 冲突形成的 active cluster。

预测 rollout 诊断中：CREATE/HOLD/RELEASE 为 316/266/373。由于现有日志没有直接输出
`futureA1OwnerForPair()` 调用返回计数，不能把 rollout CREATE 数量等同为函数有效次数；可追溯的
最小运行证据是上述 43 次 REAL future-handoff 生效。更精确的“函数调用有效次数”为**未知，需要确认**，
本阶段没有为统计而侵入控制逻辑。

### 8.3 2213 s 关键现场

新运行：

```text
sim_t=2213.7 s, plan=1195
baseline_first_t=9.600, interaction=CROSSING, band=MID
selected=YIELD/NOMINAL, after_action=CLEAR
reservation=not_created
```

下一 refresh：

```text
owner V0 -> V1
future departure pair/resource conflict confirmed
selection=SKIPPED reason=a1_protected
ConflictReservation create_reason=a1_related
```

这不是 authority 回弹：第一周期没有真实 departure conflict，普通 dynamic 是唯一 authority；第二周期
owner/preview 改变后真实 resource conflict 成立，A1 才接管。

## 9. 修改前后对照

| 120 min | 基线（修改前） | 本轮 |
|---|---:|---:|
| hard guard | 1，首次 2230.3 s | 2，首次 2759.3 s |
| deadlock ticks | 9886 | 8831 |
| tasks V0/V1 | 30 / 28 | 36 / 34 |
| A1 skips（evaluated） | 260 | 280 |
| active cluster CREATE/RELEASE（REAL） | 43 / 42 | 49 / 49 |
| reservation ordinary / A1（executed） | 0 / 48 | 0 / 55 |

原始计数随有效运行任务数增加而增加，不能单凭 A1 reservation 的绝对数判断管辖是否收窄。直接
证据是 focused truth-table tests 和 2213.7 s 等价现场：身份式误接管已经消失，而真实 future/active
resource protection仍然生效。

## 10. 120 min 失败归因

首次 hard guard 前：

- V0 已在 TO_B 航段 `slot 17 -> 33`，`path_s=4.13/7.11`，早已越过本航段
  `a1_departure_priority_until_s=0.758`；
- V1 在 `PICKUP_DWELL` 后于 `sim_t=2752.5 s` 启动新的 TO_A1 航段；
- Future A1 候选为 `horizon_exceeded`，没有 future owner/active departure cluster；
- 普通 rolling dynamic 令 V1 CREEP，但 V0 已进入多个 conflict zones，最终在 2759.3 s 重叠；
- 随后双方均 committed，形成等待环。

因此被删除的 `a1_departure_committed && path_s < priority_until` 在该现场本来也为 false；
`future_a1_pair` 和 staged cluster 身份条件也均不成立。该现场无法由恢复任一被删除的过宽条件修复，
归入阶段三 DWELL→ACTIVE admission/launch gate 缺口，而不是阶段一仍残留 authority 过宽。

## 11. 产物

日志目录：`D:/desktop/forklift_a1_stage1_20260820`。

| 文件 | SHA-256 |
|---|---|
| `console_10.log` | `3BFD0DDE72B6394CF0B80BEA18D1B385BEB95341967592FDB683CDADE83161DD` |
| `coord_10.log` | `7EDB00F43B64E640E70C845A9A102EE0C88122035168DFF981BAA4EDD31B4023` |
| `console_30.log` | `B3944EBDFD551A89B1FF5FD7208C00C3750CE6B4BC8A6D9C334F3C999EE1EA6F` |
| `coord_30.log` | `B0F4B46A36A313F6321BC5A591A779E43C65591E995509725E135AD670FA44F4` |
| `console_120.log` | `A70E9146539EDC4AFD381A7607263A1123C58518CC476034903537CE17710ABA` |
| `coord_120.log` | `0286DD6AA8CB36EABD88F91C6E21E3404879B0CCF7BB3F227B5E065842A94D67` |

## 12. 验收

| 验收项 | 结论 |
|---|---|
| Future owner 身份不再无条件接管 pair | PASS |
| inactive staged cluster 不再接管普通 pair | PASS |
| 单车 departure flag 收窄为非独立 authority | PASS |
| 普通 TO_A1 pair 恢复 rolling dynamic | PASS |
| Future/PICKUP_DWELL/actual TO_B departure 保护连续 | PASS（本阶段边界）；launch gate 仍待阶段三 |
| ordinary ConflictReservation create = 0 | PASS |
| actual occupancy 优先保持 | PASS（代码与 focused test） |
| 10/30 min 无 hard guard / deadlock | PASS |
| 120 min 无 hard guard / persistent deadlock | FAIL |

**最终结论：PARTIAL PASS**

