# EXP-011：A1 连续 Service Owner 与 Launch Admission

## 1. 总结结论

**PARTIAL PASS**

本阶段要求的两个局部机制已经实现并在自然运行中生效：A1 service owner 在
`TO_A1 -> PICKUP_DWELL -> TO_B` 生命周期内不再被 arrival ranking 抢占；从
`UNLOAD_DWELL` 发起下一条 `TO_A1` 航段前，会以候选路径检查当前 owner 尚未清出的
A1 departure resource，冲突时车辆保持 `DWELL + STOP + speed=0`，资源清出后再放行。
EXP-010 的 pair/resource scoped `a1_related` 定义未被扩大，ordinary
`ConflictReservation create` 在三组正式实验中均为 0。

但完整验收失败：seed 2024、2025、2026 的 120 min 回归各出现 1 次 hard guard，随后均
形成长期 deadlock。2024/2025 首次失败属于既有 actual-occupancy/departure handoff 缺口；
2026 首次失败发生在没有 service owner 的两条普通 `TO_A1` 路径上，归类为 ordinary
dynamic 问题。两类问题均超出 EXP-011 的授权边界，因此本阶段没有修改动态调速、hard
guard、安全参数或扩大 A1 authority 来掩盖结果。

此外，三个自然种子中 `faster_candidate_observed=0`。代码和 `CHANGE=0` 能证明已有 owner
不会被 ranking 覆盖，但本轮没有自然出现“owner 存在且另一 candidate 的预测到达更早”这一
特定证据，不能把该观察项写成已由自然实验直接覆盖。

基线 commit：`cf92d97`（EXP-010）。本轮按要求未创建 Git commit。

## 2. 实际修改

| 文件 | 位置 | 原逻辑 | 新逻辑 |
| --- | --- | --- | --- |
| `src/multi_vehicle_patrol_node.cpp` | `retainLockedFutureA1Owner()` / `buildSimulationHorizonPlan()` | owner 主要在 pickup dwell 保留，后续 ranking 可重新选 owner | 已有合法 owner 时只做 retain/release/invalidate；仅 `NO_OWNER` 才调用 ranking |
| 同上 | `retainLockedFutureA1Owner()` | generation 与 service 生命周期没有完整连续语义 | 显式允许同一次服务的 `TO_A1 N -> TO_B N+1`，并持续到 departure prefix 和全部 active clusters 均清空 |
| 同上 | `launchPickupLegWithA1Admission()` / `updateDwellAndTasks()` | `UNLOAD_DWELL` 完成后直接分配并激活下一条 `TO_A1` | 先在 clone 上得到确定性 candidate，再做 departure resource admission；HOLD 不改变真实路径 generation |
| `include/.../rule_engine.h`、`src/multi_vehicle/rule_engine.cpp` | `A1LaunchAdmission` / `checkA1LaunchAdmission()` | 无 launch 前资源检查入口 | 复用 `computeConflictZonesFull()` 与 `selectFutureA1ProtectedZones()`，分别支持 pending preview 和实际 TO_B prefix |
| `include/.../task_allocator.h`、`src/multi_vehicle/task_allocator.cpp` | `assignPickupLeg(..., emit_log)` | candidate clone 会产生正式任务日志 | 仅增加静默预览开关；实际分配、路径缓存及随机语义不变 |
| `launch/multi_vehicle_phase2_batch.launch` | `random_seed` arg | seed 固定写入 launch | seed 作为 launch arg，默认仍为 2026，便于正式矩阵复现 |
| `test/future_a1_policy_test.cpp` | 既有 case B | 测试文字仍宣称 earlier candidate 可替换 owner | 改为无 owner 时 ranking 选择更早 candidate；未新增 focused test 文件 |

没有修改 `dynamic_speed_coordination.cpp`、FAR/MID/NEAR 阈值、forward clearance、hard
guard、deadlock 策略、安全 margin 或普通 reservation 生命周期。

## 3. 修改后的 Service Owner 生命周期

```text
NO_OWNER
  -> ranking SELECT / SERVICE_OWNER_GRANTED
  -> TO_A1_LOCKED
  -> PICKUP_LOCKED
  -> TO_B_DEPARTING
  -> departure prefix clear AND all owner active clusters clear
  -> RELEASE
  -> NO_OWNER
```

当旧 owner 合法时，`predictA1Arrivals()` 仍计算候选用于诊断，但结果不再覆盖 owner。旧 owner
异常失效时本次 plan 只做 INVALIDATE，不在同一次 plan 内把另一个 candidate 解释成正常抢占。

## 4. Owner retain 条件

- `TO_A1`：`ACTIVE`、目标 A1、generation 与 owner 记录一致，且 prepared dropoff 有效。
- `PICKUP_DWELL`：`DWELL`、generation 一致，且 prepared dropoff 有效。
- `TO_B`：车辆处于本次已装载 B 航段，generation 为原值或唯一允许的 `N+1`；departure
  prefix 未清或任一以该车为 owner 的 departure cluster 仍 active 时继续 retain。

active cluster 通过完整 `RuleEngine` snapshot 遍历统计，不假设只有一个 pair。

## 5. Owner release 与异常失效

正常 release 只有：车辆已清出 `a1_departure_priority_until_s` 定义的 prefix，且该 owner 的
active departure cluster 数为 0。车辆可能在两次 refresh 之间到达 `UNLOAD_DWELL`，该状态也按
同一 prefix/cluster 条件正常 release。

异常 invalidation 包括 owner 丢失、服务 phase/target 无效、prepared dropoff 无效、无法解释的
path generation 变化以及在 prefix 清空前丢失 departure commitment。`horizon_exceeded` 不再是
release 原因。

## 6. path_gen handoff

owner 在 `TO_A1` 和 `PICKUP_DWELL` 保持 generation `N`。prepared dropoff 被激活后，只有同一
owner 的 `TO_B` generation `N+1` 被接受，并立即将 commitment generation 更新为 `N+1`。
其它跳变均按 `unexplained_path_gen_change` invalidation。正式三组运行
`invalidate=0`，说明自然任务循环中的 handoff 均被解释成功。

## 7. Launch Admission

原子边界位于 `updateDwellAndTasks()` 的 `UNLOAD_DWELL` 分支、正式调用
`assignPickupLeg()` 之前：

1. 复制 `VehicleAgent`，在 clone 上用缓存生成候选 `TO_A1` 路径；此操作不改变真实车辆、
   `path_gen` 或随机序列。
2. owner 仍为 `TO_A1/PICKUP_DWELL` 时，以其 `pending_dropoff_track` 构造 `N+1` 的 synthetic
   TO_B；owner 已在 TO_B 时使用实际 track、实际 `path_s` 与尚未清出的 departure prefix。
3. 复用 conflict zones 与 Future A1 protected-zone selection。candidate 已实际占用时继续保留
   `actual occupancy > future reservation`，不倒压已经进入的车辆。
4. 冲突时真实车辆保持 `DWELL`、`STOP`、`requested STOP`、`current_speed=0`，每 0.1 s
   重新检查但只在状态变化时写日志；清空后才对真实车辆调用一次 `assignPickupLeg()`。

自然证据（seed 2026）：0:46.3，V0 对实际 TO_B owner V1 的 4 个 protected zones 被 HOLD，
日志同时记录 `vehicle_mode=DWELL vehicle_speed=0.000`；0:54.9 资源清空后 ALLOW，记录
`vehicle_mode=ACTIVE vehicle_path_s=0.000`。pending preview 路径也在 1:11.4 自然触发 HOLD。

## 8. Authority 边界

`RuleEngine::decide()` 仍严格使用 EXP-010 定义：

```cpp
a1_related = departureClusterOwnerForPair(a, b) >= 0 ||
             futureA1OwnerForPair(a, b) >= 0;
```

service owner 身份、`a1_departure_committed` 或 inactive staged cluster 均未单独进入该表达式。
seed 2025 的 plan 671/672 在 service owner 锁定期间仍执行普通 MID 动态协调，目标为
`NOMINAL/CREEP` 且重新预测 CLEAR，说明身份没有吞掉普通道路协调。

## 9. Build 与 CTest

环境：WSL `Ubuntu-20.04-ros`、ROS Noetic，工作区 `~/stage32_ws`，Release。

- `catkin_make -DCMAKE_BUILD_TYPE=Release`：成功。
- 初次尝试的包级 `run_tests_forklift_planner` Make target 不存在；这是命令入口问题，不是测试
  失败。
- 改用 build 目录实际 CTest 入口：7/7 passed，0 failed。
- 覆盖：conflict-zone closure、Future A1 policy、spatiotemporal interaction、dynamic-speed
  coordination/rule-engine、prediction-execution consistency、rolling-decision timing。
- `git diff --check`：通过，仅显示仓库现有 Windows 行尾转换提示，无 whitespace error。

## 10. 三组 120 min 正式结果

统一参数：`vehicle_count=2`、`reproducible_task_random=true`、10 Hz；每组为真实 batch clock
的 72,000 ticks / 7,200.0 s，不是 rollout 累计时间。

| seed | sim time | hard guard | deadlock ticks / first | first failure | tasks V0/V1 | max wait V0/V1 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2024 | 7200.0 s | 1 | 13043 / 678.6 s | 653.7 s | 9 / 7 | 6546.5 / 6549.7 s |
| 2025 | 7200.0 s | 1 | 11802 / 1299.1 s | 1273.9 s | 15 / 16 | 5928.7 / 5926.2 s |
| 2026 | 7200.0 s | 1 | 12628 / 886.1 s | 861.1 s | 11 / 10 | 6343.0 / 6339.0 s |

三组均完成要求时长，但均未达到 `hard_guard=0` 与无 persistent deadlock 的安全目标。

## 11. Service Owner 统计

| seed | CREATE | HOLD | CHANGE | RELEASE | INVALIDATE | arrival preemption | faster candidate observed | max duration |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2024 | 17 | 3477 | 0 | 16 | 0 | 0 | 0 | 6569.2 s |
| 2025 | 32 | 3347 | 0 | 31 | 0 | 0 | 0 | 5945.5 s |
| 2026 | 21 | 265 | 0 | 21 | 0 | 0 | 0 | 41.3 s |

2024/2025 的末个 owner 在碰撞后因 active cluster 永久未清而继续 HOLD；这不是“已远离 A1 却
无资源仍泄漏”，而是失败后的真实 cluster 未释放。它仍表明系统存在 persistent deadlock，不能
作为正常 lifecycle PASS。正常完成的 owner 周期均能 release，无 cluster 的周期也能依靠 prefix
clear release。

## 12. Launch Gate 统计

| seed | ALLOW | HOLD transitions | retry checks | released after HOLD | max HOLD | active holds at end |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2024 | 16 | 6 | 663 | 6 | 13.0 s | 0 |
| 2025 | 31 | 14 | 1938 | 14 | 23.6 s | 0 |
| 2026 | 21 | 13 | 1681 | 13 | 23.0 s | 0 |

所有自然 HOLD 最终都成功放行，末尾无 active launch hold；未观察到 launch 饥饿。ALLOW 也包含
无 owner 或 candidate 不侵入 protected resource 的正常放行，因此 gate 不是“A1 同时只允许一车
TO_A1”。

## 13. DepartureCluster 统计

以下只统计 `[SOURCE=REAL]`，不把 rollout 内反复构造/恢复的事件计入：

| seed | CREATE | HOLD event | RELEASE | INVALIDATE | CREATE future_handoff=true | CREATE already_inside=true |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2024 | 6 | 0 | 5 | 0 | 6 | 1 |
| 2025 | 10 | 0 | 9 | 0 | 10 | 1 |
| 2026 | 4 | 0 | 4 | 0 | 4 | 0 |

当前 cluster 日志只在状态转换时输出，持续 active 不单独产生 `event=HOLD`。2024/2025 少 1 次
RELEASE 对应首次碰撞后一直 active 的 cluster，并与长期 owner HOLD、deadlock 一致；不是异常
INVALIDATE。

## 14. 普通动态协调统计

| seed | CROSSING | OPPOSING | SAME_DIRECTION | FAR | MID | NEAR | A1 fallback/skips | ordinary reservation create |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2024 | 25 | 17 | 2 | 12 | 13 | 19 | 52 | 0 |
| 2025 | 12 | 53 | 1 | 10 | 20 | 36 | 112 | 0 |
| 2026 | 23 | 3218 | 1 | 17 | 31 | 3194 | 30 | 0 |

表中 interaction/band 与 A1 fallback 使用 evaluated metrics；ordinary reservation 使用
evaluated 和 executed 两处核对，均为 0。2026 的大量 OPPOSING/NEAR 与 duplicate authority 是
首次碰撞后长期僵持的后果，不应解释为正常吞吐表现。

## 15. seed 2026 旧问题复核

- 旧约 2213 s owner 抢占：本次在 861.1 s 已先发生新的普通道路失败，无法自然推进到旧时间点；
  三组正式运行均 `CHANGE=0`、arrival preemption=0，但又均
  `faster_candidate_observed=0`，所以“更快 candidate 存在时仍 HOLD 原 owner”没有获得所要求的
  自然直接证据。实现上已有 owner 时不再执行 selection。
- 旧约 2759.3 s launch gap：精确旧时间点同样未到达，但该机制在此前已经多次自然复现并被 gate
  阻止。seed 2026 共 13 次 HOLD 均发生在移动前、全部随后 ALLOW；0:46.3→0:54.9 是一条完整
  actual-TO_B 时间线，1:11.4 起还覆盖 pending-preview 时间线。因此旧 gap 的触发机制已被修复，
  但不能宣称 2759.3 s 原轨迹被完整重放。

## 16. 正式失败归因

### seed 2024 / 2025

两辆车曾在 `NO_OWNER` 状态分别正常 launch；其中一车随后进入 15 s horizon 并成为 service
owner，另一车已实际进入相关区域。现有 `actual_occupied_priority` 保留实际占用优先，owner 的
TO_B handoff 没有在出发前解决与该已在途车辆的冲突，最终 hard guard。owner 无非法 CHANGE、
没有过早 release，launch gate 也没有在有 owner 时误放行。归类：**既有 actual occupancy /
departure handoff 缺口**，不是本阶段 launch admission 或 lifecycle bug。

### seed 2026

13:49.9 V0 在 owner V1 departure clear 后由 HOLD 转 ALLOW，13:49.9 owner 正常 RELEASE；
14:17.1 V1 在 `NO_OWNER` 下启动，V0/V1 均因 `horizon_exceeded` 不在 A1 candidate 集，14:21.1
仍为 `NO_OWNER`，随后 861.1 s 两条普通 `TO_A1` 路径 hard guard。归类：**ordinary dynamic
问题**。Launch gate 按设计只管当前 service owner 的 departure resource，不应扩大为所有 TO_A1
互斥锁。

共同检查结果：owner 非法抢占=否；正常周期过早 release=未见；异常 invalidation=0；Launch
HOLD 均在移动前；所有 HOLD 均最终放行；安全 candidate 没有被永久阻塞；actual occupancy 原则
保留；ordinary dynamic 在 authority 外仍运行。

## 17. 日志与最终验收

持久日志目录：`D:/desktop/forklift_exp011_20260820`。

| seed | console SHA-256 | coord SHA-256 |
| ---: | --- | --- |
| 2024 | `E0312F4284037AD7C4775015EC56BEC9C48752F5E05AA649E8035D69B945D977` | `497597F0CAF92FDF6F289A88F44CDD03F9A4F99BF1109CE29D4B02EE798D3FE5` |
| 2025 | `797A8B65352F096752A800AE4C99AD84BAB5FC0F19E7B5350AAD534580130D75` | `D8304B6838E64F934D7F4D257F1ADF10779AB985CCA3485EAE17466176DDAE76` |
| 2026 | `1CAD0D012C8D4C9B096830B6CFB3E3D3FD47485DBB3DD933402456A37F46DC3E` | `B732A7DE8427C5543D878E84257933A6AE36752A5ED4CCF95C84148902313EDC` |

| 验收项 | 结果 |
| --- | --- |
| 正常 owner 不发生 arrival-ranking 抢占 | PASS（CHANGE/preemption 均为 0） |
| 自然出现更快 candidate 但 owner 不换 | **未覆盖**（三组 observed 均为 0） |
| TO_A1→PICKUP_DWELL→TO_B 连续、N→N+1 正确 | PASS（自然周期、INVALIDATE=0） |
| prefix + 全部 active clusters clear 后 release | PASS（正常周期） |
| 无 owner 泄漏或 persistent deadlock | **FAIL**（2024/2025 失败后 cluster 持续 active） |
| 危险 launch 在移动前 HOLD，clear 后继续 | PASS（33 次 HOLD，33 次 released） |
| safe candidate 不被无意义永久 HOLD | PASS（active holds at end 均 0） |
| EXP-010 authority、ordinary reservation=0、actual occupancy 保留 | PASS |
| Release build / existing CTest | PASS（7/7） |
| 三组 120 min hard guard=0、无 persistent deadlock | **FAIL** |

因此最终结论为：**PARTIAL PASS**。EXP-011 的局部实现可独立评审和回退，但当前系统尚不能以
三种子 120 min 安全回归通过；后续应单独立项处理 actual-occupancy/departure handoff 与 ordinary
dynamic 两类问题，不能在本提交中扩大 A1 service owner authority。
