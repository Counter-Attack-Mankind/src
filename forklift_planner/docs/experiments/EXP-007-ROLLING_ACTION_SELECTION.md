# EXP-007：滚动周期单动作动态速度协调

## 1. 结论

**PASS**

本阶段把普通双车动态速度协调从同周期 `YIELD -> CREEP -> legacy fallback` 候选搜索改为单动作选择：`FAR -> NOMINAL`、`MID -> YIELD`、`NEAR -> CREEP`，制动余量不足时直接 `STOP`。所选动作只做一次完整 15 s rollout；rollout 中仍有冲突只作为趋势诊断，不再否决动作、继续搜索第二个动作或创建普通 `ConflictReservation`。

Ubuntu 20.04 / ROS Noetic、2 车、seed=2026 的正式回归真实推进 72000 个 0.1 s 主仿真 tick，达到 `sim_t=7200.0 s`。结果为 hard guard=0、deadlock ticks/recovery=0/0、reciprocal STOP cycle=0、普通动态 reservation create=0，V0/V1 完成 86/95 个任务，总数 181，与阶段 2.3 持平。

## 2. 代码基线与边界

- 修改时 Git HEAD：`fe417d4`。
- 正式业务仍为 `B -> A1 -> B`；未修改 A1 admission/departure、任务分配、路径生成、following、hard guard、forward clearance 或 deadlock 策略。
- 动态速度仍只在 `vehicles.size()==2` 时启用；三车及以上仍使用 legacy pairwise reservation，本阶段只用 focused test 确认该边界，没有扩展多车算法。
- 未实现后续阶段的 `SharedSegment`、`OccupancyInterval`、新 following 或 `LocalMotionCommitment`。

## 3. 真实 rolling 调用链与时间语义

调用链为：

```text
MultiVehiclePatrolNode::tick()/runBatch()
  -> rollWorldModel()：从真实车辆和 RuleEngine snapshot 建立 15 s sandbox
  -> sandbox frame 0: RuleEngine::decide()
  -> RuleEngine::resolvePairwiseConflicts()
  -> evaluatePairSpeedCoordination()
  -> applyActionRequest()/applyRequestedActions()
  -> 保存 RollingDynamicDecision/period_ordinary_decision
  -> sandbox frame 1..149: reuse_ordinary_coordination=true
  -> restore 真实 RuleEngine 状态
  -> executeSimulationPlanSample()/advanceVehicles() 推进一个真实 0.1 s tick
```

实际 YAML/代码语义：prediction horizon=15.0 s，prediction step=0.05 s，rolling refresh period=2.0 s，真实 update rate=10 Hz，即执行步长约 0.1 s。一个 rolling period 的普通目标只在 frame 0 基于真实周期起点选择一次；后续 sandbox 帧复用该目标，不再次进行普通 FAR/MID/NEAR 仲裁。安全链、A1 和已有 reservation 仍可覆盖该目标。约 20 个真实 tick 后，从新的真实位置和中间速度重新建立 15 s 预测并重新选择。

`runBatch()` 每次外层循环只执行一次真实状态推进，并执行 `sim_time_ += 0.1`。sandbox 使用 snapshot/restore，150 个预测帧不增加真实 `tick_count_` 或 `sim_time_`。正式日志的 `[BATCH_RUNTIME]` 给出 requested/completed ticks=72000/72000、real_sim_t=7200.0、dt=0.100。

## 4. 动态速度修改前后

修改前：

```text
NOMINAL baseline conflict
  -> YIELD rollout，要求完整 15 s CLEAR
  -> 未 CLEAR 则同周期 CREEP rollout
  -> 仍未 CLEAR 则 legacy fallback / ConflictReservation
```

修改后：

```text
baseline first_t + existing winner/loser + braking safety
  -> 本周期唯一最终动作
     FAR=NOMINAL / MID=YIELD / NEAR=CREEP / unsafe=STOP
  -> 只对该动作做一次完整 15 s rollout
  -> 记录 CLEAR、剩余 first_t 和 delay
  -> 普通场景直接接受；约 2 s 后从真实状态重新判断
```

紧急情况允许从 NOMINAL/YIELD/CREEP 直接越级到 STOP；这不是同周期候选搜索，而是最高优先级安全覆盖。winner/loser 继续使用现有 `priorityWinner()`/`preferred_winner_id`，winner 保持 NOMINAL。

## 5. 修改文件

- `include/.../dynamic_speed_coordination.h`、`src/multi_vehicle/dynamic_speed_coordination.cpp`：以 `selectRollingSpeedAction()`、`evaluateSelectedAction()` 取代 YIELD/CREEP candidate list；一次选择、一次完整 rollout，新增冲突延后量诊断。
- `include/.../rule_engine.h`、`src/multi_vehicle/rule_engine.cpp`：在既有普通双车入口应用单动作；剩余冲突不再进入普通 reservation；仅制动安全不足继续走保留的 safety fallback；为 reservation 增加创建原因并统计 create/update/delete。
- `src/multi_vehicle_patrol_node.cpp`：在真实执行口径统计 band/action 映射、紧急 STOP 来源、延后/最终解除、reservation transition 和真实 tick/sim_t；sandbox 评估口径继续独立输出。
- `test/dynamic_speed_coordination_test.cpp`：覆盖配置边界、四种动作映射、单次 rollout、剩余冲突仍接受和动力学约束。
- `test/dynamic_speed_rule_engine_test.cpp`：覆盖 FAR/MID/NEAR、紧急 STOP、普通 reservation-free、已有 reservation、A1 和三车 legacy。
- `test/rolling_decision_timing_test.cpp`：覆盖 2 s 周期内目标复用、下周期重新判断及完整 15 s 诊断。

没有新增生产 `.h/.cpp` 文件，没有复制第二套协调器。

## 6. ConflictReservation 完整审计

当前只有 `RuleEngine::resolvePairwiseConflicts()` 中一处 `conflict_reservations_[key] = r` 创建入口。创建后，pair lookup 在普通动态入口之前执行，因此已有 reservation 会跳过动态速度选择；holder/waiter 按 owner、物理占用和 A1 protected owner 决定，必要时只更新 owner。路径 generation 失效、冲突区消失或 owner 越过 exit 后删除。`snapshot()`/`restore()` 完整保存/恢复 reservation map，sandbox 变化不会直接污染真实状态；计划帧被真实执行时才按 create/update/hold/delete 计入 EXECUTED 指标。

| 触发场景 | 进入普通动态调速 | 创建 reservation | 创建原因 | 释放条件 | 后续迁移 |
|---|---|---|---|---|---|
| 新普通双车 FAR/MID/NEAR | 是 | 否 | `ordinary_dynamic` 应为 0 | 不适用 | 直接保留 rolling arbitration |
| 制动余量不足 | 先选 STOP，后进安全 fallback | 是 | `braking_safety` | owner 清出/zone 或路径失效 | 未来由 LocalMotionCommitment/硬安全边界承接 |
| A1 commitment/admission/departure | 否 | 是 | `a1_related` | 现有 A1 生命周期 | 保留专用 A1 语义 |
| 已进入任一 interaction zone | 否 | 是 | `already_inside` | owner 越过 exit 或 zone/path 失效 | 未来 LocalMotionCommitment |
| terminal docking | 否 | 是 | `terminal` | 现有 exit/失效条件 | 保留或建立 terminal 专用约束 |
| deadlock breaker | 否 | 是 | `deadlock_breaker` | 现有 exit/失效条件 | 后续独立审计，不在本阶段替换 |
| 三车及以上普通 pair | 否（动态入口关闭） | 是 | `multi_vehicle_legacy` | 现有 exit/失效条件 | 等 multi-vehicle conflict graph 阶段 |
| 已有 reservation | 跳过 | 不重复创建；可 hold/update | 保留原 `create_reason` | owner 越过 exit、zone/path 失效 | 逐类缩短 ownership 生命周期 |

普通 2V rollout 未完全 CLEAR 时，代码在应用所选动作后直接 `continue`；只有 `braking_safety_fallback` 不提前 continue。因此“仅因 15 s 未完全 clear 而创建普通 reservation”的答案是 **NO**。正式 EXECUTED 统计也得到 `ordinary_create=0`。

## 7. Focused tests 与构建

环境：WSL `Ubuntu-20.04-ros`，ROS1 Noetic。

```text
catkin_make --pkg forklift_planner
cd build && ctest --output-on-failure
```

结果：构建成功，7/7 CTest passed。关键 focused 输出：

```text
[MID-DELAY] baseline_first_t=5.05 selected=YIELD after_first_t=5.3 delay=0.25 accepted=true
[ROLLING-FAR] start_first_t=10.05 future_mid_first_t=8.15 frames_nominal=20
[ROLLING-REFRESH] first_t=8.05 new_target=NOMINAL/YIELD
[FULL-HORIZON-ACTION] remaining_conflict_t=5.05 accepted=true
```

RuleEngine focused test 还确认：FAR/MID/NEAR 普通路径均不建 reservation；NEAR 可 NOMINAL 直接到 CREEP；制动不足直接 STOP 并允许 `braking_safety` reservation；已有 reservation 跳过动态；A1 与三车 legacy 原路径仍可创建并带分类原因。

## 8. 2V / seed=2026 / 120 min 正式验证

命令：

```text
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=120 \
  coord_log_file:=/mnt/d/desktop/叉车/Testing/EXP007/rolling_action_120_coord.log \
  debug_log_dir:=/mnt/d/desktop/叉车/Testing/EXP007/debug
```

| 指标 | 结果 |
|---|---:|
| environment | WSL Ubuntu 20.04 / ROS Noetic |
| vehicles / seed / reproducible | 2 / 2026 / true |
| requested / reached real sim duration | 7200.0 / 7200.0 s |
| requested / completed real ticks | 72000 / 72000 |
| hard guard | 0 |
| deadlock ticks / recovery | 0 / 0 |
| reciprocal STOP cycles | 0 |
| transient wedge episodes (`wait>25 s`) | 10，均自行解除 |
| FAR decisions / FAR->NOMINAL | 17 / 17 |
| MID decisions / MID->YIELD | 7 / 6（另 1 次直接 emergency STOP） |
| NEAR decisions / NEAR->CREEP | 2 / 1（另 1 次直接 emergency STOP） |
| NOMINAL->CREEP | 1 |
| emergency NOMINAL/YIELD/CREEP->STOP | 2 / 0 / 0 |
| selected rollout CLEAR | 5 |
| conflicts delayed by YIELD / CREEP | 2 / 1 |
| later rolling replan became conflict-free | 11 |
| ordinary ConflictReservation creates | 0 |
| special creates：A1 / inside / braking | 137 / 32 / 2 |
| terminal / deadlock / multi / other creates | 0 / 0 / 0 / 0 |
| reservation create / update / delete / hold | 171 / 4 / 171 / 17363 |
| V0 tasks / max wait | 86 / 34.2 s |
| V1 tasks / max wait | 95 / 29.3 s |

每车动作时间：

| 车辆 | NOMINAL | YIELD | CREEP | STOP | active |
|---|---:|---:|---:|---:|---:|
| V0 | 5501.4 s | 2.0 s | 68.2 s | 785.6 s | 6357.2 s |
| V1 | 5432.8 s | 9.4 s | 62.4 s | 762.5 s | 6267.1 s |

10 次 wedge episode 是 `current_max_wait>25 s` 的诊断计数，不等同于 deadlock；最长等待为 34.2/29.3 s，之后清零并恢复任务，最终任务总数 181、deadlock ticks=0、reciprocal STOP=0。因此没有稳定楔死或新增安全回归。

正式日志真实出现连续周期：

```text
plan 2981: first_t=13.900 FAR -> NOMINAL/NOMINAL, reservation=not_created
plan 2982: first_t=11.850 FAR -> NOMINAL/NOMINAL, reservation=not_created
plan 2983: first_t= 9.850 MID -> NOMINAL/YIELD, rollout=CLEAR
plan 2984: first_t= 8.950 MID -> NOMINAL/YIELD,
           rollout conflict moved to 14.400 (+5.450 s), accepted=true,
           reservation=not_created
```

这同时证明：远期冲突不提前降速；下一个真实 rolling period 会重新判断；YIELD rollout 即使未 clear 仍被接受且不会继续搜索 CREEP/创建普通 reservation。

与阶段 2.3 对照：总任务均为 181，hard guard 和 deadlock 均为 0；任务分布从 94/87 变为 86/95，最大等待从 34.2/33.6 s 变为 34.2/29.3 s。单次固定 seed 只能证明本验收场景未降低总吞吐和安全性，不能外推为普遍性能提升。

## 9. EVALUATED 与 EXECUTED 口径

EVALUATED 包含 sandbox 内的诊断性状态，正式汇总为 baseline conflicts=744、FAR/MID/NEAR=17/7/2、emergency=2、YIELD evaluations/clear/delayed=6/3/2、CREEP=1/0/1、ordinary create=0。

EXECUTED 只在计划帧真实应用时统计，是正式业务口径。由于 sandbox 中可能先模拟创建、更新或释放，再被 snapshot/restore，EVALUATED 的 reservation create/update/delete=720/6/1288 不应与真实 EXECUTED 的 171/4/171混用。

## 10. 下一阶段迁移建议（仅设计，未实现）

- Crossing：由同步轨迹的 0.1 s OBB overlap 直接生成 `TimedConflictEvent`，不再要求预生成 `ConflictZone` 才能定义 crossing。
- Opposing shared path：静态 `SharedSegment` 只描述空间边界；每轮预测生成双方 `OccupancyInterval[t_enter,t_exit]`，以时间窗重叠提供 winner/loser 和 urgency。
- Same-direction following：独立计算 leader/follower、纵向间距、相对速度和安全制动约束，不再借用 crossing reservation。
- Physical commitment：仅在车辆已经进入、越过安全停车边界或下一执行窗必然进入且换 owner 不安全时建立短期 `LocalMotionCommitment`；清出 interaction 后立即释放。

上述交互最终都向现有动态速度选择器提供 `winner/loser + first_t/urgency + safety condition`，统一输出 NOMINAL/YIELD/CREEP/STOP。普通 `ConflictReservation` 可按 crossing、following、shared segment、physical commitment 的替代顺序逐类退出，而不是一次性删除。

## 11. 十二项明确回答

1. FAR 保持 NOMINAL：**是**。
2. MID 同周期只选 YIELD、不继续搜索 CREEP：**是**。
3. NEAR 允许 NOMINAL 直接到 CREEP：**是**。
4. 紧急条件允许直接 STOP：**是**。
5. 15 s 未完全 clear 仍接受本周期动作：**是**。
6. 同周期未来 sandbox 保持普通周期动作：**是**；安全/A1/已有 reservation 仍可覆盖。
7. 下一真实 rolling period 重新判断 band 和 winner：**是**。
8. 普通 reservation 仍用于：A1、terminal、already-inside、braking safety、deadlock special 和三车以上 legacy；已有 reservation 可 hold/update。
9. 普通冲突还会仅因“15 s 不完全 clear”进入 reservation：**否**，focused test 与正式 `ordinary_create=0` 双重验证。
10. 120 min 是否真实推进 7200 s：**是**，72000 个真实 tick；sandbox 时间未累计。
11. 是否发生 hard guard、deadlock 或新安全回归：hard guard=0、deadlock=0、reciprocal STOP=0；有 10 次有限等待 wedge 诊断，均自行解除，不是稳定回归。
12. 动态调速是否推迟或逐步消除冲突：**是**；正式执行中 YIELD/CREEP 延后 2/1 次，后续 rolling replan 变为无普通冲突 11 次。

## 12. 实验产物

原始大日志归档在仓库外同级目录 `Testing/EXP007`，避免污染本阶段 Git diff：

| 文件 | bytes | SHA-256 |
|---|---:|---|
| `rolling_action_120_console.log` | 1,945,368 | `F8C49DFE5D76882A8E4EF0A5A8B6A8F13FF8494197C76552BA83A1967FEA290F` |
| `rolling_action_120_coord.log` | 28,868,053 | `7CBD22E0B22AA2CB0DAB13C65FBE2D5F99C0997D4D9EAEF182FA06CED99BB3D6` |

## 13. 最终判定

本阶段是对既有双车动态速度模块和阶段 2.3 rolling 时序的局部替换，没有大规模无关重构。代码、focused tests、完整 CTest 和正式 7200 s 主仿真时间回归均满足验收条件。

**PASS**
