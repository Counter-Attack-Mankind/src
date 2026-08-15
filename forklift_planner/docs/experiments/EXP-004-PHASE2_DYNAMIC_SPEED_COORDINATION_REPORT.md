# PHASE2_DYNAMIC_SPEED_COORDINATION_REPORT

实验编号：EXP-004

日期：2026-08-15

阶段一基线：`9a1e82a`

正式场景：2 vehicles，seed=2026，`reproducible_task_random=true`
最终结论：**PASS**

本报告只覆盖“双车普通 TimedConflict 的动态速度协调验证”。正式业务仍是 `B -> A1 -> B`；A1 ownership、following、冲突类型分类、多车图和 deprecated 死锁策略均不在本实验改造范围。

## 1. 阶段一基础接口如何复用

直接复用 `spatiotemporal_interaction` 的 `PotentialConflictZone`、`PredictedKinematicSample`、`TimedConflictEvent`、`PairInteractionResult` 和 `detectPairInteractionFromPredictions()`。候选动作没有实现第二套 OBB 或冲突检测器。

证据：

- `forklift_planner/include/forklift_planner/multi_vehicle/spatiotemporal_interaction.h`
- `forklift_planner/src/multi_vehicle/dynamic_speed_coordination.cpp::evaluateCandidate()`
- `forklift_planner/src/multi_vehicle/rule_engine.cpp::resolvePairwiseConflicts()`

## 2. predictor 如何 Action 参数化

阶段一的 NOMINAL-only predictor 改为统一接口：

```cpp
predictTrajectory(vehicle, map_param, config, target_action, horizon)
```

同一循环支持 STOP、CREEP、YIELD、NOMINAL；BOOST 仅因既有枚举而保持兼容，阶段二候选集合没有使用 BOOST。每次预测从输入车辆当前 `path_s/current_speed` 开始，并继续受 `max_accel`、`max_decel`、曲率速度上限、路径终点、真实 OBB、`conflict_margin` 和 `prediction_step` 约束，没有瞬时速度跳变。

## 3. NOMINAL 默认效率原则如何实现

`RuleEngine::decide()` 每轮仍先把 ACTIVE 车辆的 `requested_action` 重置为 NOMINAL；pair baseline 固定以 NOMINAL/NOMINAL 预测。只有 baseline 存在普通 timed conflict 且候选通过全时域复检时，才写入 YIELD 或 CREEP。候选失败时回到原 reservation/brake 链。

动态层只在 `vehicles.size()==2` 时启用，默认 8 车流程没有被扩展为阶段二策略。

## 4. candidate 在 15 s 内的持续语义

一次 `evaluateCandidate()` 为两车各生成一条完整 15 s 预测。所选目标动作在该次预测中保持不变，例如 NOMINAL/YIELD；运动速度仍按加减速度、曲率和终点逐步更新。

## 5. 为什么真实车辆不会 YIELD 整整 15 s

15 s 是反事实判断窗口，不是控制承诺窗口。`buildSimulationHorizonPlan()` 生成 150 个 0.1 s frame，但 `simulationPlanNeedsRefresh()` 只提交约 20 frame，即约 2 s；剩余预测随后废弃。

## 6. 下一 2 s 如何重新恢复 NOMINAL baseline

下一滚动周期从推进后的真实 `path_s/current_speed` 重新调用 NOMINAL/NOMINAL predictor。动态层不保存“继续让行”的控制偏好；`previous_dynamic_actions_` 只在一轮 `decide()` 开始时读取上一动作以输出恢复诊断，随后清空，不参与候选选择。

若动态约束消失，动态请求恢复 NOMINAL；后置的 `enforceForwardClearance()` 等安全规则仍可进一步收紧动作，这不是动态历史锁定。

## 7. candidate 搜索顺序

先复用既有 `priorityWinner()` 得到 preferred winner。winner 保持 NOMINAL，另一车严格按以下顺序试算：

1. YIELD
2. CREEP

首个全时域无冲突候选即选中。阶段二未测试反向让行组合，未使用 BOOST、STOP 候选搜索、连续优化或随机搜索；YIELD/CREEP 都失败后直接进入 legacy reservation/STOP 链。

## 8. candidate 成功/失败标准

成功的唯一标准是重新预测后 `TimedConflictEvent.valid == false`，且这一判断覆盖完整 prediction horizon。若 first_t 只从较早时刻推迟到较晚时刻，候选仍为失败，并继续尝试更强动作或 fallback。

## 9. counterfactual 无副作用证明

`evaluatePairSpeedCoordination()` 和内部 `evaluateCandidate()` 只接收 const 车辆、配置和 potential zones，只生成局部预测与结果对象，不调用 `RuleEngine::decide()`。

Focused test 重复执行候选评估后比较 live vehicle；另以含 sentinel reservation、token 和 `now` 的 `RuleEngine::SimSnapshot` 比较调用前后状态。两项均通过。

## 10. existing reservation 处理

`resolvePairwiseConflicts()` 在动态入口前先检查有效 existing reservation。存在时记录 `reason=existing_reservation fallback=legacy`，随后完整执行旧 owner/waiter、删除条件和 `brakeBefore()` 逻辑，不进入候选搜索。

新普通冲突只有在动态候选成功时跳过新 reservation；动态失败、特殊场景或安全余量不足均落回原链。

## 11. near fallback 条件

对拟让行车辆计算：

```text
safe_stop_s = conflict_entry_s - 0.01 m - safety_margin
available = safe_stop_s - path_s
required = current_speed^2 / (2 * max_decel)
           + current_speed * max(decision_dt, rolling_refresh_period)
```

当 `available <= required` 时判定制动余量不足，跳过动态试算并进入 legacy。该判断复用现有速度、减速度、安全余量和滚动刷新周期；没有修改任何配置阈值。

## 12. A1 隔离方式

以下任一条件成立即标记为 A1-related 并保持 legacy：departure cluster commitment、future A1 commitment、活动中的 `a1_departure_committed`、departure cluster owner 或 future A1 owner。terminal docking、deadlock breaker、任一车辆已进入该 pair 的任一 conflict zone 也不属于普通动态入口。

未修改 `FutureA1Commitment`、`FutureA1Admission`、DepartureCluster closure、A1 release、prepared A1→B path 或 A1 状态机函数。

## 13. focused tests

新增两个测试程序，共覆盖需求中的 13 项：

- `dynamic_speed_coordination_test`：YIELD-clear；YIELD 失败/CREEP-clear；全部失败；只推迟仍失败；低速 NOMINAL 加速；高速 YIELD 减速；曲率限速；终点夹紧；车辆输入无副作用；near/far 制动余量。
- `dynamic_speed_rule_engine_test`：RuleEngine persistent state 无副作用；existing reservation；A1 pair；near legacy fallback；执行 2 s 后新 NOMINAL baseline clear 且无动态历史锁定。

确定性几何样例：YIELD-clear baseline first_t=2.05 s；CREEP 样例 baseline first_t=1.30 s，YIELD 后 first_t=1.80 s 仍判失败，随后 CREEP clear。

## 14. 构建/测试结果

构建命令：

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON --pkg forklift_planner
```

结果：通过。

测试命令：

```bash
cd build && ctest --output-on-failure
```

结果：5/5 通过：`conflict_zone_closure_test`、`future_a1_policy_test`、`spatiotemporal_interaction_test`、`dynamic_speed_coordination_test`、`dynamic_speed_rule_engine_test`。

RViz 未单独人工运行；本轮采用固定种子完整轨迹 batch 与同步 OBB hard guard 作为等价轨迹回归证据。当前仓库没有完整自动化测试体系，因此以上不能表述为完整测试覆盖。

## 15. 2 车 seed=2026 120 min 实验

命令模板：

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=120 coord_log_file:=/tmp/phase2_120_fixed_coord.log \
  debug_log_dir:=/tmp/phase2_120_fixed_debug
```

固定参数来自 launch 覆盖：vehicle_count=2、seed=2026、reproducible=true；其他规划与安全参数继续加载正式 YAML，未修改参数值。

| 时长 | hard guard | deadlock ticks | recovery | tasks V0/V1 | max wait V0/V1 (s) | YIELD clear | CREEP clear |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 min | 0 | 0 | 0 | 8 / 6 | 15.5 / 19.5 | 68 | 0 |
| 30 min | 0 | 0 | 0 | 24 / 22 | 34.2 / 29.3 | 184 | 72 |
| 120 min | 0 | 0 | 0 | 91 / 90 | 34.2 / 31.7 | 356 | 92 |

120 min 详细动态统计：baseline conflicts=1182，YIELD trials=455，YIELD-clear=356，CREEP trials=99，CREEP-clear=92，candidate all failed=7，near fallback=17，A1 fallback=645，nominal recovery=9，reservation create/delete=734/1281。

计数口径：RuleEngine 在正式真实推进和 rolling rollout 中的相同决策调用累计；动作时间只统计实际提交的 ACTIVE frame。

120 min coordination log：139318 行，27014655 bytes，SHA-256 `2c3d99dc15a3198755decc5c3dc87881386488b210eaa488880fe4f74e94a872`。正式结果和摘要已固化在本报告；`/tmp` 原始日志不是唯一实验记录。

## 16. 与 9a1e82a 对照

基线由 `git archive 9a1e82a` 建立隔离工作区，仅加入同版 action-time 诊断，不加入任何动态逻辑。其原始 coordination log SHA-256 为 `82ece52a29387698bd6033d39b0e605033cf4ed3a9d31497f47a525b9d795107`，与阶段二实施前已采集的 9a1e82a 正式日志哈希一致。

| 120 min 指标 | 9a1e82a | 阶段二 | 变化 |
|---|---:|---:|---:|
| hard guard | 0 | 0 | 0 |
| deadlock ticks / recovery | 0 / 0 | 0 / 0 | 0 |
| 最大连续等待 | 35.5 s | 34.2 s | -1.3 s |
| tasks V0/V1 | 91 / 90 | 91 / 90 | 0 / 0 |
| STOP 总时间 | 1492.2 s | 1484.0 s | -8.2 s (-0.55%) |
| reservation create | 895 | 734 | -161 (-18.0%) |
| reservation delete | 1455 | 1281 | -174 (-12.0%) |

reservation 对照使用相同 rollout+real 计数口径。吞吐量保持不变；本阶段证明闭环可行，不据此宣称普遍效率提升。

## 17. YIELD/CREEP 真实消冲突案例

固定场景中的真实 rollout 日志案例：

```text
[DYN-SPEED] pair=V0-V1 baseline=NOMINAL/NOMINAL
baseline_conflict=true baseline_first_t=13.550
try=NOMINAL/YIELD:CLEAR selected=NOMINAL/YIELD
reason=minimum_intervention_yield reservation=not_created
```

同一场景 30 min/120 min 统计还出现 YIELD 仍冲突、CREEP 清除：CREEP-clear 分别为 72/92。动态成功分支在写入动作后直接 `continue`，没有创建该次新普通 reservation；hard guard 为 0。对照基线对此类新普通 timed conflict 只能进入 reservation/brake 链，因此满足“旧 reservation/STOP，新差速 clear 且无新普通 reservation”的验证目标。

## 18. 恢复 NOMINAL 真实案例

紧随上述真实 YIELD 事件的日志：

```text
[DYN-SPEED] pair=V0-V1 baseline=NOMINAL/NOMINAL
baseline_conflict=false previous_action=YIELD
selected=NOMINAL/NOMINAL reason=rolling_recovery
```

120 min 共记录 9 次动态恢复。focused test 还显式推进 2 s 前缀后从新真实速度建立 NOMINAL baseline，并证明无动态 reason/reservation 残留。

## 19. hard guard/deadlock/wait/task 与动作指标

120 min：hard guard=0，collision=0，deadlock ticks=0，recovery=0，V0/V1 tasks=91/90，最大连续等待=34.2/31.7 s。

| 车辆 | ACTIVE (s) | STOP | YIELD | CREEP | NOMINAL | NOMINAL 占比 | transitions |
|---|---:|---:|---:|---:|---:|---:|---:|
| V0 | 6303.3 | 773.7 | 7.5 | 69.0 | 5453.1 | 86.51% | 501 |
| V1 | 6318.0 | 710.3 | 14.4 | 62.1 | 5531.2 | 87.55% | 481 |

BOOST=0。动作切换总数为 982，基线为 962，增量 20（2.1%）；结合仅 9 次 rolling recovery、结束时两车均 NOMINAL、无长期等待和 86% 以上 NOMINAL 占比，没有观察到动态速度反复振荡或长期 YIELD 锁定。这里的 CREEP 时间包含既有规则与动态规则的实际动作，不能仅由总时长拆分来源。

## 20. 发现但未处理的问题

- 初版 120 min 在约 48 min 后出现稳定双向等待。根因不是旧 deadlock 策略，而是普通动态入口只检查第一 timed event 对应 zone，未检查同一路径对的其他 conflict zones；两车可能分别已进入不同 zone，却错误跳过 reservation。最终修正为：任一车辆进入该 pair 的任一 potential zone 就保持 legacy。修正后 120 min deadlock ticks=0。
- 日志中的 A1 EXIT INTRUSION 为既有 `DIAG_ONLY` 诊断，本阶段未改变其策略。
- following 语义、interaction 类型分类、existing reservation 历史生命周期、slot priority、多车 conflict graph：Observed but out of scope。
- 当前统计同时包含 rollout 和 real 的 RuleEngine 决策次数；若后续需要生产 KPI，应增加明确的 source 维度，而不是改变本阶段控制逻辑。

## 21. Git diff 摘要

主要变更：

- 参数化阶段一 predictor；更新原阶段一调用和测试。
- 新增纯 `dynamic_speed_coordination` 结果对象、YIELD→CREEP 搜索和 near 判断。
- 在 `RuleEngine::resolvePairwiseConflicts()` 的“新普通冲突、创建 reservation 之前”接入动态层。
- 新增 `[DYN-SPEED]`、候选/恢复/reservation 统计和 batch action-time 统计。
- 新增 2 个测试程序和固定 2 车实验 launch。
- 未修改 YAML、安全阈值、消息接口、路径生成器、A1 状态机、hard guard、`enforceForwardClearance()`、`brakeBefore()` 或 deadlock 实现。

用户原有未跟踪文件 `11.md` 未修改、未纳入本阶段提交。

## 22. 验收结论

**PASS**

统一 predictor、真实状态起点、物理约束、每轮 NOMINAL baseline、纯 counterfactual、严格全时域 clear、existing reservation/A1/near fallback、安全叠加和 13 项 focused tests 均已落实。最终 2 车 seed=2026 的 120 仿真分钟 hard guard=0、无稳定双向死锁，并真实出现 YIELD-clear、CREEP-clear、reservation-free 动态成功和下一滚动周期 NOMINAL recovery。

自检答案：**是**。双车普通冲突中，车辆默认追求 NOMINAL；仅在预测到未来时空冲突时施加最小必要 YIELD/CREEP；候选必须经完整 15 s 重新预测确认冲突消失；约 2 s 后从真实状态重新建立 NOMINAL baseline，不形成新的长期速度预约。
