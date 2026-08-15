# EXP-005：阶段 2.2 预测执行一致性与动态干预时机

实验日期：2026-08-15

阶段二基线：`81cfbfd`

场景：`vehicle_count=2`、`random_seed=2026`、`reproducible_task_random=true`、仿真模式

结论：**PASS**

本实验只验证预测执行一致性，以及普通双车 `TimedConflict` 的 FAR/MID/NEAR 干预时机。未修改 A1 admission/departure、`FutureA1Commitment`、`DepartureClusterCommitment`、既有 `ConflictReservation` 生命周期、following、interaction 分类、hard guard、forward clearance、deadlock、路径生成、任务分配、BOOST 协调和近端 ownership 语义。

## 1. 预测模型与仿真执行模型核验

真实调用链为：

- 预测：`predictTrajectory()`，按 `prediction_step=0.05 s` 从输入车辆当前 `path_s/current_speed` 起步；
- 执行：`MultiVehiclePatrolNode::advanceVehicles()`，仿真周期约 `0.1 s`；
- 两者都按目标 `VehicleAction`、`max_accel/max_decel`、曲率限速和路径终点逐步更新速度及位置；
- `prediction_execution_consistency_test` 使用独立的测试侧执行器复现当前 `curvatureSpeed()`、`limitedSpeed()` 和 `advanceVehicles()` 公式，避免直接调用 predictor 自证一致。

相同直线路径、初始 `path_s/current_speed` 和 Action 推进 2 s 的结果：

| Action | speed 差 (m/s) | path_s 差 (m) | pose 位置差 (m) | yaw 差 (rad) |
|---|---:|---:|---:|---:|
| NOMINAL | 0 | 0.00500 | 0.00500 | 0 |
| YIELD | 0 | 0.00225 | 0.00225 | 0 |
| CREEP | 0 | 0.00375 | 0.00375 | 0 |
| STOP | 0 | 0.00475 | 0.00475 | 0 |
| BOOST | 0 | 0.00100 | 0.00100 | 0 |

确定性临界交叉扫描覆盖 4008 个 predictor 判 CLEAR 的候选。先按真实仿真 `0.1 s` 步长执行 2 s，再从执行后的真实状态预测剩余 13 s，`CLEAR -> dangerous conflict` 翻转数为 0。

结论：在本 focused test 的动作、初速和交叉临界扫描覆盖内，最大位置离散差为 5 mm，未改变 `TimedConflict` 的 CLEAR/冲突安全结论。因此保留现有 `0.05/0.1 s` 设置，没有为形式统一修改生产运动模型。该结论不等同于所有未来地图几何的数学证明；若车辆动力学或曲率模型改变，应重新执行本测试。

## 2. 当前 VehicleAction 目标速度和转换时间

实际 YAML 与代码参数为：`nominal_speed=0.20 m/s`、`yield_ratio=0.50`、`creep_ratio=0.25`、`max_accel=0.20 m/s²`、`max_decel=0.30 m/s²`、`boost_ratio=1.20`、`max_speed=0.26 m/s`、`enable_boost=true`。

目标速度由 `actionTargetSpeed()`/执行侧等价逻辑确定。下表忽略曲率限速和路径终点，仅给出理论匀加减速时间：

| Action | 当前实际目标速度 | 从静止加速 | 从 NOMINAL=0.20 转换 |
|---|---:|---:|---:|
| STOP | 0.00 m/s | 0.00 s | 0.667 s（减速） |
| CREEP | 0.05 m/s | 0.25 s | 0.500 s（减速） |
| YIELD | 0.10 m/s | 0.50 s | 0.333 s（减速） |
| NOMINAL | 0.20 m/s | 1.00 s | 0.000 s |
| BOOST | `min(0.26, 0.20*1.20)=0.24 m/s` | 1.20 s | 0.200 s（加速） |

BOOST 仍不进入阶段 2/2.2 动态候选搜索；上表只核对现有 Action 的真实目标语义。若 `enable_boost=false`，BOOST 的代码目标退回 NOMINAL。

`VehicleAction` 是目标速度模式，不是瞬时速度。focused test 另以 `max_decel=0.02 m/s²` 构造跨周期 YIELD：`t=2 s` 速度为 `0.16 m/s`，尚未到 `0.10 m/s` 目标；保持同一目标继续执行至 `t=4 s` 后速度为 `0.12 m/s`。这证明 `rolling_refresh_period=2 s` 只决定重新规划时机，不重置 `current_speed`，也不要求动作转换在一个 rolling period 内完成。

## 3. FAR/MID/NEAR 实现

新增两个配置项，均从 `forklift_planner/multi_vehicle` ROS 参数空间读取并校验：

```yaml
dynamic_speed_far_threshold: 10.0
dynamic_speed_near_threshold: 5.0
```

边界定义为：

- FAR：`first_t >= 10.0 s`；普通冲突保持当前 NOMINAL，不试算 YIELD/CREEP、不创建新 reservation，等待下一 rolling refresh；
- MID：`5.0 s <= first_t < 10.0 s`；复用阶段二的 winner NOMINAL、loser YIELD→CREEP 搜索；
- NEAR：`first_t < 5.0 s`；仍先复用 YIELD→CREEP，完整 horizon 无法 clear 或制动余量不足时立即进入既有 legacy 安全链。

每个 MID/NEAR 候选继续通过 `evaluatePairSpeedCoordination()` → `evaluateCandidate()` → `predictTrajectory()` → `detectPairInteractionFromPredictions()` 完整重预测、完整重检测。没有新增第二套协调器，也没有改变普通 reservation 的长期 owner 机制。

FAR 只作用于阶段二定义的“新普通冲突”。已有 reservation、A1 相关 pair、terminal docking、deadlock breaker 或车辆已进入任一 pair conflict zone 时，仍沿既有 legacy 链处理，避免削弱旧安全语义。

## 4. Focused tests

构建命令（Ubuntu 20.04 / ROS Noetic WSL）：

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON --pkg forklift_planner
cd build && ctest --output-on-failure
```

结果：构建通过，CTest **6/6** 通过：

- `conflict_zone_closure_test`
- `future_a1_policy_test`
- `spatiotemporal_interaction_test`
- `dynamic_speed_coordination_test`
- `dynamic_speed_rule_engine_test`
- `prediction_execution_consistency_test`

新增/扩展覆盖：

- 预测与执行 2 s 的 speed/path_s/pose 差异；
- 4008 个 predictor-CLEAR 候选的 2 s 执行后安全语义复检；
- 跨 2 s rolling period 的连续减速；
- `10`、`10±epsilon`、`5`、`5±epsilon` 精确边界；
- 约 13 s FAR 保持 NOMINAL 且不建 reservation；
- 约 8 s MID 使用阶段二 YIELD/CREEP；
- 约 3 s NEAR 动态成功及 NEAR 无法安全消除时的 legacy fallback；
- existing reservation、A1 pair 和 RuleEngine counterfactual 无副作用回归。

当前仓库仍缺少完整自动化测试体系，因此 6 项 CTest 是本变更范围的 focused/regression 证据，不表述为全项目完整覆盖。未单独进行人工 RViz 观看；固定种子 batch 使用完整轨迹推进、同步 OBB hard guard 和任务状态机作为等价轨迹回归证据。

## 5. 固定种子 10/30/120 分钟回归

命令模板：

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=120 \
  coord_log_file:=/mnt/d/desktop/叉车/Testing/EXP005/phase2_2_120_coord.log \
  debug_log_dir:=/mnt/d/desktop/叉车/Testing/EXP005/debug
```

| 时长 | hard guard | deadlock ticks / recovery | tasks V0/V1 | max wait V0/V1 (s) | FAR deferred | MID intervention | NEAR intervention | NEAR legacy fallback |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 min | 0 | 0 / 0 | 8 / 6 | 15.5 / 19.5 | 52 | 122 | 0 | 0 |
| 30 min | 0 | 0 / 0 | 24 / 22 | 34.2 / 29.3 | 144 | 240 | 139 | 8 |
| 120 min | 0 | 0 / 0 | 91 / 90 | 34.2 / 31.7 | 284 | 365 | 157 | 23 |

120 分钟实际动作时间：

| 车辆 | ACTIVE (s) | STOP | CREEP | YIELD | NOMINAL | BOOST | transitions |
|---|---:|---:|---:|---:|---:|---:|---:|
| V0 | 6303.3 | 776.8 | 66.5 | 2.6 | 5457.4 | 0 | 507 |
| V1 | 6318.0 | 713.0 | 64.2 | 5.4 | 5535.4 | 0 | 483 |
| 合计 | 12621.3 | 1489.8 | 130.7 | 8.0 | 10992.8 | 0 | 990 |

120 分钟动态统计：baseline conflicts=1548、YIELD trials/clear=529/332、CREEP trials/clear=197/190、candidate search failed=7、原有 near-fallback 计数=17、A1 fallback=645、existing reservation=94667、nominal recovery=9、reservation create/delete=742/1289。

协调日志持久化在仓库同级 `Testing/EXP005`：

| 日志 | bytes | SHA-256 |
|---|---:|---|
| `phase2_2_10_coord.log` | 2,092,001 | `CC9A8B5E68441259A092329B341ABCF3A38D631038F6040EFC63416A99899A91` |
| `phase2_2_30_coord.log` | 7,257,407 | `EB858142DBD611D60C229497A85BAFD33E92616C1D72096EB593E975C6FEC655` |
| `phase2_2_120_coord.log` | 27,107,699 | `B40B28AC09E40BC69F441DD6BD8E3080D7985F74151B787798200EFC821896E4` |

## 6. 关键业务验收时间线

120 分钟日志中同一 pair 的连续 rolling plan 首帧真实出现：

```text
[plan=2973] [frame=0] baseline_first_t=13.900 band=FAR
selected=NOMINAL/NOMINAL reason=far_deferred reservation=not_created

[plan=2974] [frame=0] baseline_first_t=11.850 band=FAR
selected=NOMINAL/NOMINAL reason=far_deferred reservation=not_created

[plan=2975] [frame=0] baseline_first_t=9.900 band=MID
try=YIELD/NOMINAL:CLEAR selected=YIELD/NOMINAL
reason=minimum_intervention_yield reservation=not_created
```

相邻 plan 起点间隔约 2 s。这直接证明 13–14 s 远期普通冲突不再立即降速，下一周期仍远期则继续 NOMINAL，进入 MID 后才以完整重预测确认的 YIELD 开始干预。

## 7. 与阶段二 `81cfbfd` 对照

阶段二正式报告中的 120 分钟数据作为对照：

| 120 分钟指标 | 阶段二 | 阶段 2.2 | 变化 |
|---|---:|---:|---:|
| hard guard | 0 | 0 | 0 |
| deadlock ticks / recovery | 0 / 0 | 0 / 0 | 0 |
| tasks V0/V1 | 91 / 90 | 91 / 90 | 0 / 0 |
| max wait V0/V1 | 34.2 / 31.7 | 34.2 / 31.7 | 0 / 0 |
| NOMINAL 总时间 | 10984.3 s | 10992.8 s | +8.5 s |
| YIELD 总时间 | 21.9 s | 8.0 s | -13.9 s |
| CREEP 总时间 | 131.1 s | 130.7 s | -0.4 s |
| STOP 总时间 | 1484.0 s | 1489.8 s | +5.8 s（+0.39%） |
| reservation create/delete | 734 / 1281 | 742 / 1289 | +8 / +8 |
| transitions | 982 | 990 | +8 |

阶段 2.2 吞吐量、最大等待、hard guard 和 deadlock 与阶段二持平；NOMINAL 时间增加，YIELD 时间显著下降。reservation 创建比阶段二增加 8 次，但相对阶段一基线 895 次仍减少 153 次（-17.1%）。不据单次固定场景宣称普遍吞吐提升，只确认本验收场景未降低任务完成能力且未破坏阶段二安全性。

## 8. 范围核验与观察项

- 没有修改 `FutureA1Commitment`、`DepartureClusterCommitment`、A1 admission/departure 或 prepared exit；
- 没有修改 existing reservation 的创建后 owner/waiter 生命周期；
- 没有修改 `realAdvance()`、`realHardGuard()`、实车接口、forward clearance、braking、deadlock、任务与路径代码；
- 没有修改既有速度、加减速、预测 horizon、碰撞 margin 等安全阈值；只新增了动态干预分段阈值；
- 未观察到稳定 YIELD→NOMINAL→YIELD 振荡；按阶段约束只增加 band/counter 日志，没有引入 hysteresis 或 action hold；
- `near_legacy_fallback` 是普通 NEAR 动态协调失败/制动余量不足后进入旧链的计数，不替代原 `near_fallback` 诊断口径。

## 9. Git diff 摘要

阶段 2.2 是小范围、可整体回退的增量：

- 配置结构、ROS 参数读取和 YAML 增加 5 s/10 s 阈值；
- 既有动态速度模块增加一个轻量 band 分类函数；
- `RuleEngine::resolvePairwiseConflicts()` 在原阶段二入口增加 FAR defer、MID/NEAR 计数和日志，候选与 fallback 主链保持复用；
- batch 汇总增加 FAR/MID/NEAR 指标；
- 扩展两个既有 focused test，并新增一个 prediction/execution consistency 测试文件；
- 新增本实验报告。

用户在本阶段开始前已有的 `spatiotemporal_interaction.cpp` 空白差异、`forklift_planner/src/test.cpp` 删除和未跟踪 `11.md` 均未修改、未纳入本阶段成果。

## 10. 最终结论

**PASS**

预测与执行前 2 s 在 focused coverage 内最大位置差 5 mm，未发现安全语义翻转，故没有不必要地统一模型。所有 Action 目标速度和转换时间已按真实代码/YAML 核对，跨 rolling period 的连续速度转换已验证。FAR 冲突已不再立即降速，MID/NEAR 复用阶段二完整反事实预测和 legacy 安全回退。双车 seed=2026 的 10/30/120 分钟回归均 hard guard=0、无稳定 deadlock，120 分钟任务数与阶段二相同。
