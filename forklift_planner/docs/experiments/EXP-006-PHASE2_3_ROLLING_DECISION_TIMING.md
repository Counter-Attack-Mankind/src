# EXP-006：阶段 2.3 滚动决策时序纠正

## 1. 结论

**PASS**

阶段 2.3 已实现以下时序语义：

- 每个新 rolling period 从当前真实 `path_s/current_speed/pose/mission state` 出发，使用完整 15 s horizon 做一次普通双车动态协调；
- 本周期约 2 s 内不再因 sandbox 的 0.1 s future state 变化重新执行普通 TimedConflict、FAR/MID/NEAR、priority winner 或 YIELD/CREEP candidate selection；
- 车辆仍每 0.1 s 按既有加减速、曲率限速和路径终点约束推进；
- 2 s 后从已执行状态重新建立新的完整 15 s 预测并重新决策；
- following、existing `ConflictReservation`、`FutureA1Commitment`、`DepartureClusterCommitment`、A1 admission/departure、target occupancy、forward clearance 和 hard guard 仍按原调用顺序运行，可覆盖普通动态协调目标；
- 正式统计已拆分为 `EVALUATED`（sandbox/候选诊断）与 `EXECUTED`（实际接受并执行的 rolling decision）。

正式双车 `seed=2026` 的 10/30/120 min 回归均为 hard guard=0、deadlock ticks/recovery=0/0。120 min 共完成 181 个任务，与阶段 2.2 的 181 个持平。

## 2. 基线、环境和范围

- 修改前 Git 基线：`e7f472e`（阶段 2.2）；阶段二对照 commit：`81cfbfd`。
- 仿真环境：WSL distro `Ubuntu-20.04-ros`，ROS Noetic（运行日志为 rosversion 1.17.4）。
- 场景：`vehicle_count=2`、`random_seed=2026`、`reproducible_task_random=true`、`real_mode=false`。
- 配置保持：`prediction_horizon=15.0 s`、`prediction_step=0.05 s`、`rolling_refresh_period=2.0 s`、执行步长 `0.1 s`。
- 没有修改 FAR/MID/NEAR 阈值、车辆速度/加减速、安全阈值、路径生成、任务分配、A1 ownership、普通 reservation 生命周期、deadlock、BOOST 或实车接口。

阶段开始前工作区已有且未纳入本阶段的改动：

- `forklift_planner/src/multi_vehicle/spatiotemporal_interaction.cpp` 的空白差异；
- `forklift_planner/src/test.cpp` 删除；
- 未跟踪文件 `11.md`。

## 3. 原问题和修正后的调用语义

修改前，`rollWorldModel()` 在约 150 个 future step 中逐步调用完整 `RuleEngine::decide()`。前 20 个 `SimPlanFrame` 又会被仿真执行，因此同一 plan 可以在 frame 0 判 FAR/NOMINAL，却在 frame 2 因剩余 `first_t` 进入 MID 而提前改成 YIELD。

修改后仍生成 150 帧并保留现有 `SimPlanFrame`：

1. frame 0 使用周期开始的真实状态和完整 15 s horizon，执行一次完整普通动态协调；
2. 仅记录该次普通动态协调产生的周期目标（通常为 YIELD/CREEP；FAR 为 NOMINAL）；
3. frame 1–149 继续推进世界状态并运行必要状态维护/安全链，但普通 pairwise 再仲裁入口被跳过；
4. 高优先级规则不是周期目标的一部分，仍逐帧重新求值，因而可以及时收紧为 STOP，也可在条件解除后恢复到本周期普通目标；
5. 只有前 20 帧用于实际约 2 s 执行，其余 130 帧作为当前策略下的风险预测；
6. 下一个 rolling period 从前 20 帧实际执行后的状态重新生成完整 15 s 预测。

`RuleEngine::decide()` 只增加了一个轻量复用模式和本周期普通目标参数；没有新增第二套 RuleEngine、world model、状态机或生产 `.cpp/.h` 文件。路径 `path_gen` 变化时旧周期目标不会应用到新路径。

## 4. 完整 15 s candidate validation

普通协调仅在 frame 0 更新，不等于把 candidate horizon 缩为 2 s。frame 0 仍把完整 15 s 传给现有：

- `evaluatePairSpeedCoordination()`；
- `evaluateCandidate()`；
- `predictTrajectory()`；
- `detectPairInteractionFromPredictions()`。

新增 focused test 找到一个 YIELD 后剩余冲突位于 `t=2.30 s` 的确定性场景。该候选仍被判为冲突而非 CLEAR，证明“超过 2 s”不会被误认为“完整 15 s 内安全”。

future safety/world-state 推进使用当前 plan 的剩余窗口，避免内部状态维护越过同一 plan 的绝对 15 s 终点；每个新 rolling period 的普通协调仍重新获得完整 15 s horizon。

## 5. Focused tests 与 CTest

新增 `test/rolling_decision_timing_test.cpp`，覆盖：

- FAR：周期开始 `first_t=10.05 s`，sandbox 状态随后进入 MID，20 个执行帧普通目标仍为 NOMINAL；
- refresh：2 s 后从已推进的 `path_s/current_speed` 重新决策，`first_t=8.05 s`，新周期才采用 NOMINAL/YIELD；
- MID：周期开始选择 YIELD/CREEP 后，本周期内不因 future re-evaluation 提前恢复 NOMINAL 或改选普通动作；
- 15 s：`t=2.30 s` 的剩余候选冲突不会被 2 s 执行窗口掩盖；
- 高优先级覆盖：已有 reservation 可在复用普通 NOMINAL 时收紧为 STOP，且 reservation 仍保留。

测试输出关键证据：

```text
[ROLLING-FAR] start_first_t=10.05 future_mid_first_t=8.15 frames_nominal=20
[ROLLING-REFRESH] first_t=8.05 new_target=NOMINAL/YIELD
[FULL-HORIZON-CANDIDATE] remaining_conflict_t=2.3
rolling_decision_timing_test: PASS
```

WSL 中执行：

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCATKIN_ENABLE_TESTING=ON --pkg forklift_planner
cd build
ctest --output-on-failure
```

结果：7/7 tests passed，包括阶段 2.3 新测试和阶段一、阶段二、阶段 2.2 的全部既有测试。

## 6. 正式日志中的时序证据

修改前专项核查已确认：

```text
plan=538 frame=0 first_t=10.15 FAR -> NOMINAL
same plan frame=2 first_t=9.95 MID -> YIELD
```

修改后 120 min 正式日志真实出现连续 rolling plans：

```text
plan=2973 frame=0 first_t=13.900 FAR
selected=NOMINAL/NOMINAL reason=far_deferred
[ROLLING-DECISION] band=FAR targets=NOMINAL/NOMINAL

plan=2974 frame=0 first_t=11.850 FAR
selected=NOMINAL/NOMINAL reason=far_deferred
[ROLLING-DECISION] band=FAR targets=NOMINAL/NOMINAL

plan=2975 frame=0 first_t=9.850 MID
try=YIELD/NOMINAL:CLEAR
selected=YIELD/NOMINAL reason=minimum_intervention_yield
[ROLLING-DECISION] band=MID targets=YIELD/NOMINAL
```

对 plan 2973、2974、2975 分别统计执行窗口 frame 0–19，三者均只有 frame 0 一条普通 `[DYN-SPEED]` 决策记录；不存在同一 2 s 执行窗口内的第二次普通 FAR/MID/NEAR 决策。后续 15 s 风险段仍可能触发 existing reservation/A1/inside-zone 等特殊安全链，这是允许的高优先级覆盖，不是普通动态协调重选。

## 7. 10/30/120 min 回归

执行命令模板：

```bash
source /opt/ros/noetic/setup.bash
source /mnt/d/desktop/叉车/devel/setup.bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=<10|30|120> \
  coord_log_file:=/mnt/d/desktop/叉车/Testing/EXP006/phase2_3_<minutes>_coord.log \
  debug_log_dir:=/mnt/d/desktop/叉车/Testing/EXP006/phase2_3_<minutes>_debug
```

### 7.1 安全、吞吐和等待

| 仿真时长 | hard guard | deadlock ticks/recovery | tasks V0/V1 | 总任务 | max wait V0/V1 |
|---|---:|---:|---:|---:|---:|
| 10 min | 0 | 0 / 0 | 8 / 6 | 14 | 15.5 / 19.5 s |
| 30 min | 0 | 0 / 0 | 24 / 22 | 46 | 34.2 / 29.3 s |
| 120 min | 0 | 0 / 0 | 94 / 87 | 181 | 34.2 / 33.6 s |

### 7.2 实际动作时间

| 仿真时长 | NOMINAL | YIELD | CREEP | STOP |
|---|---:|---:|---:|---:|
| 10 min | 903.6 s | 0.0 s | 10.7 s | 135.6 s |
| 30 min | 2647.7 s | 0.0 s | 37.0 s | 459.6 s |
| 120 min | 10971.8 s | 5.4 s | 128.1 s | 1516.0 s |

### 7.3 EXECUTED：真实 rolling decision

`target_*` 是每辆车在 rolling period 首帧最终实际接受的目标；它包含普通协调及随后高优先级安全覆盖后的结果。

| 仿真时长 | FAR | MID | NEAR | legacy | target NOMINAL | target YIELD | target CREEP | target STOP | reservation create/delete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 min | 2 | 0 | 0 | 0 | 495 | 0 | 0 | 153 | 13 / 13 |
| 30 min | 7 | 1 | 1 | 1 | 1442 | 0 | 1 | 487 | 47 / 47 |
| 120 min | 19 | 7 | 2 | 4 | 5959 | 3 | 2 | 1760 | 169 / 169 |

### 7.4 EVALUATED：sandbox/候选诊断

此口径保留用于诊断，不代表实际车辆经历次数。例如 120 min 为：baseline conflicts=754、FAR deferred=19、MID intervention=5、NEAR intervention=0、YIELD trials/clear=7/3、CREEP trials/clear=4/2、candidate search failed=2、evaluated reservation create/delete=730/1286。正式业务统计使用上一节的 `EXECUTED` 口径。

## 8. 与阶段 2.2 对照

| 120 min 指标 | 阶段 2.2 | 阶段 2.3 | 变化 |
|---|---:|---:|---:|
| hard guard | 0 | 0 | 0 |
| deadlock ticks/recovery | 0 / 0 | 0 / 0 | 0 |
| tasks V0/V1 | 91 / 90 | 94 / 87 | +3 / -3 |
| 总任务 | 181 | 181 | 0 |
| max wait V0/V1 | 34.2 / 31.7 s | 34.2 / 33.6 s | 0 / +1.9 s |
| NOMINAL | 10992.8 s | 10971.8 s | -21.0 s |
| YIELD | 8.0 s | 5.4 s | -2.6 s |
| CREEP | 130.7 s | 128.1 s | -2.6 s |
| STOP | 1489.8 s | 1516.0 s | +26.2 s |

系统总任务完成能力与阶段 2.2 持平，车辆间完成数重新分布；V1 最大等待增加 1.9 s，但为有限等待，未形成稳定 deadlock 或饥饿。阶段 2.2 尚未区分实际 reservation transition 与 rollout evaluation，因此旧的 742/1289 不能和阶段 2.3 的 EXECUTED 169/169直接等价比较；阶段 2.3 同口径 EVALUATED 为 730/1286。

## 9. 日志产物

日志保存在仓库同级 `Testing/EXP006`。协调日志校验如下：

| 日志 | bytes | SHA-256 |
|---|---:|---|
| `phase2_3_10_coord.log` | 2,100,368 | `639B71157FF2BEF6C8A8EC97B58E79A847D1EF46062195A8D22814303FC9FAD0` |
| `phase2_3_30_coord.log` | 7,259,702 | `DB2B566B6B33F8FD3C2E7918FEA70CA1C2C7443A4C5EA2531E7DCA4F4C7E1C9C` |
| `phase2_3_120_coord.log` | 27,730,248 | `6EFC754D16E69B9A5E767FCEE6434B69CA1165C4F80EAAFAA9834DB0014725FD` |

对应 console 和 rosout 日志也已归档，rosout 保存 batch 汇总，coord log 保存逐 plan/frame 协调证据。

## 10. Git diff 摘要

阶段 2.3 是小范围、可整体回退的增量：

- `rule_engine.h/.cpp`：为现有 `decide()` 增加普通协调复用入口；普通 pairwise 只在周期首帧选择一次，原 following 和高优先级链继续运行；
- `multi_vehicle_patrol_node.cpp`：150 帧继续生成，只在首帧保存普通 rolling decision；新增轻量 EXECUTED 统计与 `[ROLLING-DECISION]` 日志；旧动态指标改名为 EVALUATED；
- `CMakeLists.txt` 和 `rolling_decision_timing_test.cpp`：新增一个 focused test；
- 本报告。

没有新增生产 `.h/.cpp` 文件，没有复制协调器或 world model，也没有无关重构。

## 11. 四项最终回答

1. **是否真正实现 15 s 看未来、2 s 更新决策、0.1 s 执行？** 是。周期首帧完整预测/决策，20 个 0.1 s 帧执行，随后从实际状态重新生成完整 15 s。
2. **同一个 2 s 周期内是否已不存在 future 0.1 s 普通 FAR/MID/NEAR 重复决策？** 是。focused test 和正式 plan 2973–2975 均验证执行窗口内只有 frame 0 一次普通决策。
3. **15 s 完整预测和现有高优先级安全链是否正常？** 是。2.30 s 剩余冲突仍被完整 horizon 捕获，existing reservation STOP 覆盖测试通过；全套 CTest、安全回归和 A1 主流程通过。
4. **统计是否代表真实执行而不是 sandbox 试算？** 是。`BATCH_DYN_SPEED_EXECUTED` 单独统计实际 rolling period/目标/reservation transition；原试算指标明确命名为 `BATCH_DYN_SPEED_EVALUATED`。

**PASS**
