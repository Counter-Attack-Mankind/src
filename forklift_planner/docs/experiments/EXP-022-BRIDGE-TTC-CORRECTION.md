# EXP-022：ordinary bridge-aware TTC correction

## 1. 基线与目的

- 日期：2026-08-25
- 当前 HEAD：`5ab5f48561da5317a032415d1fc66da3cd9f3c88`
- 工作区前提：HEAD 已包含 EXP-020 静态 `SharedSegmentCandidate`；本实验在未清理用户其他修改的前提下替换该方案。
- 目的：ordinary pair 仅在 15 s NOMINAL 同步 OBB 已有物理冲突后，局部回溯双方各自的 opposing/proximity 近端边界并只前移 yielding vehicle 的风险 TTC；不创建 bridge reservation、owner、token 或 occupancy state。
- 正式业务边界仍为 `B -> A1 -> B`；A1 service、FutureA1、departure cluster、slot departure、forward clearance 和 hard guard 不在本次算法修改范围。

## 2. 修改前调用链审查

1. `RuleEngine::resolvePairwiseConflicts()` 先用 `predictTrajectory()` 生成两车 15 s NOMINAL prediction，再由 `detectPairInteractionFromPredictions()` 做同步 OBB overlap。
2. 修改前 `TimedConflictEvent` 有 `first_t`，但没有保存首次同步冲突的双方 `path_s`；RuleEngine 曾从 prediction 再取对应 sample。
3. ordinary 动态分支先调用 `priorityWinner()`，再直接以 `event.first_t` 进入 `classifyDynamicInterventionBand()`、`evaluateTtcStopBoundary()` 和 `evaluatePairSpeedCoordination()`。
4. `computeSharedSegmentCandidates()` 只在固定路径 cache miss 时构建完整 `(sA,sB)` sample matrix、BFS component 和 qualification；消费者只有静态 cache、`[SHARED-SEGMENT-STATIC]` 日志、RViz marker 与专用测试，没有 ordinary 动作裁决调用者。
5. `SharedSegment`、`OccupancyInterval`、`predictOccupancyInterval()`、`detectSharedSegmentInteraction()` 在当前生产链没有调用者，只剩测试/显示兼容，因此可删除。
6. `PairInteractionType::OPPOSING` 仍被 slot-departure 分类、动态 marker/诊断和新 bridge 分类使用，不能删除。
7. `PotentialConflictZone`、`computeConflictZonesFull()`、`findConflictZones()`、conflict-zone cache 和 `conflict_reservations_` 仍被 A1 admission/service、departure cluster、forward clearance、launch/诊断调用，不能删除。ordinary 新事件不再为了控制而预扫 fixed zones。

## 3. 新调用链

```text
15 s NOMINAL prediction
  -> synchronous OBB overlap?
     -> no: ordinary clear；bridge 完全不运行
     -> yes: TimedConflictEvent(first_t, first_s_a, first_s_b)
        -> evaluateBridgeTtcCorrection(A/B independently)
        -> existing priorityWinner()
        -> yielding vehicle's corrected TTC
        -> existing FAR/MID/NEAR + TTC stop gate
        -> existing speed action selection and action merge
```

`evaluateBridgeTtcCorrection()` 是无状态函数。每侧以首次 collision 的另一侧 `s` 为 nearest cursor seed，限制在 seed 所属连续 traversal；沿本车 `s` 以 0.025 m 回溯，到 cusp 或 proximity/opposing 关系消失即停止。FORWARD 使用 pose heading，REVERSE 使用 `heading + pi`。TTC 由已生成的 NOMINAL prediction samples 插值得到，并执行 `min(original_ttc, boundary_ttc)`。

最近点实现是工程近似：从同步 collision seed 的 waypoint 开始，复用上一回溯点的 cursor，以相邻 waypoint hill-climb 后投影相邻 segment。它避免完整 `N_A x N_B`，但不宣称在自交、并行邻近分支等复杂路径上获得严格全局欧氏最优对应点。

## 4. 参数

| 参数 | 原值 | 新值 | 单位/范围 | 作用域 |
|---|---:|---:|---|---|
| `bridge_opposing_threshold` | 不存在 | -0.50 | direction dot，读取后限制 `[-1,0]` | ordinary baseline conflict 后的局部 relation |
| `bridge_backtrack_step` | 不存在 | 0.025 | m，最小 0.005 | self-path backtrack |

proximity 不增加独立魔数，使用 `vehicle_width + conflict_margin`；两者原值未改。旧 `shared_segment_min_span`、`shared_segment_strong_opposing_threshold`、`shared_segment_min_strong_ratio` 已删除。

## 5. 文件变化

新增：

- `include/forklift_planner/multi_vehicle/bridge_ttc_correction.h`
- `src/multi_vehicle/bridge_ttc_correction.cpp`
- `test/bridge_ttc_correction_test.cpp`

删除：

- `include/forklift_planner/multi_vehicle/shared_segment_geometry.h`
- `src/multi_vehicle/shared_segment_geometry.cpp`
- `test/shared_segment_geometry_test.cpp`

修改：

- `spatiotemporal_interaction.*`：首次物理冲突直接保存 `first_s_a/first_s_b`；删除无生产调用的 occupancy helper。
- `rule_engine.*`：接入 bridge correction、effective TTC、日志/指标；保留 A1 fixed-zone/reservation 链。
- `marker_publisher.*`：删除完整静态 candidate marker，只消费当前动态事件已有的双方 near-boundary 数据。
- `multi_vehicle_config.*`、`planner_param.yaml`：替换旧 qualification 参数。
- `dynamic_speed_coordination.cpp`：只接收已经修正的 `event.first_t`，不知道 bridge geometry。
- `multi_vehicle_patrol_node.cpp`：batch 汇总 bridge 计算量指标。
- `multi_vehicle_phase2_batch.launch`：新增可复现的 `start_slot_a/start_slot_b` 与 `node_output` 参数；默认值保持原启动行为。
- `CMakeLists.txt` 与相关测试：加入新模块/测试并清理旧 target/字段。

EXP-020/EXP-021 是历史实验记录，未篡改；其静态方案已被本记录明确取代，不再代表当前源码。

## 6. 验证

### 6.1 构建与 CTest：PASS

环境：WSL Ubuntu 20.04 + ROS Noetic，Release。

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
cd build
ctest --output-on-failure
```

结果：构建成功；8/8 PASS，总测试时间 1.68 s：

1. `bridge_ttc_correction_test`
2. `conflict_zone_closure_test`
3. `future_a1_policy_test`
4. `spatiotemporal_interaction_test`
5. `dynamic_speed_coordination_test`
6. `dynamic_speed_rule_engine_test`
7. `prediction_execution_consistency_test`
8. `rolling_decision_timing_test`

新增覆盖包括 baseline clear 完全 dormant、稳定 opposing、同向拒绝、远距离 opposing 拒绝、REVERSE motion heading、cusp 不跨越、单边 bridge、prediction-sample TTC，以及 RuleEngine 中无 ordinary reservation、yielding STOP 和 clear pair `bridge_checked=0`。

### 6.2 B42/B50、seed 2026、120 min：FAIL

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=120 vehicle_count:=2 random_seed:=2026 \
  start_slot_a:=42 start_slot_b:=50 node_output:=screen
```

完成 72000/72000 ticks、7200.0 s 仿真，但节点按安全验收返回失败：

- hard guard：1 次，首次 tick 2449（sim 244 s），V0-V1。
- deadlock 检出：13860 ticks；wedge episode：1。
- 最大等待：V0 6955.2 s，V1 6957.1 s。
- ordinary reservation create：evaluated/executed 均为 0。
- A1 reservation create：evaluated 42、executed 5。
- bridge：baseline conflicts 62，其中 ordinary bridge checked 20；crossing 5、opposing 15；A/B related 12/15；corrected pairs 13。
- bridge 计算量：平均每个 evaluated vehicle 回溯 15.18 samples，最大 50；nearest evaluations 总计 6451。没有完整 `N_A x N_B` matrix 或 BFS。

关键 bridge 日志示例：

```text
[BRIDGE-TTC] pair=V0-V1 original_ttc=6.750 collision_s_a=1.053 collision_s_b=1.053 bridge_a=true bridge_b=true boundary_s_a=0.503 boundary_s_b=0.353 corrected_ttc_a=3.651 corrected_ttc_b=2.598 priority_vehicle=V0 yielding_vehicle=V1 effective_ttc=2.598 band=NEAR selected=NOMINAL/CREEP backtrack_samples=24/30 nearest_evaluations=184/270
```

单边 relation 也实际出现：plan 5 为 `bridge_a=false bridge_b=true`。plan 101--108 在 A1 碰撞前已让 yielding V1 使用 `effective_ttc=0` 并 STOP；但实际 hard guard 位于 A1 特殊资源事件：V1 从 A1 激活 `A1 -> B61` 后停在 `s=0`，V0 已处于 A1 conflict zone 的 committed 区间，tick 2449 两车 overlap。之后 FutureA1 长期为 `service_owner_locked_active_cluster`。这说明完整回归未通过；其精确根因和修复方案仍是“未知，需要确认”，需作为 A1/launch admission 专项处理，不能在本次 ordinary bridge 重构中越界修改。

完整 ROS 运行证据：`docs/experiments/EXP-022-B42-B50-seed2026-120min-rosout.log`。该文件包含固定 seed/起点、首次 collision 前 80 ticks、冲突 geometry、最终 batch 指标和长期等待证据。

### 6.3 RViz：未完成视觉验收

batch 模式按现有实现不发布 marker。本次完成了 marker 代码构建和测试，确认数据直接来自已计算的 dynamic bridge correction，没有显示专用几何重扫；但未启动实时 RViz 对 marker 外观做人工视觉验收。因此只能报告“代码存在且编译通过”，不能报告“RViz 验收通过”。

## 7. 结论与保留风险

- 新 ordinary 控制链满足：clear 不触发 bridge、bridge 只修正 TTC、双方独立、correction 在 priority 之前、yielding 使用自身 corrected TTC、无 bridge 长期状态、无完整 path-pair matrix/BFS。
- A1、slot departure、hard guard、priority 数学规则和动态速度档位语义未重写。
- 单元/集成测试通过，但 B42/B50 120 min 安全回归失败，因此本变更不能标记为完整多车回归通过。
- 最近点局部搜索是明确的工程近似；复杂路径的全局对应精度仍需后续独立验证。
