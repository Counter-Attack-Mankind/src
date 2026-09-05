# EXP-033 A1Coordinator 架构迁移与双车 120 min 验收

## 基线与范围

- 分支：`fix`
- 基线 commit：`4e713095a72cb29d3d32a9c3f51439684b39e97a`
- 开始前工作区已有修改：根目录 `.gitignore` 为删除状态；本任务未恢复、覆盖或修改该状态。
- 目的：在不改变 A1 算法、停车规则、TTC、deadlock、参数和安全阈值的前提下，将 A1 owner、rolling context、Future admission、DepartureCluster、pair authority 和纯 A1 launch admission 迁入独立 `A1Coordinator`。

## 修改范围

- 新增 `include/forklift_planner/multi_vehicle/a1/a1_coordinator.h` 与 `src/multi_vehicle/a1/a1_coordinator.cpp`。
- `MultiVehiclePatrolNode` 仅通过 `RuleEngine::refreshA1PlanningContext()`、`futureA1Commitment()`、`checkSlotDepartureAdmission()` 等 façade 使用 A1 协调能力，不直接持有 `A1Coordinator`。
- `RuleEngine` 保留统一编排、pairwise reservation 执行、普通道路 launch 检查和 restrictive action merge。
- A1 ETA 仍使用 `dt=1/update_rate`、Node 的 `curvatureSpeed()`、`limitedSpeed()` 与 NOMINAL speed 离散推进；未换用 `predictTrajectory()`。
- `RuleEngine::decide()` 顺序保持：DepartureCluster refresh → pairwise → Future A1 admission → DepartureCluster enforcement → target-slot occupancy → forward-clearance → apply actions。
- CMake 只给主节点及三个直接编译 `rule_engine.cpp` 的测试目标补充 `a1_coordinator.cpp`。

## 构建与测试

环境：WSL `Ubuntu-20.04-ros`、ROS1 Noetic。

构建命令：

```bash
cd /mnt/d/desktop/叉车
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j8 -l8
```

结果：主节点及全部测试 target 编译、链接成功。WSL/Windows 时钟存在约 10 秒偏差，make 输出 clock-skew warning，但命令退出码为 0。

现有 CTest：9 项中 6 项通过，3 项失败。

- PASS：`real_state_estimation_test`、`bridge_ttc_correction_test`、`conflict_zone_closure_test`、`future_a1_policy_test`、`spatiotemporal_interaction_test`、`prediction_execution_consistency_test`。
- FAIL：`dynamic_speed_coordination_test`（FAR fixture）、`dynamic_speed_rule_engine_test`（同一 FAR fixture）、`rolling_decision_timing_test`（MID fixture）。其中独立的 `dynamic_speed_coordination_test` 及其被测实现未被本任务修改；因此不能将整套 CTest 记为通过，也不能仅凭这次运行把三项失败归因于 A1 迁移。

## 固定运行验收

唯一有效的主要运行命令：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=120 vehicle_count:=2 random_seed:=2026 \
  coord_log_file:=/mnt/d/desktop/叉车/src/forklift_planner/docs/experiments/EXP-033-A1-COORDINATOR-120min-coord.log
```

沿用 launch/YAML 既有默认：`start_slots=[38,20]`、可复现任务随机开启、10 Hz、正式 A1 cycle 配置；未为通过测试调整参数。

运行完成 72,000/72,000 ticks、7,200.0 s，进程正常退出，hard guard/collision 为 0。但验收失败：

- `sim_t=196.1 s`（3 min 16.1 s）：plan 106 首次持续输出 `STOP/STOP`。
- 同一 plan 的 frozen transaction 为 owner V1、transaction generation 5、departure generation 6、waiter V0 generation 7；创建时立即触发 `A1_ADMISSION_INVARIANT_VIOLATION`，原因为 `waiter_crossed_frozen_boundary`。
- `sim_t=221.0 s`：首次 `FIRST-WEDGE`，两车已连续等待 25.0 s。
- `sim_t=221.1 s`：正式检测到 V0/V1 deadlock。
- 至 `sim_t=7200.0 s` 未恢复；最大等待 7004.0 s，任务数 V0/V1=`3/2`，deadlock 检出拍 13,958，重规划脱困 0。
- 因此“连续正常运行多久”的答案是约 **196.1 s（3.27 min）**，不是 120 min passed。

本任务是行为等价结构迁移，未针对该失败修改 A1 admission、停车或 deadlock 算法。该运行结果作为后续独立问题的证据保留。

## 产物

- `EXP-033-A1-COORDINATOR-120min-rosout.log`：SHA-256 `584531C47982E6414AA17070E95DC4FAAC4B54618567FB07235075984C05BA44`
- `EXP-033-A1-COORDINATOR-120min-coord.log`：SHA-256 `F93240677F74C01479F036A0B7C9E4E566638A228BBE292789EB93222B6820B3`

结论：保留架构迁移实现与失败证据；固定 120 min 运行验收判定为 **FAIL**。
