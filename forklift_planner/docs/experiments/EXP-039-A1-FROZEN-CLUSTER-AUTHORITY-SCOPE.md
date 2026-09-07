# EXP-039：A1 frozen departure cluster authority scope

## 基线与目的

- Git 基线：`2009bc9`（分支 `fix`，实验前工作区 clean）。
- 目的：确认并修复 active `DepartureCluster` / Future A1 owner 将整对车辆的当前道路冲突升级为 `a1_related` 的问题。
- 改动范围：仅 `RuleEngine::resolvePairwiseConflicts()` 的 A1 pair authority 分流与对应测试；未修改 A1 owner、ETA、cluster 生成、frozen zone、stop/release、TTC 参数、bridge 算法、priority、DeadlockManager 或控制器。
- 参数变化：无。

## 审查证据

修改前，`resolvePairwiseConflicts()` 调用 `A1Coordinator::authorityForPair()`，只要 `departure_owner_id` 或 `future_owner_id` 有效，就把当前 pair 标成 `a1_related`。这个判定没有把当前 timed OBB event 与 frozen `FutureA1ConflictInterval` 做几何归属匹配。随后该 pair 跳过 ordinary dynamic TTC，创建 `create_reason=a1_related` 的 `ConflictReservation`，并由 `brakeBefore()` 产生 `time_brake_Vx`。

`A1Coordinator::departureClusterOwnerForPair()` 校验 pair、车辆身份、phase 和 path generation，但本身不判断当前 `ConflictZone` 是否属于 frozen protected interval；`futureA1OwnerForPair()` 返回的同样是 pair authority。因此它们不能作为当前道路 event 的几何归属判定。

## 保留的权威边界

当前路径的 timed OBB event 全部继续进入 ordinary dynamic TTC。Frozen departure closure 仍由 `A1Coordinator::enforceDepartureClusterCommitments()` 单独执行，并继续依据：

- transaction owner / waiter ID 与 frozen path generation；
- frozen `FutureA1ConflictInterval`；
- `waiter_stop_boundary_s`、`waiter_stop_s`；
- `owner_release_exit_s` 与 transaction 生命周期；
- waiter identity change、越过 boundary、进入 closure 的 invariant STOP。

因此 A1 protected 的含义不再是“该 pair 存在 owner 关系”，而是 A1Coordinator 当前正在执行、且车辆/path identity 与 frozen transaction 相符的 protected interval / stop-boundary enforcement。该保护不再复制为 ordinary `ConflictReservation`。

## 修改

- 删除 `resolvePairwiseConflicts()` 对 `authorityForPair()` 的 pair-level `a1_related` 分流。
- 当前 event 统一使用同步预测的 timed OBB overlap，并保留 bridge TTC correction、priority、FAR/MID/NEAR 与 NOMINAL/YIELD/CREEP/STOP 链。
- 删除 `reason=a1_related` reservation 的创建、更新、复用和 `brakeBefore()` 路径。
- 对 snapshot 中遗留的旧 `ConflictReservation` 在进入 pair loop 前清除，避免旧状态继续跳过 TTC。
- 未修改 `A1Coordinator` 的 frozen cluster 数据结构、生成、生命周期与 enforcement。

## 验证

### 构建与定向测试

命令：

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make --pkg forklift_planner

cd /mnt/d/desktop/叉车/build
ctest --output-on-failure -R '(future_a1_policy_test|dynamic_speed_rule_engine_test|rolling_decision_timing_test|spatiotemporal_interaction_test)'
```

结果：构建通过；4/4 定向测试通过。

新增测试覆盖：

1. active frozen commitment 存在，但当前 crossing event 位于 frozen intervals 之外：出现 `[DYN-SPEED]` 和 `[BRIDGE-TTC]`，不出现 A1 skip、A1 reservation 或 `time_brake`。
2. waiter 接近 frozen `waiter_stop_s`：仍输出 STOP，reason 为 `departure_cluster_priority`，且不创建重复 reservation。
3. snapshot 中旧 `a1_related` reservation 被清除，不再覆盖 ordinary rolling decision。

### 固定种子短 batch

场景：2 车、60 s、`random_seed=2026`、`start_slots=[38,20]`，正式 A1 cycle。

产物：`forklift_planner/logs/EXP039-seed2026-coord.log`。

关键结果：

- plan 5 创建 active frozen transaction 后，plan 6--10 对同一 pair 继续输出 `[DYN-TTC]`、`[DYN-SPEED] interaction=GENERIC_TIMED_CONFLICT` 和 `[BRIDGE-TTC]`。
- 后续真实 bridge/opposing event 输出 `bridge_a=true bridge_b=true`，并按修正后 TTC 选择 CREEP/STOP；所有记录均为 `reservation=not_created`。
- 日志中 `a1_protected`、`reason=a1_related`、`reservation_reason=a1_related`、`time_brake` 均为 0 次。
- waiter 在 `stop_s=2.400` 前由 `departure_cluster_priority` 保持，transaction 仍按 `owner_cleared_frozen_transaction` 释放。
- batch 完成 600/600 ticks，`hard_guard=0`；dynamic metrics：baseline conflicts 17，bridge checked 17，bridge corrected pairs 8，A1 fallback 0，reservation create/update/existing 均为 0。

## 限制

- 未执行 RViz 人工轨迹核对；本次证据为定向测试、固定种子 batch、协调日志和 hard-guard 汇总。
- 当前仓库没有完整自动化系统测试，因此上述结果是本次范围内回归证据，不代表完整安全认证。
