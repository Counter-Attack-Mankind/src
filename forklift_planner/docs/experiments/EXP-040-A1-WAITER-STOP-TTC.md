# EXP-040：A1 waiter stop boundary 接入 rolling TTC

## 基线与范围

- 分支：`fix`。
- Git 基线：`f61967b`。
- 目的：将 active frozen departure commitment 已有的 `waiter_stop_s` 转换为 waiter-specific TTC，并裁掉 stop line 之后不可执行的普通预测 overlap。
- 未修改：普通 priority、bridge TTC 算法、动态阈值、STOP threshold、DeadlockManager、A1 owner ranking/horizon、DepartureCluster 生成和释放、路径生成、控制器。
- 参数变化：无。

## 实现

`A1Coordinator::waiterStopConstraint()` 只读返回与车辆 `id + path_gen + TO_A1` 匹配的 active commitment 边界。若同一 waiter 存在多个有效 commitment，选择最上游的最小 `waiter_stop_s`，相同 stop line 时按 owner ID 确定性选择。

`RuleEngine::resolvePairwiseConflicts()` 使用本轮已经生成的 NOMINAL rolling prediction：

1. 当前 `path_s >= waiter_stop_s` 时 TTC 为 0；否则寻找首个达到 stop line 的 sample，并在相邻 `(t,s)` 间线性插值；horizon 内未达到则为 CLEAR。
2. 复用 `classifyDynamicInterventionBand()`、`selectRollingSpeedAction()` 和 `evaluateTtcStopBoundary()` 产生 waiter 的 NOMINAL/YIELD/CREEP/STOP 请求。
3. 该请求通过既有 restrictive action merge 与 ordinary TTC 合并，不参与或改变 `priorityWinner()`，也不创建 `ConflictReservation`。
4. 若 timed overlap 的 waiter `collision_s > waiter_stop_s`，该 overlap 标记为不可达，不再约束 owner；若 collision 位于 stop line 之前，ordinary timed OBB、bridge correction 和 priority 链保持不变。
5. `enforceDepartureClusterCommitments()` 原有 stop-line STOP 与 invariant STOP 保留为最终兜底。

## 验证

构建命令：

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make --pkg forklift_planner
```

结果：构建通过。环境报告约 15 秒 clock skew warning，没有编译错误。

定向测试：

```bash
cd /mnt/d/desktop/叉车/build
ctest --output-on-failure -R '(future_a1_policy_test|dynamic_speed_rule_engine_test|rolling_decision_timing_test|spatiotemporal_interaction_test)'
```

结果：4/4 通过。覆盖 active commitment 外普通 conflict、stop line 前普通 conflict、stop line 后不可达 overlap 裁剪、waiter CREEP/STOP 和 frozen hard-stop 兜底。

本次按任务边界未运行多车 batch 或 RViz，不能据此声明完整协调回归或实车安全验证完成。
