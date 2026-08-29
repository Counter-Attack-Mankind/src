# EXP-037：A1 frozen future-departure transaction

## 基线与目的

- 分支：`fix`
- 基线 commit：`83fd2aaf85402191324142885fa408c286c5d1d2`
- 工作区：包含本实验对应的未提交源码修改
- 目的：在 Future A1 owner 锁定时冻结未来 `A1 -> B` departure geometry，使 waiter fence 跨 `TO_A1 -> PICKUP_DWELL -> TO_B` 连续有效，并仅由 owner 清出 frozen owner exit 后释放。

## 实现范围

- `DepartureClusterCommitment` 保存 service transaction generation、frozen departure generation 和 frozen owner track。
- 首次 Future A1 closure 计算后直接创建 active frozen cluster；不再等待 TO_B activation。
- phase/current path generation 不再使 frozen cluster 失效。
- waiter 超过 frozen boundary、进入 frozen closure 或 transaction identity 改变时，请求 owner/waiter 同时 STOP，并记录 `A1_ADMISSION_INVARIANT_VIOLATION`。
- 同一 owner transaction 的所有 pair closure 以最大 `owner_release_exit_s` 一次性释放。
- 移除 TO_B `deterministic_rebuild` 回生入口。
- `a1_stop_margin=0.10 m`、普通 TTC、bridge、priority、裸 OBB 和 ConflictZone 几何未修改。

## 构建与轻量验证

命令环境：WSL `Ubuntu-20.04-ros`、ROS Noetic。

- `catkin_make -DCATKIN_ENABLE_TESTING=ON`：PASS。
- `future_a1_policy_test`：PASS。
- `spatiotemporal_interaction_test`：PASS。
- `dynamic_speed_rule_engine_test`：FAIL，停在既有 ordinary FAR fixture：`FAR did not remain reservation-free NOMINAL`，尚未运行到本次新增 frozen fixture。
- `rolling_decision_timing_test`：FAIL，既有 ordinary MID fixture：`MID fixture did not select YIELD/CREEP`。

后两项失败位于本次 A1 transaction 修改之外；未修改 ordinary TTC 逻辑来掩盖该基线问题。

## 30 min 回归

场景：

- launch：`multi_vehicle_phase2_batch.launch`
- `random_seed=2025`
- `vehicle_count=2`
- `start_slots=[38,20]`
- `minutes=30` / `18000 ticks`
- 生产参数确认：`a1_stop_margin=0.10`

产物：

- `coordination.log`
- `debug/forklift_onset.log`
- ROS run id：`37d8d71c-a3bf-11f1-81af-8fa850bab09f`

### 首个 frozen transaction

- plan 5 frame 0：CREATE，owner V0，transaction gen 1，frozen owner gen 2，waiter V1 gen 1，boundary `2.500`，stop `2.400`，owner release exit `1.150`。
- plan 5 frame 70：HOLD，V1 rollout progress `2.320 < stop_s 2.400` 时进入制动条件。
- plan 18 frame 15（REAL snapshot）：RELEASE；相应 rollout owner progress `1.164 > owner_release_exit_s 1.150`。
- 下一 plan 的 Future A1 commitment 记录 `departure_resource_clear`；没有同事务 deterministic rebuild。

### 生命周期统计

- frozen departure CREATE：23。
- REAL frozen departure RELEASE：22；最后一个事务因回归结束时仍卡住未释放。
- 所有 23 个 CREATE 的 `boundary - stop_s`：min/max/avg 均约 `0.100 m`。
- `INVALIDATE=0`。
- `deterministic_rebuild=0`。
- `handoff_already_inside=true=0`。
- `A1_ADMISSION_INVARIANT_VIOLATION=0`。
- A1 service：create 41、hold 603、change 0、release 40、invalidate 0。

### 首个未恢复失败 transaction

- plan 828：owner V0 transaction gen 43、frozen owner gen 44、waiter V1 gen 39；boundary `0.575`、stop `0.475`、owner release exit `1.225`。
- waiter 最终停在 `s=0.477`，未超过 boundary，但略过 control stop point `0.475`；frozen fence 持续输出 `departure_cluster_priority`。
- owner V0 在 TO_B 上推进到 `s=1.165`，距离 release exit 尚差 `0.060 m`。
- plan 831 frame 101 的另一个局部 A1 ConflictZone 将已在该局部 event 内的 V1 选为 holder，V0 因 `time_brake_V1` STOP；由此形成 V0 等 V1、V1 等 frozen owner V0 的闭环。
- 首次明确 deadlock：tick 15741 / sim 1574.1 s；成员 V0、V1。回归结束仍未恢复。

### 最终统计

- completed ticks：18000 / 18000。
- hard guard：0。
- deadlock detected ticks：452。
- wedge episodes：3。
- reciprocal STOP cycles：0。
- 最大 wait：V1 263.9 s；V0 251.3 s。
- 完成运输任务：V0 21，V1 19。
- 结束状态：V0 `s=1.165`, `time_brake_V1`；V1 `s=0.477`, `departure_cluster_priority`。

## 结论

**FAIL**。

本次修改消除了 phase/path generation 导致的 `INVALIDATE -> CREATE` 循环，frozen fence、0.10 m stop margin 和 owner-only RELEASE 在前 22 个 closure 中均按预期工作，且 hard guard 为 0。但第 23 个 transaction 暴露出 frozen transaction authority 与既有局部 ConflictZone “actual inside 优先”规则的叠加闭环，造成 owner 在 release exit 前 0.060 m 停住。按本轮约束，仅记录首次未恢复失败，不继续修改 A1 ownership 或其他架构。
