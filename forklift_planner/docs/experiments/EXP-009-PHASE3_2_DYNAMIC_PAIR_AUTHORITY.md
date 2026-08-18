# EXP-009：阶段 3.2 普通 Pair 动态冲突与单一速度协调权威

## 1. 结论

**FAIL**

阶段 3.2 的普通道路结构目标已经落实并通过 focused tests 与 10/30 min
回归：Crossing 不再依赖静态 `ConflictZone`，Opposing 使用
`SharedSegment + OccupancyInterval`，Same-direction 由实际运动方向、横向距离和
前后顺序识别；普通 pair 不创建 `ConflictReservation`，运行时 action authority
只有既有 `evaluatePairSpeedCoordination()`。30 min 中四项阶段指标全部为 0。

但是 2 车 `seed=2026` 的正式 120 min 回归在 `sim_t=2230.3 s` 出现一次
`hard_collision_guard`，随后形成稳定 A1 等待环。因此整体阶段不能判 PASS。
现场属于保留的 A1 特殊资源链：V1 已到 A1 并进入 `PICKUP_DWELL`，V0 的
TO_A1 当前航段 reservation 在 V1 到站时删除，随后 V0 被放行接近 A1 静止车体；
这不是普通 pair reservation 或重复 safety 仲裁。尝试在
`FutureA1Admission::already_inside` 局部分支补 STOP 会把问题提前变成 A1 双停，
该试验改动已撤销。本阶段不越界重构 FutureA1/DepartureCluster 的 owner 时序。

## 2. 基线、环境与范围

- Git 基线：`a10691d`（阶段 3.1）。
- 阶段二对照：`81cfbfd`。
- 环境：WSL `Ubuntu-20.04-ros`，ROS1 Noetic。
- 验证工作区：`~/stage32_ws`，其中三个 source package 链接到当前 Windows
  源码：`forklift_planner`、`forklift_map`、`experiment/sandbox_msgs`。
- 正式参数：`vehicle_count=2`、`random_seed=2026`、
  `reproducible_task_random=true`、`rolling_horizon=15.0 s`、
  `rolling_refresh_period=2.0 s`、控制步长 `0.1 s`。
- 未修改路径生成、任务分配、A2、BOOST、deadlock recovery、实车接口和安全阈值。
- A1 `FutureA1Commitment`、`DepartureClusterCommitment` 与
  `a1_related` reservation 保留为特殊资源机制。

## 3. 当前真实调用链与单一权威

`RuleEngine::decide()` 的相关顺序为：

```text
reset requested_action
  -> refreshDepartureClusterCommitments
  -> resolvePairwiseConflicts
       -> detect dynamic relation/event
       -> evaluatePairSpeedCoordination (ordinary pair authority)
  -> enforceFutureA1Admission / enforceDepartureClusterCommitments
  -> resolveTargetSlotOccupancy
  -> enforceForwardClearance (next-step physical validation only)
  -> applyRequestedActions
```

`resolveFollowing()` 与 `applyFollowingSuggestions()` 的定义为兼容代码保留，但不再
从 `decide()` 调用；它们不再改变普通 pair 的运行时 action。普通 pair 的
`ConflictReservation CREATE` 唯一写入口不可达，实际 create 仅保留
`create_reason=a1_related`。`enforceForwardClearance()` 只比较双方下一控制步的
零余量 OBB，不再扫描制动距离并再次决定谁先行；其覆盖计入
`duplicate_pair_authority_overrides`。

## 4. 三种动态关系

### 4.1 Crossing

- `predictTrajectory()` 从双方当前 `path_s/current_speed` 生成同步 15 s 预测。
- `detectPairInteractionFromPredictions()` 即使 potential zones 为空也直接比较
  `OBB_A(t)` / `OBB_B(t)`，返回首个连续 `TimedConflictEvent` 的
  `first_t/last_t/timed overlap polygons`。
- 普通 Crossing 不再映射固定 zone，不创建长期 holder/waiter。
- preferred winner 的完整 horizon 候选若仍冲突，统一协调器内部尝试相反通行
  顺序；若仍不安全，再评估一个 rolling period 的 STOP/STOP 过渡。该逻辑不
  用于 Opposing 或 Same-direction，不形成跨周期 owner。

确定性失败点的修正证据（30 min plan 788）：

```text
baseline_first_t=1.000 interaction=CROSSING
preferred order V0 NOMINAL / V1 STOP -> conflict remained
selected winner=V1, V0 CREEP / V1 NOMINAL
after_action=CLEAR reason=rolling_crossing_order_swap
```

### 4.2 Opposing

- `SharedSegment` 来自完整固定路径的连续 OBB 重叠几何，仅描述空间边界，不携带
  owner。
- 方向使用共享段中点的实际运动切向；REVERSE 段补 `pi`，复用现有
  `dot < -0.5` 对向阈值。
- 每次 rolling 根据预测计算双方 `[t_enter,t_exit]`；冲突条件为 occupancy
  重叠，带 winner 时重新验证
  `loser.t_enter >= winner.t_exit + clearance_time`。
- 一方已实际进入共享段时，实际占用优先；双方已进入时按清空时间/既有优先级
  选择。所有 YIELD/CREEP/STOP 候选仍由同一协调器完整重预测。

### 4.3 Same-direction

- 不读取 `ConflictZone.same_dir`。
- 用双方当前实际运动 heading 点积、横向投影和纵向投影确认同向与前后关系。
- 稳定前车始终是 winner，后车才接受动态调速；不创建独立
  `FollowingConstraint` 或 reservation。

## 5. RViz 语义

- 蓝色 polygon 只保留固定路径可能 OBB 重叠的几何参考。
- 删除普通静态 `ConflictZone zoneN` 文本、静态橙色 zone AABB 和普通
  reservation marker。
- Crossing 的橙色框来自当前 timed overlap polygon 的 union AABB；红色区域只
  显示真实同步 OBB intersection polygon。
- Opposing 的橙色框为当前 active `SharedSegment`，标签带双方 occupancy interval、
  winner/loser、`first_t/last_t`。
- 本轮完成源码、构建和 marker 生命周期核对；未在 GUI 中保存人工 RViz 截图，
  视觉效果的人工验收状态为：**未知，需要确认**。

## 6. 构建与 focused tests

命令：

```bash
cd ~/stage32_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
cd build
ctest --output-on-failure
```

结果：构建成功，`7/7 tests passed`。

覆盖：

- zone-free Crossing timed event；
- Opposing occupancy conflict/clear 与 winner inequality；
- Same-direction 前车 winner；
- FAR/MID/NEAR、完整 horizon 候选升级；
- preferred Crossing order 不安全时改选完整 horizon CLEAR 的相反顺序；
- 普通 pair 不创建 reservation，A1 reservation 仍保留；
- prediction/execution consistency 与 rolling decision timing 原回归继续通过。

## 7. 2 车 seed=2026 回归

命令模板：

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=MINUTES vehicle_count:=2 \
  coord_log_file:=/mnt/d/desktop/叉车/Testing/EXP009/FILE_coord.log \
  debug_log_dir:=/mnt/d/desktop/叉车/Testing/EXP009/DEBUG_DIR
```

### 7.1 10 min（最终源码）

| 指标 | 结果 |
|---|---:|
| hard guard / deadlock ticks | 0 / 0 |
| tasks V0/V1 | 8 / 7 |
| max wait V0/V1 | 15.6 / 19.5 s |
| NOMINAL V0/V1 | 462.3 / 453.3 s |
| YIELD V0/V1 | 2.0 / 0.0 s |
| CREEP V0/V1 | 8.5 / 7.1 s |
| STOP V0/V1 | 41.6 / 71.0 s |
| crossing/opposing/same-direction | 1 / 8 / 1 |
| ordinary reservation / duplicate authority | 0 / 0 |

### 7.2 30 min（普通道路正式复核）

| 指标 | 结果 |
|---|---:|
| hard guard / deadlock ticks | 0 / 0 |
| tasks V0/V1 | 24 / 22 |
| max wait V0/V1 | 34.2 / 23.8 s |
| NOMINAL V0/V1 | 1311.0 / 1317.9 s |
| YIELD V0/V1 | 8.0 / 8.0 s |
| CREEP V0/V1 | 26.3 / 30.4 s |
| STOP V0/V1 | 219.5 / 223.2 s |
| crossing/opposing/same-direction | 12 / 25 / 1 |
| ordinary reservation create | **0** |
| duplicate pair authority | **0** |
| reciprocal STOP cycles | **0** |

### 7.3 120 min（正式验收）

| 指标 | 结果 |
|---|---:|
| requested/completed ticks | 72000 / 72000 |
| hard guard | **1（FAIL）** |
| first guard | tick 22303，sim_t 2230.3 s，V0-V1 |
| deadlock ticks / recovery | **9886 / 0（FAIL）** |
| max wait V0/V1 | 4969.9 / 4968.1 s |
| tasks V0/V1 | 30 / 28 |
| NOMINAL V0/V1 | 1666.7 / 1680.6 s |
| YIELD V0/V1 | 8.0 / 10.0 s |
| CREEP V0/V1 | 29.8 / 31.2 s |
| STOP V0/V1 | 5201.5 / 5198.9 s |
| crossing/opposing/same-direction | 12 / 26 / 2 |
| ordinary reservation create | **0** |
| duplicate pair authority | **0** |
| reciprocal STOP cycles | 0 |

120 min 的失败现场：V1 `TO_A1 -> PICKUP_DWELL` 后静止于 A1；V0 的当前
`a1_related` reservation 删除后重新获得 NOMINAL，直到 next-step safety 在
`s=1.881/2.116` 触发 STOP，随后 hard guard 阻止下一物理步。V1 激活 TO_B 后，
existing A1 reservation/forward safety 形成长期互等。该现场证明普通动态权威已不
重复仲裁，但现有 A1 admission、dwell、departure handoff 的保护闭包在 late owner
change/arrival transition 上仍不闭合。

## 8. 对照

| 版本 | 120 min hard guard | deadlock ticks | tasks V0/V1 | ordinary reservation |
|---|---:|---:|---:|---:|
| 阶段二 `81cfbfd` | 0 | 0 | 91 / 90 | 阶段二旧语义 |
| 阶段 3.1 `a10691d` / EXP-008 | 0 | 7303 | 44 / 45 | 0 |
| 阶段 3.2 本轮 | **1** | **9886** | **30 / 28** | **0** |

普通道路的阶段 3.1 死锁已被修复：本轮 30 min 无 hard guard、无 deadlock、无
重复仲裁，吞吐为 24/22。正式 120 min 的新失败晚于该普通道路现场，位于 A1
特殊资源链，因此任务能力仍明显低于阶段二，不能验收。

## 9. 实验产物

日志位于仓库外同级目录 `D:/desktop/叉车/Testing/EXP009`。

| 文件 | bytes | SHA-256 |
|---|---:|---|
| `stage3_2_2v_10_final2_console.log` | 356043 | `0752C6120B2155C3A3362432E09929D330D3B9B066420C6D408D8840DD61C5E6` |
| `stage3_2_2v_10_final2_coord.log` | 1987268 | `7EDB00F43B64E640E70C845A9A102EE0C88122035168DFF981BAA4EDD31B4023` |
| `stage3_2_2v_30_v5_console.log` | 670850 | `C0D9304B8AF94DC0B23BE039DC36F16383F8E938C83F5426028B77D854E3AF61` |
| `stage3_2_2v_30_v5_coord.log` | 7440690 | `5611BC88884529301497166E89FFBA21D05CEB89769888F18A39D4E6BD140B37` |
| `stage3_2_2v_120_final_console.log` | 1897196 | `0D6FFA94A820F222D0D63EEEDF59A071230480BFD0AFEBE41F3A30187EBCD56B` |
| `stage3_2_2v_120_final_coord.log` | 68841301 | `FCFFD1F843C533285351F7611A46D84AF99E839A1E3204275D4C6D31F678C27C` |

## 10. 最终验收

| 验收项 | 结论 |
|---|---|
| Crossing 脱离静态 ConflictZone / 普通 reservation | PASS |
| Opposing SharedSegment + OccupancyInterval | PASS |
| Same-direction 动态前后关系，前车 winner | PASS |
| 普通 pair 单一 Dynamic Speed Coordination 权威 | PASS |
| ordinary reservation create = 0 | PASS |
| duplicate pair authority = 0 | PASS |
| RViz 静态普通 zone 误导显示已删除 | PASS（源码/构建）；GUI 人工确认待办 |
| 2 车 seed=2026，120 min hard guard = 0 | **FAIL** |
| 2 车 seed=2026，无 persistent deadlock | **FAIL** |
| 任务能力不低于阶段二 | **FAIL** |

**FAIL**
