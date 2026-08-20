# EXP-013：普通道路 Generic Timed Conflict 最小收缩

## 1. 结论

**FAIL**

本轮已把非 A1 的普通 `ACTIVE-ACTIVE` pair 收缩为单一同步 OBB
`TimedConflictEvent` 决策链，移除了 `CROSSING / OPPOSING /
SAME_DIRECTION`、`SharedSegment / OccupancyInterval` 对普通 winner、候选动作和
候选安全复检的影响。Release 构建成功，7/7 CTest 通过，普通
`ConflictReservation create=0`。

但是 seed=2026 自然回归没有达到安全验收：10 min 和 30 min 均在
`sim_t=178.4 s` 首次触发 hard guard，并在 `203.6 s` 起形成长期 deadlock；30 min
仅完成 V0/V1=`2/2` 个任务。因此没有启动有明确前置条件的 120 min 回归。

分类主权跳变已经消失，但当前 `priorityWinner()` 并不具备请求中假定的跨周期稳定性：
其 `unifiedPriority()` 包含 starvation `wait_time` 提升。等待车超过 8 s 后成为 winner，
一旦恢复运动又立即清零 wait，下一 rolling period 可再次失去 winner。该既有优先级翻转与
action hold、加减速惯性叠加，构成本次碰撞的直接控制链。本轮按约束没有增加 hysteresis、
长期 owner、普通 reservation、窄道资源或 deadlock 状态机。

基线 commit：`d847a70`。本轮未创建 Git commit。

## 2. 修改前核查

普通道路分类和控制依赖集中在以下调用链：

1. `RuleEngine::resolvePairwiseConflicts()` 先调用
   `detectPairInteractionFromPredictions()` 得到同步 OBB `CROSSING` 事件。
2. 同一函数随后用 `sameDirectionLeader()` 把事件改写为 `SAME_DIRECTION`，或把
   `ConflictZone` 映射为 `SharedSegment`，再以
   `detectSharedSegmentInteraction()` 改写为 `OPPOSING`。
3. winner 分别可能来自前车、shared segment 实际占用者、预计先清出者、即时
   A-only/B-only 清出时间或 `priorityWinner()`。
4. `evaluateSelectedAction()` 对 OPPOSING 候选重新切回 SharedSegment occupancy
   检测；`evaluatePairSpeedCoordination()` 只为 CROSSING 交换 winner，并让即时
   STOP/STOP 兜底排除 SAME_DIRECTION。
5. `resolveFollowing()` 仅有定义，当前 `RuleEngine::decide()` 没有调用；它不是本次
   自然场景的实际控制入口。

EXP-012 旧日志明确记录了标签跳变：plan 90--96 为
`OPPOSING winner=V0 selected=NOMINAL/STOP after_action=CLEAR`，plan 97 突变为
`CROSSING winner=-1 selected=STOP/STOP after_action=CONFLICT@0`。

## 3. 最小收缩内容

- `RuleEngine::resolvePairwiseConflicts()`：普通 pair 直接使用 nominal/nominal 的
  同步 OBB 事件，不再运行 same-direction 或 SharedSegment 分流。
- 普通 winner 统一调用 `priorityWinner()`；`priorityWinner()` 保留原 slot occupant
  前置约束和 `unifiedPriority()`，移除同向前后车几何 override。
- 删除普通 pair 的 occupancy winner、双车同时 inside 清出时间 winner、即时
  A-only/B-only 清出时间 winner。
- `evaluateSelectedAction()` 无条件调用
  `detectPairInteractionFromPredictions()`；baseline 和 YIELD/CREEP/STOP 候选使用
  同一个同步 OBB 物理入口。
- 删除 CROSSING 专属 opposite-order swap / both-stop 分支和 OPPOSING 专属 reason。
- unresolved immediate STOP/STOP 保留，但只依赖候选同步 OBB 复预测，不再排除
  SAME_DIRECTION。
- 普通动态日志统一输出
  `interaction=GENERIC_TIMED_CONFLICT`。历史 `crossing_conflicts` 统计字段为保持指标
  ABI 暂时承担 generic conflict 计数；它不参与控制。
- `SharedSegment`、`OccupancyInterval` 结构和 focused helper tests 保留，未物理删除，
  但普通运行时和普通候选评估不再调用它们。

修改后的普通 pair 链：

```text
current real state
  -> predict A/B with NOMINAL over 15 s
  -> synchronized OBB overlap
  -> TimedConflictEvent
  -> priorityWinner
  -> FAR/MID/NEAR candidate ladder
  -> re-predict both vehicles over full 15 s
  -> same synchronized OBB detector
  -> execute frozen first 2 s target
  -> refresh from real state
```

## 4. 明确未修改的边界

没有修改 `checkSlotDepartureAdmission()`、`checkA1LaunchAdmission()`、
`enforceFutureA1Admission()`、`enforceDepartureClusterCommitments()`、A1 service owner、
FutureA1Commitment、DepartureClusterCommitment、A1 reservation 生命周期、hard guard、
forward clearance、target slot occupancy、任务分配或路径生成。

现有 A1 focused tests 全部通过；30 min 中 A1 reservation create=4（REAL），普通
reservation create=0。SlotDepartureAdmission 仍为 ALLOW=4、HOLD=1、释放=1、最大
HOLD=2.8 s，与 EXP-012 同 seed 结果一致。

## 5. 测试

环境：WSL `Ubuntu-20.04-ros`、ROS Noetic、`~/stage32_ws`、Release。

- `catkin_make -DCMAKE_BUILD_TYPE=Release -j2`：成功。
- `ctest --output-on-failure`：7/7 passed。
- `dynamic_speed_coordination_test` 新增标签不变性：同一个 baseline 分别携带
  CROSSING、含有效 SharedSegment 的 OPPOSING、SAME_DIRECTION 标签，winner、动作、
  OBB 复检结果和 reason 必须完全一致。
- `dynamic_speed_rule_engine_test` 验证共线普通 pair 只记录
  `GENERIC_TIMED_CONFLICT`，不产生 same-direction authority 或普通 reservation。
- `git diff --check`：通过；只有 Windows 行尾转换提示，无 whitespace error。

## 6. seed=2026 回归

共同参数：2 vehicles、`reproducible_task_random=true`、10 Hz、15 s horizon、2 s
rolling refresh。

| 指标 | 10 min | 30 min |
| --- | ---: | ---: |
| completed ticks / sim time | 6000 / 600.0 s | 18000 / 1800.0 s |
| hard guard | 1，首次 178.4 s | 1，首次 178.4 s |
| deadlock ticks / first | 793 / 203.6 s | 3193 / 203.6 s |
| tasks V0/V1 | 2 / 2 | 2 / 2 |
| max wait V0/V1 | 421.7 / 423.6 s | 1621.7 / 1623.6 s |
| ordinary reservation create | 0 | 0 |
| generic conflict evaluated | 220 | 820 |
| FAR / MID / NEAR | 2 / 2 / 216 | 2 / 2 / 816 |
| unresolved immediate STOP/STOP | 211 | 811 |

30 min 动作时间：V0 STOP/CREEP/YIELD/NOMINAL =
1638.4/1.4/0.0/135.7 s；V1 = 1668.3/3.1/0.0/106.2 s。动态协调
目标没有自然产生 YIELD/CREEP；已有 CREEP 时间来自其它既有控制/保持链。因此“短测中
YIELD/CREEP/STOP 均正常选择”没有满足。

没有普通 DYN-SPEED 日志再输出 OPPOSING、CROSSING 或 SAME_DIRECTION；coord log 中唯一
`interaction=CROSSING` 来自 SlotDepartureAdmission 诊断，不是普通动态动作分流。

## 7. 首次失败原因链

```text
164.5 s: V1 SlotDepartureAdmission ALLOW，首个冲突位于 slot prefix 之外
plan 90--94:
  Generic Timed Conflict，winner=V0
  V1 STOP；候选只延后冲突、未在完整 horizon clear
  V1 wait_time 累积并超过 starvation_wait_time=8 s
plan 95:
  unifiedPriority 的 starvation 分量把 winner 翻转为 V1
  selected=STOP/NOMINAL
  V1 一旦运动，wait_time 清零
plan 96:
  winner 又翻回 V0
  selected=NOMINAL/STOP
  既有 action hold 和物理加减速使上一周期动作不能瞬时切换
178.4 s:
  两车实际足迹重叠，hard guard=1
plan 97:
  Generic baseline first_t=0
  candidate 仍冲突 -> STOP/STOP, winner=-1, accepted=false
203.6 s:
  首次 deadlock，之后持续到实验结束
```

这不是旧的 interaction 标签主权跳变；标签已经统一。新证据表明，现有
`priorityWinner()` 的 starvation 分量与“稳定 pair priority”的需求假设冲突。修复它需要单独
决定 starvation、公平性、action hold 与跨 rolling period winner 稳定性的边界，超出本轮明确
禁止新增 hysteresis/ownership/deadlock 策略的范围。

## 8. 产物

日志目录：`forklift_planner/logs/EXP013/`。

- `seed2026_10_console.log` SHA-256：
  `F455446B24D652712372703727A99D63E95597F34EC6B79AE53D450A7777818B`
- `seed2026_10_coord.log` SHA-256：
  `D84C8B8F9651B70F79431EF1BEA8DB260BC52ACCAF908D6A60D95357A889979D`
- `seed2026_30_console.log` SHA-256：
  `D8AE2C7A2FDE4F85446158981C67E4BB5BD9502FE44EFB5D85878D55F1A93D76`
- `seed2026_30_coord.log` SHA-256：
  `D08807DA1EC0FA7D7EF49E268809E53E9774829EFF75C954714BE520ADA59BCA`

120 min：**未执行**。30 min 的 `hard_guard=0`、无长期 deadlock 前置条件未满足。

