# EXP-014：Generic Timed Conflict 双顺序物理验证

## 1. 结论

**FAIL**

EXP-013 确实删除了原 CROSSING 的 opposite-order swap：普通 pair 只调用一次
`evaluateOrder(preferred_winner)`。本轮已按要求恢复与 interaction 类型无关的双顺序物理验证：
priority 只给 preferred order；preferred 的完整候选梯全部不 CLEAR 时，立即用同一个 15 s
同步 OBB detector 验证 alternate order；alternate 有 CLEAR 候选才采用
`reason=rolling_order_swap`；两种顺序都失败时保留现有安全兜底。

Release 构建和 7/7 CTest 通过，双顺序 focused cases 全部通过，普通 reservation create=0。
但 seed=2026 的原现场没有被修复：plan 90 的 alternate order 首次得到 CLEAR 并被采用，执行
2 s 后 plan 91 两种顺序均不 CLEAR，控制又回到 preferred order。hard guard 从 EXP-013 的
178.4 s 提前到 177.6 s，deadlock 从 203.6 s 提前到 202.6 s。30 min 未通过，因此未运行
120 min。

基线 commit：`d847a70`；EXP-013 变更在开始本轮时已位于 Git index。本轮没有执行 stage、
unstage 或 commit。

## 2. 核查与修改

修改前普通链已经是：

```text
real state
 -> NOMINAL/NOMINAL 15 s prediction
 -> synchronized OBB TimedConflictEvent
 -> priorityWinner
 -> FAR/MID/NEAR
 -> preferred loser YIELD/CREEP/STOP
 -> full 15 s OBB re-prediction
 -> execute first 2 s
```

缺失点是 preferred 梯失败后没有第二次 `evaluateOrder()`。本轮增加：

```text
preferred_order = evaluateOrder(priorityWinner)
if preferred_order CLEAR:
    select preferred
else if band is MID/NEAR:
    alternate_order = evaluateOrder(other winner)
    if alternate_order CLEAR:
        select alternate, reason=rolling_order_swap
    else:
        retain current preferred minimal action
        if current/immediate conflict remains: STOP/STOP fallback
```

FAR 仍保持 NOMINAL/NOMINAL 且不评估没有意义的 alternate order。每个 order 中的每个候选
都调用 `predictTrajectory()` 生成完整 horizon，并统一调用
`detectPairInteractionFromPredictions()`。没有恢复 SharedSegment、OccupancyInterval、
OPPOSING、SAME_DIRECTION、front-car winner、inside winner、exit-time winner 或 A-only/B-only
特殊 winner。

日志新增：preferred/alternate winner、每个候选及 CLEAR/CONFLICT 时间、两边 best、最终
winner、wait time、真实 speed、最终动作和 accepted/reason。

## 3. focused tests

环境：WSL `Ubuntu-20.04-ros`、ROS Noetic、`~/stage32_ws`、Release。

- Release build：成功。
- CTest：7/7 passed。
- preferred order CLEAR：不尝试 alternate。
- preferred 全失败、alternate 某候选 CLEAR：采用 alternate，reason 为
  `rolling_order_swap`。
- 两个 order 都失败且冲突 immediate：STOP/STOP、winner=-1、accepted=false。
- CROSSING / OPPOSING（含有效 SharedSegment）/ SAME_DIRECTION 标签不改变动作结果。
- RuleEngine ordinary reservation focused case：create=0。
- `git diff --check`：通过，无 whitespace error。

## 4. plan 90--96 回放

以下来自 seed=2026 的 4 min 短测。`best` 为该顺序候选梯最终结果。

| plan | first_t | preferred | wait V0/V1 | speed V0/V1 | preferred best | alternate best | selected |
| ---: | ---: | --- | --- | --- | --- | --- | --- |
| 90 | 6.150 | V0 | 0.0/0.0 | 0.200/0.000 | NOMINAL/STOP conflict@13.950 | STOP/NOMINAL CLEAR | V1，order swap |
| 91 | 5.850 | V0 | 2.0/0.0 | 0.000/0.142 | NOMINAL/STOP conflict@10.650 | STOP/NOMINAL conflict@14.400 | V0 |
| 92 | 4.350 | V0 | 0.0/2.0 | 0.200/0.000 | NOMINAL/STOP conflict@8.850 | STOP/NOMINAL conflict@12.850 | V0 |
| 93 | 3.600 | V0 | 0.0/4.0 | 0.160/0.000 | NOMINAL/STOP conflict@6.850 | STOP/NOMINAL conflict@11.100 | V0 |
| 94 | 2.800 | V0 | 0.0/6.0 | 0.200/0.000 | NOMINAL/STOP conflict@4.850 | STOP/NOMINAL conflict@4.350 | V0 |
| 95 | 1.550 | V0 | 0.0/8.0 | 0.142/0.000 | NOMINAL/STOP conflict@2.850 | STOP/NOMINAL conflict@3.200 | V0 |
| 96 | 0.550 | V1 | 0.0/10.0 | 0.200/0.000 | STOP/NOMINAL conflict@0.700 | NOMINAL/STOP conflict@0.850 | V1 |

plan 95 时 V1 的 wait 正好为 8.0 s，而 starvation 判断是严格 `> 8.0 s`，所以 preferred
仍为 V0；两种顺序都不 CLEAR。plan 96 时 V1 wait=10.0 s，starvation 将 preferred 改为
V1；V1-first 不 CLEAR，V0-first 也不 CLEAR。按照本轮明确规则“只有 alternate CLEAR 才
采用 alternate；双失败保持当前最小安全行为”，最终仍选择 preferred V1。

关键的新发现发生得更早：plan 90 的 V1-first 虽然在当前完整 15 s 预测中 CLEAR，但只执行
2 s 后，真实状态进入 plan 91 时已经没有任何 CLEAR 通行顺序。也就是说，一次 rollout 的
full-horizon CLEAR 没有提供跨 rolling refresh 的通行顺序连续性。本轮禁止新增 winner lock、
hysteresis、persistent owner 或 reservation，因此没有继续扩展。

## 5. seed=2026 回归

共同参数：2 vehicles、`reproducible_task_random=true`、10 Hz、15 s horizon、2 s rolling
refresh。

| 指标 | 4 min | 10 min | 30 min |
| --- | ---: | ---: | ---: |
| ticks / sim time | 2400 / 240 s | 6000 / 600 s | 18000 / 1800 s |
| hard guard | 1 @177.6 s | 1 @177.6 s | 1 @177.6 s |
| deadlock ticks / first | 75 / 202.6 s | 795 / 202.6 s | 3195 / 202.6 s |
| tasks V0/V1 | 2/2 | 2/2 | 2/2 |
| order swap count | 1 | 1 | 1 |
| ordinary reservation create | 0 | 0 | 0 |
| generic conflicts | 40 | 220 | 820 |

30 min 动作时间：V0 STOP/CREEP/YIELD/NOMINAL =
1640.3/1.4/0.0/133.8 s；V1 = 1667.0/3.3/0.0/107.3 s。普通动态候选没有产生
自然 YIELD/CREEP 最终目标；唯一 order swap 是 plan 90 的 STOP/NOMINAL CLEAR。

A1 与 launch 指标保持：A1 real reservation create=4；SlotDepartureAdmission
ALLOW/HOLD/released/max hold=`4/1/1/2.8 s`。普通 reservation 始终为 0。

120 min：未执行。30 min 的 hard_guard=0、无长期 deadlock 前置条件未满足。

## 6. 最早失败原因链

```text
plan 90:
  preferred V0-first 全部不 CLEAR
  alternate V1-first 的 STOP/NOMINAL 在当前 15 s rollout CLEAR
  -> 采用 rolling_order_swap

执行该目标前 2 s

plan 91:
  preferred V0-first 与 alternate V1-first 均不 CLEAR
  -> 回到当前 preferred V0，形成第一次真实顺序反转

plan 92--95:
  两种顺序持续不 CLEAR，V1 wait 持续增加

plan 96:
  starvation 将 preferred 翻为 V1
  V1-first conflict@0.700，V0-first conflict@0.850，均不 CLEAR
  -> 按现有双失败规则保留 preferred V1

177.6 s:
  hard guard

plan 97:
  baseline first_t=0；两种顺序都 conflict@0
  -> STOP/STOP, winner=-1, accepted=false

202.6 s:
  persistent deadlock
```

结论：恢复的双顺序物理验证真实运行且能选出 alternate CLEAR，但没有消除 EXP-013 失败链，
反而暴露了“单周期 CLEAR 不等于滚动执行中的顺序连续性”。按要求仅报告该最早原因，不自动
增加新的 owner、锁、reservation 或 recovery 机制。

## 7. 产物

日志：`forklift_planner/logs/EXP014/`。

- 4 min console/coord SHA-256：
  `889908ECA568579AE6711046CF58B19D4485FA1021447D786711F821ED93276D` /
  `04DB52833BFE3FBC7A21059957EA824B7066F4AFD221437AC02C71EF905E9068`
- 10 min console/coord SHA-256：
  `D22BB5D18E75FFCE27A1BE73E6D541A53439E7238346BEB8B3EF545B6F011198` /
  `7B3143A8254ABC2A4EB706360007EB977D4B4A763BA7BC934197D2C09F5E9C18`
- 30 min console/coord SHA-256：
  `9B55026DE0C782652B5C997D8B30A734A0C3B5FAA6AD0735C8F569FE5A72B8B7` /
  `0888FC9FC9A9D0845BBD34F699CC5DC6224F616453626FC6C99822D3C4723132`

