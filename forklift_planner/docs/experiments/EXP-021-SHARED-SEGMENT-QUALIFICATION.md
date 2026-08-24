# EXP-021：SharedSegment component-level qualification

## 基线与范围

- 日期：2026-08-24
- Git 基线：`4a6038f0c54a5052a66d4eb076237f20b271a4c6` 加工作区内 EXP-020 Stage 1 静态 geometry 实现
- 目的：区分 local opposing sample、opposing connected component 和 qualified `SharedSegmentCandidate`
- 不在范围：A1、priority、TTC、动态调速、动作、reservation、hard guard、slot departure admission、路径生成和车辆动力学

## 问题确认

修改前的 `computeSharedSegmentCandidates()` 在同一 traversal pair 内完成 8 邻域 BFS 后，无条件执行 `candidates.push_back(candidate)`。因此任何包含 `direction_dot < -0.5` sample 的 connected component 都直接成为 candidate；统计仅有 dot min/max 和 sample count。问题判断成立。

## 参数变化

原配置不存在下列参数。本实验新增：

| 参数 | 新默认值 | 单位/范围 | 作用 |
|---|---:|---|---|
| `shared_segment_min_span` | 0.50 | m，>=0 | A、B 双方 component path span 的下限 |
| `shared_segment_strong_opposing_threshold` | -0.80 | [-1,-0.5] | strong sample 判据 `dot < threshold` |
| `shared_segment_min_strong_ratio` | 0.60 | [0,1] | strong sample 占比下限 |

`0.50 m` 约等于当前两倍膨胀车长：`2 * (vehicle_length 0.211 + conflict_margin 0.040) = 0.502 m`。它是保守首版诊断默认值，不是正式安全标定值。`-0.8` 和 `0.60` 用于要求多数 sample 明显对向，同时保留 B42/B50 实测数据供后续标定。

第一层 local opposing 判据保持编译期常量 `direction_dot < -0.5`，语义未修改。

## 实现

完整 BFS closure 完成后才计算：

- `span_a = s_a_exit - s_a_enter`
- `span_b = s_b_exit - s_b_enter`
- `direction_dot_mean`
- `strong_opposing_count`
- `strong_opposing_ratio`

仅当双方 span 和 strong ratio 同时达到配置阈值时才输出 candidate。BFS 过程中不根据 span 或 strong threshold 删除 sample。

## Focused tests

新增 qualification 断言：

1. 足够长且 dot=-1 的稳定对向 component 通过，mean=-1、strong ratio=1；
2. dot=-1 但双方 span 只有 0.30 m 的 component 被 span 条件拒绝；
3. 足够长但全部 dot=-0.60 的 component 先完整形成，随后因 strong ratio=0 被拒绝；
4. 原 Stage 1 traversal、cusp、转弯、semantic gap 和确定性测试保持通过。

Release 构建成功，CTest 8/8 通过。

## B42/B50 原始 component 审计

为了观察被拒绝 component 的分布，另运行 1 tick 诊断，将 `min_span=0`、`min_strong_ratio=0`；这不是默认配置，也不保留为正式参数。

| component | span A/B (m) | dot min/max | mean | strong | ratio |
|---|---|---|---:|---:|---:|
| C0 R/R | 1.100 / 1.125 | [-1.000,-0.538] | -0.985 | 698/715 | 0.976 |
| C1 F/F | 0.225 / 0.200 | [-0.758,-0.526] | -0.610 | 0/31 | 0.000 |
| C2 F/F | 0.325 / 0.250 | [-0.894,-0.501] | -0.698 | 13/64 | 0.203 |

默认参数下仅 C0 qualified。C1 同时不满足 span 和 strong ratio；C2 同时不满足 span 和 strong ratio。

## Ordinary coordination 回归

固定 `start_slots=[42,50]`、seed 2026、600 tick，与 EXP-019 基线逐行比较：

- `ROLLING-DECISION`：30/30，差异 0
- `DYN-SPEED`：30/30，差异 0
- `FUTURE_A1`：30/30，差异 0
- `FIRST-COLLISION`：4/4，差异 0
- 动作计数均为 `NOMINAL/YIELD=2`、`NOMINAL/CREEP=1`、`NOMINAL/STOP=3`、`STOP/STOP=24`
- ordinary reservation created 均为 0
- hard guard 均在 tick 95、sim_t=9 s 首次触发

## RViz / 日志

默认参数实时运行并实际启动 RViz。6.1 s bag 包含 65 条 planner MarkerArray；稳定的 62 帧中每帧各有一个 `shared_segment_candidate_aabb` 和一个 explanation marker，标签为 C0。C1/C2 不再显示。

静态日志增加 span、dot mean、strong count/ratio 和本次使用的 qualification 阈值，仍明确标记 `static_prior=true control_input=false`。

产物：`forklift_debug_bags/EXP-021-B42-B50-QUALIFICATION/`。

## 结论

PASS。最小 component-level qualification 达到预期，三层语义已分离；默认参数只保留稳定的 C0，且 ordinary dynamic coordination 行为不变。默认阈值仍属于首版实验参数，后续如修改必须使用新的实验编号重新回归。
