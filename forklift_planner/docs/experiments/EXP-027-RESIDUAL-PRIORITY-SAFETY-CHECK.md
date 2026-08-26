# EXP-027：yielding 动作后的 priority residual safety check

## 1. 基线与范围

- 日期：2026-08-26
- 基线 HEAD：`81fa70d09053ed2bc253a932ab11cb415e508a1c`
- 固定场景：`vehicle_count=2`、`start_slots=[42,50]`，seed 2025/2026，各 72000 ticks / 7200.0 s 真实仿真时间。
- 保留：pair 共用的同步 OBB baseline TTC、bridge 双车各自 corrected TTC、既有 yielding FAR/MID/NEAR/emergency 档位、priority winner。
- 未修改：bridge、同步 OBB detector、priority 规则、A1、reservation、ConflictZone、forward clearance、阈值及配置。
- 工作区原有未跟踪 EXP-024/EXP-025 证据均保留，未覆盖。

## 2. 实施语义

`evaluatePairSpeedCoordination()` 现在按固定顺序执行：

1. 仍用调用方传入的 pair baseline/effective TTC 选择 yielding 动作；
2. 用既有 `evaluateSelectedAction()` 和同步 OBB detector 额外预测一次 `priority=NOMINAL + yielding=已选动作`；
3. residual CLEAR 时 priority 保持 NOMINAL；
4. residual 有碰撞但 TTC 高于现有 NOMINAL stop boundary 时，priority 仍保持 NOMINAL；
5. 仅 residual TTC 进入该 stop boundary 时，priority 才 safety STOP。

没有把 pair 的 `emergency_stop` 直接映射成 priority STOP，也没有拆分原始同步 TTC。

新增结果诊断：`residual_evaluated`、`residual_conflict`、`residual_first_conflict_t`、`priority_stop_threshold`、`priority_safety_stop`。`[DYN-SPEED]` 日志同步输出这些字段。

## 3. 构建与 focused test

环境：WSL Ubuntu 20.04 / ROS Noetic / Release。

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
cd build
ctest --output-on-failure
```

结果：构建成功，8/8 CTest PASS，总时间 1.77 s。存在约 0.69 s clock skew warning，但受影响测试目标已重新编译、链接并通过。

既有 `dynamic_speed_coordination_test` 已明确覆盖：

- yielding emergency STOP 后 residual CLEAR：priority=NOMINAL；
- immediate residual emergency：priority=STOP，且 priority identity 不变。

## 4. seed 2025 / B42-B50 / 120 min

结果：安全硬约束通过，但协调验收失败。

- 完成 72000/72000 ticks，7200.0 s；hard guard=0；`emergency_next_step`=0。
- deadlock=12566 ticks，wedge episode=1，最大等待 6318.0 s（V1）。
- 任务完成：V0=10、V1=12。
- ordinary reservation create=0；A1 service create/release=23/23。

关键转换：

| plan | yielding baseline/effective 结果 | residual TTC | priority threshold | 输出 |
|---:|---|---:|---:|---|
| 478 | yielding=STOP | 4.550 s | 2.767 s | NOMINAL/STOP |
| 479 | yielding=STOP | 2.550 s | 2.767 s | STOP/STOP |
| 480+ | yielding=STOP | 约 2.650 s | 2.767 s | 持续 STOP/STOP |

因此 plan 478 已直接证明：yielding 的 baseline emergency 不再自动让 priority STOP。plan 479 的双停来自额外 residual prediction 本身进入 NOMINAL emergency boundary。

## 5. seed 2026 / B42-B50 / 120 min

结果：安全硬约束通过，但协调验收失败。

- 完成 72000/72000 ticks，7200.0 s；hard guard=0；`emergency_next_step`=0。
- deadlock=13959 ticks，wedge episode=1，最大等待 7016.5 s（V1）。
- 任务完成：V0=2、V1=2。
- ordinary reservation create=0；A1 service create/release=5/5。

关键转换：

| plan | yielding baseline/effective 结果 | residual TTC | priority threshold | 输出 |
|---:|---|---:|---:|---|
| 106 | yielding=STOP | 3.950 s | 2.767 s | NOMINAL/STOP |
| 107 | yielding=STOP | 1.950 s | 2.767 s | STOP/STOP |
| 108+ | yielding=STOP | 约 2.100 s | 2.767 s | 持续 STOP/STOP |

seed 2026 同样证明 priority STOP 不是共享 emergency bool 的直接结果，而是 residual TTC 进入紧急边界后的结果。

## 6. 结论

本次代码语义已实现：共享 baseline/effective TTC 只直接决定 yielding 档位，priority 使用 yielding 已选动作后的独立 residual 物理预测。

但 B42/B50 的完整场景验收为 **FAIL**：两组 seed 最终都存在持续低于 NOMINAL stop boundary 的 residual collision，并形成长期 STOP/STOP。该结果说明“共享 emergency bool”确是原实现的响应耦合问题，但移除该耦合不足以解决当前几何下的永久死锁。

本实验不据此修改 stop boundary、bridge、priority、reservation 或恢复策略；后续方向属于新的算法决策，需要单独审查。

证据：

- `EXP-027-seed2025-B42-B50-120min-coord.log`
- `EXP-027-seed2025-B42-B50-120min-rosout.log`
- `EXP-027-seed2026-B42-B50-120min-coord.log`
- `EXP-027-seed2026-B42-B50-120min-rosout.log`
