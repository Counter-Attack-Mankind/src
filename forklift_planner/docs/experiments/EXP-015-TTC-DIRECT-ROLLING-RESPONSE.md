# EXP-015：普通道路 TTC 单次应激与稳定优先级回归

## 1. 实验身份

- 日期：2026-08-20
- Git HEAD：`d847a701edc3508055747743a86daec57e3445ac`
- 工作区：实验开始前已有用户暂存修改（EXP-013、EXP-014、EXP014 日志及相关动态协调源码/测试）；本实验未 reset、checkout、覆盖或重新暂存这些修改。
- 环境：WSL `Ubuntu-20.04-ros`，ROS Noetic，`~/stage32_ws`，Release，C++17。
- 目的：验证普通道路协调改为“15 s NOMINAL baseline 只分类风险，约 2 s 周期内按 TTC 单次请求动作”，并验证去除 full-horizon candidate ladder、alternate winner 和 wait-time priority flip 后的安全性与吞吐。
- 结论：**FAIL**。代码级测试通过，但三组正式实验均发生一次 hard-guard collision，随后形成持续至 120 min 结束的双车楔死。

## 2. 修改范围

- `dynamic_speed_coordination.h/.cpp`：删除 ordinary candidate ladder、候选 CLEAR 接受条件、alternate-order/winner swap 及相应结果字段；保留 `evaluateSelectedAction()` 诊断工具和制动余量工具；直接映射 FAR/NOMINAL、MID/YIELD、NEAR/CREEP、braking unsafe/STOP。
- `rule_engine.cpp/.h`：普通稳定 priority 改为 deadlock-breaker、slot dependency 前置后使用 `(loaded ? 0 : 1, task_count, id)`；`wait_time` 保留但不参与 ordinary priority；保留多 pair 最严格请求合并、A1/slot/forward-clearance 后置加严和 hard guard；删除 ordinary candidate rollout/换序日志字段。
- `multi_vehicle_patrol_node.cpp`：仅适配已删除的 candidate rollout 结果字段及日志，不改变 15 s/2 s 仿真框架。
- `planner_param.yaml`、`multi_vehicle_config.h`：只同步参数注释，不修改参数值。
- 三个动态协调 focused tests：覆盖直接 MID/NEAR、无换序、braking STOP、wait-time 稳定 priority、loaded/task-count/id、slot/deadlock 前置、多 pair 聚合、零速 NOMINAL 重启预测、forward/A1/ordinary reservation/2 s freeze 回归。
- 未修改 `spatiotemporal_interaction.cpp`、`task_allocator.cpp`、A1 service-owner 机制、安全阈值或任务/路径策略。

## 3. 正式生效配置

三组配置完全相同，仅 `random_seed` 不同：

- `vehicle_count=2`
- `batch_minutes=120`，`update_rate=10 Hz`，请求并完成 `72000` 个真实仿真 tick，`real_sim_t=7200.0 s`
- `prediction_horizon=15.0 s`，`prediction_step=0.05 s`
- `rolling_horizon=15.0 s`，`rolling_refresh_period=2.0 s`
- `nominal_speed=0.20 m/s`，`yield_ratio=0.50`（YIELD `0.10 m/s`），`creep_ratio=0.25`（CREEP `0.05 m/s`）
- `max_accel=0.20 m/s^2`，`max_decel=0.30 m/s^2`
- `dynamic_speed_far_threshold=10.0 s`，`dynamic_speed_near_threshold=5.0 s`
- `reproducible_task_random=true`，`simple_forward_demo=false`，`real_mode=false`

## 4. 构建与 focused tests

命令：

```bash
source /opt/ros/noetic/setup.bash
cd ~/stage32_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
cd build
ctest --output-on-failure
```

结果：Release 构建通过；CTest 注册的 7/7 测试通过：

- `conflict_zone_closure_test`
- `future_a1_policy_test`
- `spatiotemporal_interaction_test`
- `dynamic_speed_coordination_test`
- `dynamic_speed_rule_engine_test`
- `prediction_execution_consistency_test`
- `rolling_decision_timing_test`

备注：工程未生成 `run_tests_forklift_planner` 聚合 target，因此使用 CTest 执行实际注册测试；这不是测试失败。

## 5. 正式实验命令

每个 seed 只执行一次成功进入仿真并完成 72000 tick 的正式运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/stage32_ws/devel/setup.bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=120 vehicle_count:=2 random_seed:=SEED \
  coord_log_file:=/home/lsj/stage32_ws/src/forklift_planner/logs/EXP015/seedSEED_coord.log \
  debug_log_dir:=/home/lsj/stage32_ws/src/forklift_planner/logs/EXP015/debug_SEED
```

正式日志：`forklift_planner/logs/EXP015/`。在 seed 2024 正式运行前有一次未 source devel workspace 的启动错误；该进程在 ROS 节点和仿真 tick 启动前退出，随后日志被成功正式运行覆盖，不计为正式实验。

## 6. 正式结果总表

| seed | ticks / sim time | hard guard / first | deadlock ticks / first | max wait | V0/V1 tasks | priority flips | wedge / reciprocal short cycles |
|---|---:|---|---|---|---|---:|---|
| 2024 | 72000 / 7200.0 s | 1 / tick 1405, 140.5 s | 14069 / tick 1656, 165.6 s | 7069.1 s (V0) | 2 / 1 | 1 | 1 / 0 |
| 2025 | 72000 / 7200.0 s | 1 / tick 6815, 681.5 s | 12987 / tick 7066, 706.6 s | 6526.4 s (V1) | 7 / 10 | 2 | 1 / 0 |
| 2026 | 72000 / 7200.0 s | 1 / tick 1739, 173.9 s | 14002 / tick 1991, 199.1 s | 7035.6 s (V1) | 2 / 2 | 0 | 1 / 0 |

Priority flip 的统计口径是相邻 ordinary `[DYN-SPEED]` 决策的 `priority_vehicle` 变化。2024 为 V0→V1 一次；2025 为 V0→V1→V0 两次；2026 为零。变化之间存在任务/航段推进，不存在连续 rolling period 的 wait-time 自激往返，且源码 priority 已不读取 `wait_time`。

## 7. 动态协调、请求和 A1 指标

| seed | FAR/MID/NEAR | braking emergency STOP | target N/Y/C/S | ordinary create | rolling_order_swap | A1 executed create/delete | A1 owner create/change/release |
|---|---|---:|---|---:|---:|---|---|
| 2024 | 2 / 3 / 3532 | 3530 | 104 / 3 / 2 / 7101 | 0 | 0 | 4 / 4 | 4 / 0 / 4 |
| 2025 | 7 / 7 / 3266 | 3260 | 571 / 7 / 6 / 6666 | 0 | 0 | 14 / 14 | 18 / 0 / 18 |
| 2026 | 2 / 1 / 3517 | 3514 | 133 / 1 / 3 / 7077 | 0 | 0 | 4 / 4 | 5 / 0 / 5 |

- A1 service owner `change=0`，create/release 成对，实验结束 `active_holds=0`，没有发现 owner 异常切换。
- ordinary `ConflictReservation create=0`；保留的 reservation 全部是 A1 special protection。
- 日志与源码中 `rolling_order_swap=0`。
- 三组均在首次碰撞后长期 `first_t=0`，braking emergency STOP 持续，任务吞吐基本停止。
- `reciprocal_stop_cycles=0`，未发现 STOP→GO→STOP 的短周期“毛毛虫”震荡；相反，三组都是一次 wedge 后长期 STOP/STOP。
- 当前没有独立的“真实执行帧 forward-clearance override 次数”计数器。evaluated `duplicate_pair_authority`（70604/65200/70266）包含每个 rollout 的 20 个可执行预测帧，不能当作真实 override 次数。计划级 post-tightening `creep_to_stop` 为 1/0/1；首次事故历史中 2024、2025 明确出现 `emergency_next_step`，2026 未在碰撞前触发。精确 executed override 次数：**未知，需要确认**。

## 8. 首次失败现场

### seed 2024

- tick/sim_t：1405 / 140.5 s；车辆 V0-V1。
- mission phase：V0 `TO_A1`（slot 46→-1），V1 `TO_B`（slot 10→49）。
- plan 75：priority V1，`first_t=1.300 s`，pair requested `CREEP/NOMINAL`，braking STOP=false，A1 related=false。
- tick 1404：forward clearance 将实际动作加严为 V0/V1 `STOP/STOP`，blocker 互指；tick 1405 hard guard 检出已重叠。
- 后续：plan 76 起 `first_t=0`，V0 braking STOP、V1 NOMINAL 的 ordinary 请求再被安全层压成 STOP/STOP，直至结束。

### seed 2025

- tick/sim_t：6815 / 681.5 s；车辆 V0-V1。
- mission phase：V0 `TO_B`（slot 31→63），V1 `TO_A1`（slot 57→-1）。
- plan 365：priority V0，`first_t=3.800 s`，pair requested `NOMINAL/CREEP`，braking STOP=false，A1 related=false。
- tick 6814：V0 被 forward clearance 加严为 STOP（blocker V1）；tick 6815 V1 也被加严为 STOP，但 hard guard 同拍检出重叠。
- 后续：`first_t=0` 并长期 STOP/STOP，直至结束。

### seed 2026

- tick/sim_t：1739 / 173.9 s；车辆 V0-V1。
- mission phase：V0 `TO_B`（slot 28→64），V1 `TO_A1`（slot 63→-1）。
- plan 94：priority V0，`first_t=0.850 s`，pair requested `NOMINAL/STOP`，braking STOP=true，A1 related=false。
- 碰撞前 V1 已实际 STOP；V0 仍 NOMINAL，forward clearance 未提前加严，tick 1739 hard guard 检出 V0 驶入 V1 足迹并将双方 STOP。
- 后续：`first_t=0` 并长期 STOP/STOP，直至结束。

## 9. 直接代码根因与下一步统一建议

共同根因位于普通单次应激与后置安全层之间：

1. `evaluatePairSpeedCoordination()` 固定让 priority 侧对本 pair 保持 NOMINAL，只把 FAR/MID/NEAR/braking 动作给 yielding 侧。
2. `hasInsufficientBrakingMargin()` 比较“当前速度的制动距离 + 当前速度×2 s 前缀”和冲突入口可用距离；它判断的是现在若制动是否尚可停车，并未验证选中的非零 CREEP 在完整 2 s 执行窗内仍留在安全停止线外。
3. `enforceForwardClearance()` 只比较下一 `dt=0.1 s` 的两个足迹，而不是完整 2 s committed prefix 的制动包络。因此 seed 2024/2025 到重叠前一拍才 STOP；seed 2026 中 yielding 已停在 priority 的未来扫掠区，priority 仍获 NOMINAL。
4. hard guard 在实际足迹重叠后把双方 STOP，安全动作不能解除几何楔死；deprecated 重规划/旧死锁恢复保持关闭，所以长期 `first_t=0`。

建议的下一步最小、统一修复（本实验未实现）：保持无 15 s candidate optimizer、无 alternate winner、无 ordinary reservation；把 braking/post-safety 改为检查完整 2 s committed prefix 的同步足迹与安全停止线。若 yielding 的本周期动作会在 refresh 前进入冲突区，则提前 STOP；若从当前状态已经无法在冲突区外停住，则在进入重叠前同步收紧 priority 侧。该修复必须重新做 focused tests 和独立正式实验，不能复用本次失败结果宣称通过。

