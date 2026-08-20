# EXP-016：ordinary NEAR TTC STOP 边界修正

## 实验身份

- 日期：2026-08-20
- Git HEAD：`d847a701edc3508055747743a86daec57e3445ac`
- 工作区：开始前已有用户 staged/unstaged 修改和 EXP-015 产物；本轮未 reset、checkout、覆盖、暂存或提交这些修改。
- 环境：WSL `Ubuntu-20.04-ros`，ROS Noetic，`~/stage32_ws`，Release，C++17。
- 目的：仅将 ordinary NEAR 的主 STOP gate 从 path-space 判断改为 baseline TTC 时间边界；不修改 full-clear、winner、priority、A1、reservation、forward clearance、hard guard、15 s/2 s 框架或 task allocator。

## 参数记录

| 参数 | 原值 | 新值 | 单位 | 适用模式 | 原因 |
|---|---|---:|---|---|---|
| `dynamic_stop_time_margin` | 不存在 | 0.10 | s | 仿真/实车共享 ordinary NEAR 判断 | 给一轮决策等待时间和 CREEP 制动时间增加小型显式裕度；避免复用语义不明且数值较大的旧时间字段 |

该安全参数由本轮负责人请求明确授权新增。`emergency_time=0.80`、`final_decision_time=2.00`、`warning_time=5.00` 当前只有配置读取、没有运行消费者，也没有“STOP 时间裕度”的明确接口语义，因此未复用、未改值。

## 旧逻辑核查

旧 `hasInsufficientBrakingMargin()` 只有 ordinary `RuleEngine::resolvePairwiseConflicts()` 和一个 focused test 两处调用。它用 yielding 车：

```text
available_distance <= current_speed * max(dt, rolling_refresh_period)
                      + current_speed^2 / (2 * max_decel)
```

它不直接使用 baseline `first_t`。EXP-015 plan 75 中 yielding V0 已降至 0.05 m/s，旧公式的 required distance 只有约 `0.05×2 + 0.05²/(2×0.30) = 0.104 m`；当 path-space available distance 大于该值时就得到 false，与此同时同步 OBB `first_t` 已只有 1.3 s。旧日志没有输出 `zone.s_*_enter`，因此不能从 `event_exit` 反推当时精确 available distance。

## 新逻辑

新增 `evaluateTtcStopBoundary(first_t, planned_action, config)`：

```text
action_speed  = speed(planned_action)
braking_time  = action_speed / max_decel
stop_threshold = rolling_refresh_period
               + braking_time
               + dynamic_stop_time_margin
stop_required = first_t <= stop_threshold
```

仅 ordinary NEAR 调用，`planned_action=CREEP`。FAR 仍 NOMINAL，MID 仍 YIELD；不重新预测 CREEP，不计算 conflict delay，不创建 ordinary reservation。

正式配置下：`2.0 + 0.05/0.30 + 0.10 = 2.267 s`。

## 构建与测试

命令：

```bash
source /opt/ros/noetic/setup.bash
cd ~/stage32_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
cd build
ctest --output-on-failure
```

结果：Release 构建通过；CTest 7/7 通过。focused tests 覆盖：

- NEAR `first_t > threshold` 不 STOP；
- NEAR `first_t < threshold` STOP；
- `first_t == threshold` STOP；
- `rolling_refresh_period`、`creep_ratio`、`max_decel` 改变时阈值自动改变；
- MID 仍单次 YIELD，FAR 仍 NOMINAL；
- 无 alternate order / `rolling_order_swap`；
- ordinary reservation 为零；
- A1、2 s rolling freeze、同步 OBB、预测/执行一致性测试通过。

## 单次短时 smoke

- 命令：`multi_vehicle_phase2_batch.launch minutes:=4 vehicle_count:=2 random_seed:=2024`
- 完成：2400/2400 tick，真实仿真时间 240.0 s。
- 日志：`forklift_planner/logs/EXP016/smoke2024_console.log`、`smoke2024_coord.log`。
- 阈值证据：
  - plan 74：`first_t=4.400`，CREEP 0.050 m/s，decision 2.000 s，braking 0.167 s，margin 0.100 s，threshold 2.267 s，`stop_required=false`，选择 CREEP/NOMINAL。
  - plan 75：`first_t=1.300`，同一 threshold 2.267 s，`stop_required=true`，选择 STOP/NOMINAL。
- ordinary reservation create=0，`rolling_order_swap=0`，A1 service create/release=4/4、owner change=0。

## smoke 异常与结论

smoke 仍在 tick 1405（约 140.5 s）发生 V0-V1 hard-guard collision，随后 first_t=0、长期 STOP/STOP，并在 165.6 s 检出 deadlock。与 EXP-015 不同的是，V0 已从 plan 75 起由新 TTC gate 实际 STOP 在 `s=0.684`；priority V1 仍以 NOMINAL 前进，直到 tick 1404 才被既有 `emergency_next_step` 停止，下一拍 hard guard 检出重叠。

结论分两层：

1. **TTC STOP 边界实现通过**：公式、参数读取、包含等号、档位速度和日志均符合本轮定义。
2. **系统 smoke 仍失败**：单边 yielding STOP 不能阻止 priority 车驶入其静止足迹。直接关联的是本轮明确禁止修改的 priority pair 语义和仅检查下一 dt 的 forward clearance，而不是 TTC gate 未生效。

本轮不越界增加双边 STOP、full-clear、alternate winner、reservation 或 forward/hard-guard 改造，也不运行三组 120 min。参数与实现暂时保留供负责人审查；尚不能据此声明多车协调回归通过。
