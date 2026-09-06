# EXP-038：临时禁用 row_id=1 目标派发并扩展 next-A1 ETA

## 基线与目的

- Git 基线：`bb9682a`（分支 `fix`）。
- 开始时已有用户修改：`src/path_generator_routes/a1_to_b/path_generator_route.cpp`，本实验未修改。
- 验证期间观察到 `config/planner_param.yaml` 的 `a1_owner_horizon` 已由 45 s 变为 15 s；该变化不是本实验实现内容，未回退，batch 按 15 s 执行。
- 目的：临时禁止新任务选择 `row_id == 1` 的 B 库位，同时保留地图、路径与 B→A1/A1→B cache；把 Future A1 首次 owner 候选扩展到 next-A1 ETA 已知的 `TO_A1`、`TO_B`、`UNLOAD_DWELL`。

## 修改范围

- `TaskAllocator::dispatchTargetEnabled()`：统一只读目标开关，当前规则为 `map_.slots().at(slot).row_id != 1`。
- `chooseNextTarget()`、`nearestForwardTargetLen()`、`forwardTargets()`、`prepareDropoffLeg()` 的 exit 候选、普通首次 preset 和 A1 首次 preset：统一过滤禁用目标。
- `tryPlan()`、`tryPrepareFromA1()`：增加最终防御检查，避免未来新增调用者绕过选择器。
- `assignPickupLeg()`、地图、路径生成和 A1 leg cache 构建未修改，因此当前位于 row1 的车辆仍可按现有 B→A1 cache 离开。
- `TaskAllocator::getPickupLegTrack()`：只返回已构建且有效的 B→A1 cache，不调用任务分配、RNG 或路径生成器。
- `A1Coordinator::predictA1Arrivals()`：
  - `TO_A1`：当前 B→A1 剩余轨迹时间；
  - `UNLOAD_DWELL`：`dwell_remaining + current_slot` 的完整 B→A1 时间；
  - `TO_B`：当前 A1→B 剩余时间 + `unload_dwell_time` + `target_slot` 的完整 B→A1 时间；
  - 每个新 B→A1 段以速度 0 开始，并复用 NOMINAL/曲率限速/加减速 callback；最终仍由 `ETA <= a1_owner_horizon` 过滤。
- `FutureA1Commitment` 对 `TO_B/UNLOAD_DWELL` 首次候选绑定下一条 pickup leg 的 `path_gen = current + 1`，并在 `TO_B -> UNLOAD_DWELL -> TO_A1` 期间保持锁定。既有 `TO_A1 -> PICKUP_DWELL -> TO_B` 锁定、arrival 排序和 unified-priority tie break 未改。

## 验证

### 构建与定向测试

```text
wsl -d Ubuntu-20.04-ros -- bash -lc "source /opt/ros/noetic/setup.bash && cd '/mnt/d/desktop/叉车' && catkin_make --pkg forklift_planner"
```

- 结果：PASS；`multi_vehicle_patrol_node` 和相关测试目标构建成功。
- 环境提示：Windows/WSL 文件时间存在约 14 s clock-skew warning；未出现编译错误。

```text
ctest --output-on-failure -R '(future_a1_policy_test|dynamic_speed_rule_engine_test|rolling_decision_timing_test|spatiotemporal_interaction_test)'
```

- 结果：4/4 PASS。
- 新增定向覆盖：`TO_B` next-A1 ETA、下一服务 `path_gen`、跨 `UNLOAD_DWELL` 锁定和进入 `TO_A1` 后的 generation 交接。

### 固定 seed batch

参数：2 车，start slots `[38,20]`，seed `2026`，1 仿真分钟，`dt=0.1`，A1 owner horizon 15 s；相同输入重复两次。

- 两次均完成 600/600 ticks。
- 两次均为 hard guard 0、wedge episodes 0、异常退出 0。
- 两次任务/动作/A1 汇总一致；协调日志 SHA-256 均为 `D65851E4DAB7EEB1D7A98A8C200BA0E20D3B82C95A7989409DB6E93B4E8B7E44`。
- A1→B 新目标序列均为 `V0:B9, V1:B52, V0:B30`。
- `row_id=1` 对应 slot `10,12,14,16,18`；两份 `[PREPARE_DROPOFF]` 日志均未出现这些目标。首次 preset `V1:B10` 未覆盖过滤，实际选择 `B52`。
- 产物：`logs/EXP038-seed2026-run1-coord.log`、`logs/EXP038-seed2026-run2-coord.log`。

## 结论与限制

- 本次最小修改保留，结论为 **YES WITH CHANGES**；必须使用下一服务 path generation 才能让提前选中的 owner 正确跨越 `TO_B/UNLOAD_DWELL`。
- 未修改安全阈值、路径拒绝条件、地图、路径 cache 或 deprecated 策略。
- 未执行 RViz/等价人工轨迹核对，也未执行长时多车压力回归；1 分钟、2 车 batch 不能视为完整覆盖。实车行为未知，需要确认。
