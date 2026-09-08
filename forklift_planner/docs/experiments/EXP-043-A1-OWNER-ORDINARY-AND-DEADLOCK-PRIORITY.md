# EXP-043 A1 owner 普通与 Deadlock 优先级统一

- 日期：2026-09-08
- 分支：`fix`
- 基线 commit：`773141b`
- 目的：将现有持续 A1 service owner 纳入普通 pair priority，并让 Deadlock retreat/pass 角色服从同一优先级。

## 修改范围

- `RuleEngine::priorityWinner()` 的顺序调整为：目标库位占用者、有效 `FutureA1Commitment.owner_id`、原 `unifiedPriority()`。
- `unifiedPriority()` 未修改，继续保持 loaded、task_count、ID 的确定性全序。
- `RuleEngine::observeDeadlock()` 将当前 `priorityWinner()` 结果随 pair geometry 传入 `DeadlockManager`。
- `DeadlockManager` 仍评估双方 retreat 几何；普通 yielding 车辆可退时直接选为 retreat，仅在其不可退而 priority 车辆可退时执行安全 fallback。
- `SELECT` 日志新增 `selection_reason=priority_yielding_vehicle_retreat` 或 `preferred_retreat_infeasible`。

未修改 A1 owner 竞争、锁定和释放、departure cluster、TTC/bridge、恢复几何、PASS/CLEAR、cooldown、hard guard、RViz 或参数。

## 验证

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
```

- Release 构建通过，`multi_vehicle_patrol_node`、`deadlock_manager_test` 及其余目标成功编译链接。
- 按要求未运行 CTest、batch 或仿真校验；运行行为仍为“未知，需要确认”。

