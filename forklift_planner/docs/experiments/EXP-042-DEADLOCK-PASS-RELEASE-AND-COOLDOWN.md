# EXP-042 Deadlock PASS 释放与单车重启等待

- 日期：2026-09-07
- 分支：`fix`
- 基线 commit：`154ff04`
- 目的：修正 RETREAT 完成复核、PASS 释放语义、ABORT 清理、retreat 单车重启等待及 RViz/ROS 诊断。

## 修改范围

- `DeadlockManager`：到达 retreat 目标容差后，以当前实际 `path_s` 的裸 OBB 重新检查 passer 前向通道；PASS 仅保留一个协调周期作为释放确认，不再以 `pass_clear_s` 作为结束条件。
- `DeadlockManager`：CLEAR 后立即销毁 pair transaction，并对 retreat 车辆施加 `rolling_refresh_period`（当前 2 s）的独立 cooldown；ABORT 记录后立即清除旧 transaction/directive。
- `RuleEngine`：PASS 不再跳过普通 pairwise，也不再清除 passer 的普通 TTC hold 或强制 NOMINAL；cooldown 仅约束 retreat 车辆。
- `MarkerPublisher`：接收 `RecoveryDirective`，显示 `RETREAT/PASS/HOLD/DEADLOCK_COOLDOWN` 和 cooldown 剩余时间；pair-level `rolling_emergency_stop` 不再作为单车 emergency 标签。
- deadlock 关键事件 `CONFIRMED/SELECT/RETREAT_DONE/PASS_START/CLEAR/UNRESOLVED/ABORT` 同步输出 `ROS_WARN`。
- 更新 `deadlock_manager_test` 的编译期接口和断言语义，不再依赖已删除的 PASS pairwise override。

未修改 TTC 阈值、priority、A1、路径生成、hard guard、retreat 搜索几何或 `pass_clear_s` 的候选评估用途。

## 验证

环境：WSL `Ubuntu-20.04-ros`，ROS1 Noetic。

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
```

- `catkin_make -DCMAKE_BUILD_TYPE=Release`：通过；`multi_vehicle_patrol_node`、`deadlock_manager_test` 及其余目标成功编译链接。
- `git diff --check`：通过。
- 按本轮既定要求未运行 CTest、batch、RViz 或实车回归。
- 因未执行运行回归，实际 deadlock 解锁时序、cooldown 可视化效果及实车行为仍为“未知，需要确认”。

