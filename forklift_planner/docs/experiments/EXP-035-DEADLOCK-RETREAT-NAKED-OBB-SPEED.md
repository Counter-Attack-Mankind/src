# EXP-035 Deadlock retreat 裸车身 OBB 与专用速度

- 日期：2026-09-07
- 分支：`fix`
- 基线 commit：`e9f2ad738c36`
- 本轮未执行 reset、checkout、stash、stage 或 commit。

## 范围

- `DeadlockManager::retreatSweepClear()` 的 retreat sweep 与其他车辆 OBB 改为 `makeBody(..., 0.0)`。
- `DeadlockManager::evaluateRetreat()` 的 pass corridor 和 retreat 最终停车 OBB 改为 `makeBody(..., 0.0)`。
- 保留 sweep 采样、搜索离散化、`targetClearsCorridor()`、`retreatSweepClear()` 与 `pass_clear_s` 计算。
- 新增 `deadlock_retreat_speed=0.10 m/s`，仅用于仿真/实车 `RecoveryMotion::RETREAT` 和预计回退时间；普通 CREEP 不变。

`deadlock_retreat_clearance` 仍用于冲突 corridor 的 s 边界与 `pass_clear_s`，不再用于 OBB 膨胀。

## 验证

WSL `Ubuntu-20.04-ros` / ROS Noetic / Release：

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
cd build/forklift_planner
ctest --output-on-failure
```

- 完整构建：成功。
- CTest：10/10 通过，0 失败，1.96 s。
- 未运行长时 batch、RViz 或实车试验；实车端到端效果未知，需要确认。
