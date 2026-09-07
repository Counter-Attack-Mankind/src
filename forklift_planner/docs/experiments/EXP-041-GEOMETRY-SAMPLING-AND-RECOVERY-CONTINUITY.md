# EXP-041 几何采样精度与 deadlock recovery 速度连续性

- 日期：2026-09-07
- 分支：`fix`
- 基线 commit：`010392e0f8fe558b32c672491b0a8d1c89b84b72`
- 本轮未执行 reset、checkout、stash、stage 或 commit。
- 修改前工作区已有 `deadlock_manager.cpp` 注释性未提交改动；本实验保留这些改动。

## 实验目的

- 将 bridge relation backtrack 和静态 OBB extension 的空间采样从 `0.025 m` 细化为 `0.01 m`。
- 将 deadlock retreat sweep 和 pass corridor 的空间采样从 `0.02 m` 细化为 `0.01 m`，保持退让目标搜索步长 `0.05 m`。
- 保持主运动学预测步长 `0.05 s`，在同步 OBB 检查阶段依据二维相对速度自适应细分，限制相邻检查的相对位移约为 `0.01 m`，检查时间步长限制在 `[0.005, 0.05] s`。
- 在首次几何可行 retreat candidate 后额外回退 `0.02 m`，并重新检查 corridor 和 retreat sweep。
- 仅对 `RecoveryMotion::HOLD` 清零速度，允许 RETREAT/PASS 保持速度连续性。

## 参数变化

| 参数 | 原值 | 新值 | 单位 | 适用范围 |
|---|---:|---:|---|---|
| `bridge_backtrack_step` | 0.025 | 0.01 | m | bridge relation/static OBB 回溯 |
| `path_validation_step` | 0.02 | 0.01 | m | deadlock sweep/corridor；同时影响既有路径验证消费者 |
| `prediction_step` | 0.05 | 0.05 | s | 主预测保持不变 |
| `deadlock_retreat_search_step` | 0.05 | 0.05 | m | retreat target 搜索保持不变 |
| `deadlock_retreat_clearance` | 0.02 | 0.02 | m | 同时用于 corridor s 延伸和可行 candidate 后的安全回退 |
| `deadlock_retreat_speed` | 0.10 | 0.10 | m/s | RETREAT 专用速度保持不变 |

## 验证

环境：WSL `Ubuntu-20.04-ros`、ROS Noetic、Release。

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
```

- `git diff --check`：通过。
- `catkin_make -DCMAKE_BUILD_TYPE=Release`：通过，`multi_vehicle_patrol_node` 及相关测试目标均成功链接。
- 按本轮要求未运行 CTest。
- 按本轮要求未运行 batch、RViz 或实车回归。
- 因未执行运行回归，碰撞、死锁解除、长期等待、任务完成数和实车端到端效果均为未知，需要确认。

