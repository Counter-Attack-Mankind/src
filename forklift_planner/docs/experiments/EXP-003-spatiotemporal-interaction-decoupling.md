# EXP-003 阶段一：时空冲突检测解耦

- 基线 Git commit：`b99802035ae665707e89f4653d4bd115bc818cb3`
- 基线工作区状态：仅存在用户未跟踪文件 `11.md`；本实验未修改该文件
- 目的：抽取 baseline prediction、同步 OBB overlap、首段 TimedEvent 和 ConflictZone 关联，不改变后续协调决策
- 参数变化：无
- 适用模式：仿真多车协调；未验证实车链
- 车辆数：8
- 固定随机种子：`2026`
- batch 时长：10 仿真分钟，6000 ticks

## 修改范围

- `spatiotemporal_interaction.h/.cpp`：纯预测与纯时空交互检测
- `rule_engine.h/.cpp`：旧入口消费检测结果；holder、reservation、A1、制动和 STOP 决策保持原链路
- `spatiotemporal_interaction_test.cpp`：10 组聚焦测试
- `CMakeLists.txt`：接入新模块和测试目标

未修改参数、`multi_vehicle_patrol_node.cpp`、Future A1/Departure Cluster、前向净空、硬碰撞保护、deadlock、rolling horizon 或任务分配逻辑。

## 验证命令与结果

构建：

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCATKIN_ENABLE_TESTING=ON
```

结果：通过。`multi_vehicle_patrol_node` 和 `spatiotemporal_interaction_test` 编译、链接成功。构建报告 WSL/Windows 文件时间差导致的 clock-skew 警告，无编译或链接错误。

测试：

```text
cd /mnt/d/desktop/叉车/build/forklift_planner
ctest --output-on-failure
```

结果：3/3 通过：

- `conflict_zone_closure_test`
- `future_a1_policy_test`
- `spatiotemporal_interaction_test`

## 固定种子修改前后对照

基线从 commit `b998020` 导出到隔离的 WSL `/tmp` 工作区构建；没有切换、覆盖或清理当前工作树。基线和当前实现均执行：

```text
roslaunch forklift_planner multi_vehicle_batch.launch minutes:=10
```

共同配置：8 车、`random_seed=2026`、`reproducible_task_random=true`。

| 指标 | 修改前 | 修改后 |
|---|---:|---:|
| ticks / sim time | 6000 / 600.0 s | 6000 / 600.0 s |
| hard guard 事件 | 0 | 0 |
| deadlock 检出拍 | 879 | 879 |
| deprecated 重规划恢复 | 0 | 0 |
| 最大等待 | V6 / 600.0 s | V6 / 600.0 s |
| 各车任务计数 | `[1,1,1,0,0,0,0,0]` | `[1,1,1,0,0,0,0,0]` |
| 完整协调日志 SHA-256 | `87f148a26870281943c3dfc44091ac2abde45a01ec2292e52ab66427b736aba2` | 同左 |

完整协调日志逐字节一致，因此日志覆盖的 action、requested_action、blocker、reason、ConflictReservation 生命周期、Future A1/Departure Cluster 事件和任务序列均一致。当前实现以相同输入重复运行一次，协调日志 SHA-256 仍为同一值。

## 观察与结论

- 阶段一结果：保留。
- 修改前已存在长期等待与 deadlock 诊断（V6 等待 600.0 s、879 个检出拍）；修改后完全相同。本阶段按要求不处理该策略问题。
- 未执行 RViz 人工目视检查；固定 seed batch 的终态轨迹/协调日志和 hard guard 统计作为本次等价轨迹核对证据。
- 实车构建与运行：未执行；本阶段没有修改实车接口或实车保护链。
