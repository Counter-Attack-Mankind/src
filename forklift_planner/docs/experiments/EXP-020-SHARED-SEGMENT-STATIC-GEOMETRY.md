# EXP-020：阶段一静态 SharedSegmentCandidate 构建

## 基线与范围

- 日期：2026-08-24
- 基线 commit：`4a6038f0c54a5052a66d4eb076237f20b271a4c6`
- 工作区：保留任务开始时已有修改；本实验只新增静态 shared geometry、薄缓存/日志/RViz 接线和 focused test。
- 正式业务：`B -> A1 -> B`；本次聚焦初始 `B42/B50 -> A1` 航段。
- 明确未实现：realtime relevance、EntryGuard、occupancy、owner、reservation、动作/TTC/priority/rolling frozen target 修改。

## 实验目的

验证独立 `shared_segment_geometry` 模块能够按连续 `WpType` 划分 traversal，先按 traversal pair 分组，再以 local motion heading 的 `direction_dot < -0.5` 过滤，并在 `(sA,sB)` 规则采样网格上用确定性 8 邻域构建最大连通分量。确认新增结果仅用于静态诊断，不改变 ordinary dynamic coordination 行为。

## 实现与参数

- 新模块：`include/forklift_planner/multi_vehicle/shared_segment_geometry.h`、`src/multi_vehicle/shared_segment_geometry.cpp`
- 新测试：`test/shared_segment_geometry_test.cpp`
- 采样步长：沿用 legacy full scan 的 `0.025 m`
- OBB margin：沿用 legacy full scan 的 `conflict_margin / 2 = 0.020 m`
- local opposing：`direction_dot < -0.5`
- connectivity：同一 traversal pair 内的规则二维 grid 8 邻域
- 未增加最小长度、最小样本数或最小面积过滤。
- 未修改任何 YAML、地图、车辆几何、动力学或安全阈值。

## 构建与测试命令

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON --pkg forklift_planner
cd build
ctest --output-on-failure
```

结果：Release 构建成功；CTest 8/8 通过，包括新增 `shared_segment_geometry_test`。新增测试覆盖直线 F/R、cusp 分块、world-space 重复 traversal、连续 FORWARD 转弯、opposing/non-opposing/opposing 语义间隙、REVERSE 转弯逐 sample heading、非对向 overlap 排除及重复计算 ID 确定性。

## B42/B50 固定种子复现

- 车辆数：2
- `start_slots=[42,50]`
- `use_a1_cycle=true`
- `random_seed=2026`，`reproducible_task_random=true`
- 航段：V0 `B42 -> A1`，V1 `B50 -> A1`
- batch：600 tick，即 1 仿真分钟；未运行 120 min
- 路径目录：复用 EXP-018 已保存的 A1 catalog，避免路径生成差异

静态输出：

| ID | traversal | direction | V0 s | V1 s | dot | samples |
|---:|---|---|---|---|---|---:|
| 0 | 0 / 0 | R / R | [0.600, 1.700] | [0.350, 1.475] | [-1.000, -0.538] | 715 |
| 1 | 1 / 1 | F / F | [1.925, 2.150] | [1.700, 1.900] | [-0.758, -0.526] | 31 |
| 2 | 1 / 1 | F / F | [4.875, 5.200] | [4.375, 4.625] | [-0.894, -0.501] | 64 |

结果与 EXP-019 静态审查参考完全一致。raw0 不再跨双方 R/F cusp；R/R 与 F/F 分属独立 traversal pair；同一 F/F traversal pair 内的两个二维连通分量也保持独立。

## ordinary coordination 基线对比

对 EXP-019 与 EXP-020 的关键日志逐行比较：

- `ROLLING-DECISION`：30 vs 30，逐行差异 0
- `DYN-SPEED`：30 vs 30，逐行差异 0
- `FUTURE_A1`：30 vs 30，逐行差异 0
- `FIRST-COLLISION`：4 vs 4，逐行差异 0
- rolling target 计数：`NOMINAL/YIELD=2`、`NOMINAL/CREEP=1`、`NOMINAL/STOP=3`、`STOP/STOP=24`，两次实验一致
- ordinary reservation：`created=0`、`not_created=30`，两次实验一致
- hard guard：均在 tick 95、sim_t=9 s 首次触发，batch 结束均为 `hard_guard=1`

hard guard 仍触发是既有 B42/B50 基线行为，不是阶段一静态 geometry 的 FAIL。新增 3 条 `[SHARED-SEGMENT-STATIC]` 明确带有 `static_prior=true control_input=false`。

## RViz 与运行产物

实时模式按现有 `forklift.rviz` 配置实际启动 RViz，并录制：

`forklift_debug_bags/EXP-020-B42-B50-STATIC/rviz_markers_realtime.bag`

- bag 时长：13.2 s，大小 8.8 MB
- `/forklift_planner/markers`：138 条 `visualization_msgs/MarkerArray`
- 每个稳定帧包含 3 个 `shared_segment_candidate_aabb` 和 3 个 `shared_segment_candidate_explanation` marker
- 同时保留 legacy `potential_conflict_zone_overlap`/`conflict_zone_aabb` 与 timed red overlap marker 命名空间
- 新 marker 为半透明紫色诊断 AABB，标签包含 candidate/traversal/direction/path-s/dot/sample count，并注明 `AABB diagnostic only`

其他产物：`coord_timeline.log`、`patrol_stdout.log`、`coord_realtime.log`、`patrol_realtime_stdout.log`、RViz/rosbag/roscore 日志。

## 结论

PASS。阶段一静态 SharedSegmentCandidate 构建、缓存、日志和 RViz 诊断满足验收；ordinary dynamic action、priority、TTC、A1、reservation、forward clearance、hard guard、任务与路径生成语义未改变。
