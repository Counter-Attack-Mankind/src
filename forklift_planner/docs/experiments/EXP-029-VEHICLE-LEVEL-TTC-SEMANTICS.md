# EXP-029：ordinary / bridge 车辆级 TTC 语义统一

## 实验身份

- 日期：2026-08-27
- Git HEAD：`56ea0e0`
- 工作区基线：开始前仅有用户未跟踪的 EXP-028 CSV/Markdown；本轮未修改、删除、暂存或提交该产物。
- 目的：区分 pair-level `first_overlap_t` 诊断时间与 A/B 各自到自身 `danger_s` 的车辆级 TTC，修正 ordinary band、yielding STOP 和 residual priority emergency STOP 的数据来源。

## 修改范围

- `TimedConflictEvent` 明确输出 `first_overlap_t`、`collision_s_a/b`、`danger_s_a/b`、`ttc_a/b`。
- `predictionTimeAtS()` 复用 `predictTrajectory()` 生成的运动学样本做插值反演，没有增加第二套速度积分模型。
- ordinary baseline 在 priority 确定后，仅用 yielding vehicle 自身 TTC 分 FAR/MID/NEAR 及计算 STOP boundary。
- residual 仍使用现有同步 OBB 重预测，但输出新的 collision/danger position 和 `residual_ttc_a/b`；priority/yielding STOP 分别读取自己的 TTC 与动作制动阈值。
- bridge correction 分别从 A/B 自身 prediction 反算 `original_ttc`，再各自修正到 bridge boundary；不再把 `first_overlap_t` 复制成两车 original TTC。
- 日志新增 `[DYN-TTC]`，并将 `[BRIDGE-TTC]` 拆成每车 original/corrected danger position 和 TTC。
- CMake focused-test 目标增加已有 source dependency，无新运行依赖。

本轮未改：priority/unifiedPriority、A1 Service Owner、DepartureCluster、SlotDepartureAdmission、ordinary reservation、同步 OBB 几何、2 s/15 s/0.1 s 时间框架、参数值、alternate winner 或死锁机制。

## 构建与 CTest

```bash
source /opt/ros/noetic/setup.bash
cd ~/stage32_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
cd build
ctest --output-on-failure
```

结果：Release 构建通过，CTest 8/8 通过。关键覆盖：

- synchronized detector 对 A/B 各自执行 `predictionTimeAtS(prediction, collision_s)`。
- baseline 中即使 A/B TTC 数值相等，也保留两个独立字段和计算链。
- 构造 `ttc_priority=1.0`、`ttc_yield=6.0` 验证 band 只读取 yielding TTC。
- 构造 `ttc_priority=2.50`、`ttc_yield=3.00`，验证 priority/NOMINAL 进入自身 STOP boundary，yield/CREEP 仍在自身 boundary 外。
- residual priority STOP 断言使用 `residual_priority_ttc` 与 NOMINAL braking threshold。
- bridge 用不同 prediction duration 得到 `original_ttc_a=6.0`、`original_ttc_b=4.0`，证明未复制 pair overlap time。
- `[DYN-TTC]` / `[BRIDGE-TTC]` 关键字段断言通过。

## 确定性 batch 回归

两次使用完全相同命令：

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=4 vehicle_count:=2 random_seed:=2024
```

- 每次 `2400/2400` tick，`real_sim_t=240.0 s`，`dt=0.100 s`。
- 两次 coordination log SHA-256 完全相同：`BD75CBBE71720007EA8E3027A6E9F87EFB35A30156FBC8A1714D2D6BD7E1C5ED`。
- hard guard=0，tasks：V0=2、V1=1。
- FAR/MID/NEAR=2/3/52，target STOP/CREEP/YIELD/NOMINAL=143/1/3/103。
- ordinary reservation create=0，A1 create/release/change=4/4/0。
- 两次均有 deadlock 检出 152 拍、wedge episode=1，V0 max wait=109.1 s；因此不宣称多车系统回归通过。
- 未运行交互式 RViz；coordination log 保存每个 rolling plan 的 A/B path position、danger position、TTC、动作与结尾轨迹快照，作为本次等价轨迹核对证据。

## 产物

- `forklift_planner/logs/EXP029/seed2024_run1_console.log`
- `forklift_planner/logs/EXP029/seed2024_run1_coord.log`
- `forklift_planner/logs/EXP029/seed2024_run2_console.log`
- `forklift_planner/logs/EXP029/seed2024_run2_coord.log`
- `forklift_planner/logs/EXP029/debug_run1/`
- `forklift_planner/logs/EXP029/debug_run2/`

## 结论与剩余风险

本次代码层面的语义修正为 PASS：ordinary/bridge 动作决策中未再发现 pair-level `first_overlap_t` 驱动 FAR/MID/NEAR 或 priority/yielding STOP。

剩余风险：普通同步 first-overlap 的 A/B TTC 在数学上仍经常相等，这是同一预测帧的结果，不是共享变量；bridge boundary 或不同运动状态可得到不同 TTC。batch 仍存在长时间 STOP/STOP 和 deadlock，本轮未设计新死锁机制。
