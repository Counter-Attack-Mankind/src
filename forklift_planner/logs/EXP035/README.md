# EXP-035：仿真车辆间 OBB 改用裸车身

- 基线提交：`f42a5c9`
- 日期：2026-08-28
- 目的：验证 15 s future-future、priority future-vs-current、bridge geometric extension、静态 ConflictZone 与其 RViz polygon 统一使用裸车身 OBB。
- 参数变化：无；`conflict_margin` 配置值未修改。

## 构建与测试

- `catkin_make -DCMAKE_BUILD_TYPE=Release --pkg forklift_planner`：通过。
- 通过：`spatiotemporal_interaction_test`、`bridge_ttc_correction_test`、`prediction_execution_consistency_test`。
- 未通过：`dynamic_speed_coordination_test`、`dynamic_speed_rule_engine_test`、`rolling_decision_timing_test`。失败断言是依赖旧膨胀 OBB 碰撞时刻的 FAR/MID 固定场景；本轮受修改范围限制，未调整测试夹具。

## 场景结果

- Case A：`vehicle_count=2, random_seed=2026, start_slots=[22,20]`，运行 60 s。初始首次 overlap 从 `t=0, s=0` 变为 `t=2.450 s, s_a=s_b=0.332 m`，priority physical TTC 为 CLEAR；但随后在 tick 18 / sim 1 s 出现真实碰撞。
- 固定基线：`vehicle_count=2, random_seed=2025, start_slots=[42,50]`，运行 900 s。完成 9000 tick，但在 tick 477 / sim 47 s 出现真实碰撞，batch 以失败状态退出。

## 结论

目标几何替换已实现且编译通过，但协调安全回归未通过，当前改动不能据此认定为已验证安全。日志保存在本目录。
