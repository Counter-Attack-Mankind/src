# EXP-030：B42/B50 bridge 静态 OBB 延伸

## 实验身份

- 日期：2026-08-27
- 基线 HEAD：`ae518f68831b23b68d35b23a6ffde4336599cbfd`
- 工作区：本实验对应的 bridge/RViz 源码改动及本记录尚未提交
- 场景：`random_seed=2025`、`vehicle_count=2`、`start_slots=[42,50]`
- 目的：验证 opposing relation 首次丢失后，局部静态 OBB 连续占用能否把车辆级 bridge TTC boundary 延伸到真实几何上游边界，并验证原 15 min 问题窗口后的任务恢复。

## 修改范围

- `bridge_ttc_correction.h/.cpp`：保留第一阶段 relation backtracking，追加只使用现有 `makeBody()`/`overlaps()` 的局部静态 OBB 延伸；self 下限为实时 `path_s`，other 窗口为当前路径之后、nearest 两侧各一个膨胀车身对角线，首次完整窗口 clear 即停止。
- `rule_engine.h/.cpp`：追加 bridge 几何诊断日志和只读逐车 TTC 显示数据；未修改 priority、动作映射或 residual safety 判定。
- `marker_publisher.h/.cpp`、`multi_vehicle_patrol_node.cpp`：传递并显示既有逐车 TTC；精简冲突文字；未重新执行 OBB 决策计算。
- 未修改配置参数、安全阈值、A1、DepartureCluster、SlotDepartureAdmission、reservation、ConflictZone 或路径生成。

## 构建与 focused test

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg forklift_planner
source devel/setup.bash
devel/lib/forklift_planner/bridge_ttc_correction_test
```

- Release 构建：PASS。
- 现有 `bridge_ttc_correction_test`：PASS。
- 构建输出有 WSL/Windows 文件时间差导致的 clock-skew warning，但目标已重新编译并成功链接。
- 未新增大型 CTest/fixture。

## 正式目标回归

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=17 vehicle_count:=2 random_seed:=2025 \
  start_slot_a:=42 start_slot_b:=50
```

实际完成 `10200/10200` tick，`real_sim_t=1020.0 s`，覆盖 15 min 后继续运行 120 s。

### B42/B50 关键窗口

EXP-028 的旧第一阶段 V1 boundary 为约 `0.737 m F`。最终代码在 plan 479（约 14 min 50.1 s）记录：

下表把 EXP-027 修改前 plan 474 与本实验中 collision_s 完全相同的最终 plan 479 对齐；plan 编号差异来自当前最新基线在此前实验后的 rolling 调用时序变化。

| vehicle | version | collision_s | original TTC | opposing boundary | final/geometric boundary | corrected TTC | action |
|---|---|---:|---:|---:|---:|---:|---|
| V0 | before | 2.360 | 8.800 s | 0.985 | 0.985 | 1.089 s | NOMINAL |
| V0 | final | 2.360 | 8.800 s | 0.985 | 0.810 | 0.214 s | NOMINAL |
| V1 | before | 1.412 | 8.800 s | 0.737 | 0.737 | 5.373 s | YIELD |
| V1 | final | 1.412 | 8.800 s | 0.737 | 0.162 | 1.287 s | STOP |

V1 第一阶段仍在 `query_s=0.712` 因 `match_distance=0.237 > 0.231 m` 结束；第二阶段继续得到 23 个连续 overlap self sample，在首个完整 clear window 的 `query_s=0.137` 结束，最后 overlap boundary 为 `0.162 m R`，与 EXP-028 的约 `0.15 m` 静态占用边界一致（允许 0.025 m 采样差）。`cusp_near_lost=true`，但 cusp 不作为触发必要条件。

plan 480--483 中 V0 的第一阶段已经回溯到实时位置之后的历史 s，最终 boundary 被硬限制到各周期实时 `path_s`，对应 TTC 为 0；V1 boundary 保持约 `0.149--0.164 m`，随后 pair 在 plan 484 转为非 bridge 的普通 timed conflict，V1 从 STOP 转为 CREEP，之后通过冲突区。

### 结果

- 判定：PASS。
- hard guard：0。
- deadlock 检出拍：0；重规划脱困：0；reciprocal STOP cycle：0；wedge episode：0。
- 最大累计 wait：V0 22.5 s，V1 17.3 s；结尾两车 wait 均为 0。
- 任务：V0 完成 12 个，V1 完成 13 个。
- 17 min 结尾：V0、V1 均为 ACTIVE/NOMINAL，分别继续执行后续 `A1 -> B51` 与 `A1 -> B35` 航段。

## RViz 验证状态

- 源码/编译已确认车辆标签 namespace 为 `multi_patrol_label`，格式为两行逐车 action、速度、TTC/reason，并沿车头方向偏移 `0.85 * vehicle_length`。
- 源码/编译已确认冲突说明 namespace 为 `conflict_explanation`，文字精简为 `Vx-Vy type=OPPOSING/CROSSING`，橙色 AABB 本体保留。
- 本次 batch 无交互式 RViz 目视验收，因此渲染观感仍需在 RViz 会话中确认；生产控制结果不依赖 marker。

## 产物

- `forklift_planner/logs/EXP030/seed2025_42_50_console.log`
- `forklift_planner/logs/EXP030/seed2025_42_50_coord.log`
- `forklift_planner/logs/EXP030/debug_seed2025_42_50/`
