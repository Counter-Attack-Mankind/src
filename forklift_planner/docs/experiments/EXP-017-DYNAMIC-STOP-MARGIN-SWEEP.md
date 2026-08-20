# EXP-017：dynamic STOP time margin 四组对比

## 实验身份与范围

- 日期：2026-08-20
- Git HEAD：`d847a701edc3508055747743a86daec57e3445ac`
- 工作区：实验前已有用户 staged/unstaged 修改及 EXP-015/EXP-016 产物；本轮未 reset、checkout、stash、stage 或 commit。
- 目的：只检验增大 `dynamic_stop_time_margin` 是否能使 seed=2024 中的 yielding vehicle 更早 STOP，并消除约 140.5 s 的 V0-V1 hard-guard collision。
- 固定条件：`vehicle_count=2`、`random_seed=2024`、`2400 tick`、`dt=0.1 s`、真实仿真时间 240 s。
- 唯一实验变量：`dynamic_stop_time_margin=0.10/0.50/0.80/1.00 s`。
- 本轮没有修改 C++ 或算法结构，也没有改动 priority、winner、forward clearance、A1、reservation、STOP recovery、hard guard、task allocator 或空间 stop-line。

## 公式与参数

EXP-016 当前实现与本轮给定的数值表使用加法：

```text
TTC_stop = rolling_refresh_period
         + speedForAction(CREEP) / max_decel
         + dynamic_stop_time_margin
```

请求开头公式中的两个减号与其后四个明确理论阈值、EXP-016 实现相冲突；本实验以现有实现和明确阈值为准，未改公式。固定项为：a

- `rolling_refresh_period=2.000 s`
- `speedForAction(CREEP)=0.050 m/s`
- `max_decel=0.300 m/s²`
- 日志实测 `braking_time=0.167 s`

每组 `DYN-SPEED` 日志均实际输出了 `decision_period`、`action_speed`、`braking_time`、`time_margin` 和 `stop_threshold`，得到 2.267/2.667/2.967/3.167 s，不是仅使用手算值。

## 执行方法

0.10 s 组复用 EXP-016 已完成且条件完全相同的 4 min smoke；0.50/0.80/1.00 s 分别只运行一次：

```bash
roslaunch forklift_planner multi_vehicle_phase2_batch.launch \
  minutes:=4 vehicle_count:=2 random_seed:=2024
```

每次运行前只替换 YAML 中的 `dynamic_stop_time_margin`。四组都完成 `2400/2400` tick，汇总日志确认 `real_sim_t=240.0`。实验后已将 YAML 恢复为当前 baseline `0.10 s`。未运行其他 seed，未运行 120 min，未试验超过 1.00 s 的 margin。

## 主结果

| margin | TTC_stop threshold | first STOP first_t | first STOP path_s | final stop path_s | hard guard | first hard guard | long STOP/STOP | tasks at 240 s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.10 s | 2.267 s | 1.300 s | 0.682 | 0.684 | 1 | 140.5 s | 是 | V0=2, V1=1 |
| 0.50 s | 2.667 s | 1.300 s | 0.682 | 0.684 | 1 | 140.5 s | 是 | V0=2, V1=1 |
| 0.80 s | 2.967 s | 1.300 s | 0.682 | 0.684 | 1 | 140.5 s | 是 | V0=2, V1=1 |
| 1.00 s | 3.167 s | 1.300 s | 0.682 | 0.684 | 1 | 140.5 s | 是 | V0=2, V1=1 |

“提前 path_s”以 EXP-016 的最终停车 `s=0.684` 为基准：四组均为 `0.000 m`，即 0.50/0.80/1.00 s 都没有使停车点上游移。

## 请求计数与不变量核查

| margin | FAR/MID/NEAR | YIELD/CREEP/STOP 请求 | braking/TTC STOP | ordinary reservation create | rolling_order_swap | A1 owner change | forward-clearance 最后一拍 |
|---|---:|---:|---:|---:|---:|---:|---|
| 0.10 s | 2/3/52 | 3/1/142 | 51 | 0 | 0 | 0 | 介入，V1 `emergency_next_step_V0` |
| 0.50 s | 2/3/52 | 3/1/142 | 51 | 0 | 0 | 0 | 介入，V1 `emergency_next_step_V0` |
| 0.80 s | 2/3/52 | 3/1/142 | 51 | 0 | 0 | 0 | 介入，V1 `emergency_next_step_V0` |
| 1.00 s | 2/3/52 | 3/1/142 | 51 | 0 | 0 | 0 | 介入，V1 `emergency_next_step_V0` |

该 YIELD/CREEP/STOP 为 `BATCH_DYN_SPEED_EXECUTED` 中的 rolling target request 计数，不是动作持续时间。四组另均为：A1 service create/release=4/4，wedge episode=1，reciprocal STOP cycle=0，deadlock 检出拍数=149，V0 最大 wait=109.1 s。

## seed=2024 失败链条

四组的关键 rolling period 完全相同：

1. plan 74：`first_t=4.400 s`，V0/V1 选择 `CREEP/NOMINAL`。即使 1.00 s 组的阈值已提高到 3.167 s，4.400 s 仍不满足 STOP gate。
2. plan 75：下一次约 2 s rolling refresh 时 `first_t` 已跳到 1.300 s，四组均首次 TTC STOP；yielding V0 `path_s=0.682`、`current_speed=0.050 m/s`，priority V1 仍为 NOMINAL。
3. V0 最终均停在 `path_s=0.684`。在 plan 74 的 4.400 s 和 plan 75 的 1.300 s 之间没有另一次 ordinary rolling decision，因此 2.267–3.167 s 内的阈值变化没有改变 STOP 的决策拍。
4. tick 1404，forward clearance 将 V1 设为 STOP，原因 `emergency_next_step_V0`，此时日志中 V1 `path_s=4.107`、`current_speed=0.170 m/s`。
5. tick 1405（约 140.5 s），四组均发生同一 V0-V1 hard-guard collision；随后长期 STOP/STOP，并检出 deadlock。

## 产物

- 0.10 s baseline：`forklift_planner/logs/EXP016/smoke2024_console.log`、`smoke2024_coord.log`
- 0.50 s：`forklift_planner/logs/EXP017/margin_050_console.log`、`margin_050_coord.log`、`debug_margin_050/`
- 0.80 s：`forklift_planner/logs/EXP017/margin_080_console.log`、`margin_080_coord.log`、`debug_margin_080/`
- 1.00 s：`forklift_planner/logs/EXP017/margin_100_console.log`、`margin_100_coord.log`、`debug_margin_100/`

## 结论

**FAIL**：0.50/0.80/1.00 s 均与 0.10 s baseline 在同一 plan 75、同一 `first_t=1.300 s`、同一位置首次 STOP，最终停车点未提前，并均在 140.5 s 发生同类 V0-V1 hard-guard collision。所以，在本轮限定的小型安全裕度范围内，单纯提高 `dynamic_stop_time_margin` 不足以解决该几何场景。

本结论只针对 seed=2024、2 车、240 s 及指定的四个 margin，不表示整个多车系统通过或失败。按实验边界到此停止，未继续增大 margin，也未设计或实现新机制。
