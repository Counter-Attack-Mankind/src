# EXP-024：bridge 局部 gap 与 OBB collision cluster 审查

## 1. 范围与方法

- 日期：2026-08-25
- 当前 HEAD：`fbdee63970a7397f5a828d1f8973acd9e72298ce`
- 输入：EXP-023 后源码和日志。
- 场景：`random_seed=2025`、`start_slots=[42,50]`、`vehicle_count=2`、9000 ticks / 900.0 s。
- 目标：只诊断 plan 474--479 的 V1 bridge relation 和完整 15 s synchronized OBB overlap clusters，不修改控制返回值。

临时诊断分别在独立 cursor/独立 OBB scan 中继续越过生产代码的第一处 LOST/clear。诊断条件限定为 V1 `path_gen=25` 且 collision sB 在 `[0.8,1.5]`。诊断代码在取证后已移除；生产源码和控制结果没有保留 EXP-024 分支。

## 2. 当前代码事实

`bridge_ttc_correction.cpp::evaluateVehicle()`：

- 不再以 self `WpType` 变化终止；
- other cursor 可沿相邻 path order 跨 cusp；
- relation 为 `distance <= 0.231` 且 `direction_dot < -0.5`；
- 生产 loop 在第一处非 VALID state 立即 `break`。

`spatiotemporal_interaction.cpp::detectPairInteractionFromPredictions()`：

```cpp
if (!hit) {
    if (result.event.valid) break;
    continue;
}
```

因此生产 `TimedConflictEvent` 只提取第一段连续 OBB overlap。如果实际存在 `hit -> clear -> hit`，后段会被忽略；是否在本场景实际发生，必须由完整诊断扫描决定。

## 3. bridge 连续扫描结果

步长与生产 backtrack 相同，均为 0.025 m。六个 plan 的关键边界完全一致：

| plan | collision sB | last VALID | first LOST | first LOST state | next sample | 后续状态 |
|---:|---:|---:|---:|---|---:|---|
| 474 | 1.412 | 0.737 F, d=0.218, dot=-0.642 | 0.712 F, d=0.237, dot=-0.620 | DISTANCE | 0.687 R | DIRECTION |
| 475 | 1.362 | 0.737 F, d=0.217, dot=-0.642 | 0.712 F, d=0.237, dot=-0.620 | DISTANCE | 0.687 R | DIRECTION |
| 476 | 1.261 | 0.736 F, d=0.218, dot=-0.642 | 0.711 F, d=0.238, dot=-0.620 | DISTANCE | 0.686 R | DIRECTION |
| 477 | 1.160 | 0.735 F, d=0.219, dot=-0.642 | 0.710 F, d=0.238, dot=-0.620 | DISTANCE | 0.685 R | DIRECTION |
| 478 | 1.060 | 0.735 F, d=0.219, dot=-0.642 | 0.710 F, d=0.239, dot=-0.620 | DISTANCE | 0.685 R | DIRECTION |
| 479 | 0.876 | 0.726 F, d=0.226, dot=-0.635 | 0.701 R, d=0.238, dot=+0.620 | BOTH | 0.676 R | DIRECTION |

每组从第一处 LOST 继续扫描 12 samples、0.275 m，均没有重新出现 VALID：

- plan 474/475：`s=0.712 -> 0.437`；
- plan 476：`0.711 -> 0.436`；
- plan 477/478：`0.710 -> 0.435`；
- plan 479：`0.701 -> 0.426`。

距离条件本身只出现一个 0.025 m sample 的窄 gap：下一 sample 跨入 R traversal 后，distance 已由约 0.237--0.239 降回 0.225--0.227 m。但 direction dot 同时从约 `-0.620` 翻为约 `+0.625`，之后在整个诊断区间保持 `+0.625 -> +0.131~+0.159`，始终不满足 `< -0.5`。所以完整 relation 不是 `VALID -> LOST -> VALID`，而是 `VALID -> DISTANCE/BOTH_LOST -> persistent DIRECTION_LOST`。

## 4. cusp 关系

plan 474--478 的 first LOST 仍为 F，下一 sample 已为 R；plan 479 的 last VALID 为 F，first LOST 已为 R。按 0.025 m 离散步长，cusp 位于相邻两点之间。因此 first LOST 与 cusp 紧邻，plan 479 直接落在 cusp 后第一点。

这只能证明 boundary 与 cusp 的真实几何/运动语义相关，不能证明存在可跨越的短 bridge gap。跨 cusp 后空间距离很快恢复，但实际运动方向已经不再 opposing。

## 5. 完整 15 s OBB cluster

| plan | cluster | first/last t | first/last sA | first/last sB | samples |
|---:|---:|---|---|---|---:|
| 474 | 0 | 8.800 / 10.850 | 2.360 / 2.653 | 1.412 / 1.718 | 42 |
| 475 | 0 | 7.200 / 9.200 | 2.419 / 2.704 | 1.362 / 1.674 | 41 |
| 476 | 0 | 6.100 / 7.950 | 2.547 / 2.810 | 1.261 / 1.580 | 38 |
| 477 | 0 | 4.900 / 6.500 | 2.662 / 2.905 | 1.160 / 1.469 | 33 |
| 478 | 0 | 3.700 / 5.050 | 2.775 / 3.013 | 1.060 / 1.330 | 28 |
| 479 | 0 | 2.900 / 4.300 | 2.983 / 3.234 | 0.876 / 1.156 | 29 |

六组都只有 cluster 0，完整 horizon 中没有 `collision -> clear -> collision`。这些 sB 区间全部高于约 0.70 m cusp，位于 F traversal；不存在位于 R traversal 的第二 collision cluster。

## 6. 结论

判断：**不成立**。

- bridge：第一处 LOST 后至少 0.275 m / 12 samples 持续失效，没有恢复 VALID。
- distance：确有一个 sample 的局部超限，但不能单独恢复完整 relation；cusp 后 direction 已持续变为非 opposing。
- OBB：六个 plan 均只有一个连续 collision cluster。
- `TimedConflictEvent`：代码结构一般性地会忽略第二 cluster，但本场景没有第二 cluster，因此没有发生实际遗漏。
- `boundary_s_b≈0.735~0.737` 的几何原因：F traversal 上，最近点距离随回溯增至 proximity limit 附近；再向上游跨 cusp 后，虽然距离重新缩短，V1 的实际运动方向翻转，使双方 motion direction dot 变为正值并持续不满足 opposing。该 boundary 是当前 distance+direction 联合定义下的真实 relation 边界，不是短 gap 吸附。

本记录只给证据和结论，不提出或实施修复方案。

原始诊断证据：`EXP-024-B42-B50-seed2025-diagnostic.log`。定向生产协调日志：`EXP-024-B42-B50-seed2025-15min-coord.log`。
