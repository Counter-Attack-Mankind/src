# EXP-028：B42/B50 cusp 前静态 OBB 占用审查

## 范围

- 日期：2026-08-27
- 当前 HEAD：`81fa70d09053ed2bc253a932ab11cb415e508a1c`
- 场景：seed 2025、双车、`start_slots=[42,50]`，重点 plan 474--479。
- 路径：V0 `A1 -> B53`（path_gen 22，长度 6.9639 m）；V1 `B30 -> A1`（path_gen 25，长度 3.5238 m）。长度与正式日志一致。
- 本轮未修改任何生产源码、头文件、配置或测试。

## 方法

代码事实来自当前 `bridge_ttc_correction.cpp`、`footprint.cpp`、`spatiotemporal_interaction.cpp`、`rule_engine.cpp` 及相关头文件。动态事实来自 EXP-024 和 EXP-027 日志。

静态 OBB 真值使用仓库已有 `conflict_zone_geometry_diag` 的路径生成、`PathTrack::poseAtS()`、`makeBody()` 和 `overlaps()`；只在编译输入流中把该工具原场景的 slot 21/31 替换为 53/30，生成临时离线可执行文件，没有编辑仓库源码。扫描步长 0.0125 m，每车 margin 为 `0.5 * conflict_margin = 0.02 m`。完整二维扫描只用于离线取证；实施建议不采用二维全路径扫描。

结果再限制到 plan 474 的 V0 当前位置之后（`sV0 >= 0.771`）以及本次 bridge 连续相关的 V0 局部未来分支。所有表中匹配均落在 `sV0=3.3125--3.5500 m`，没有跳到远端 traversal。

紧凑数值证据见 `EXP-028-B42-B50-CUSP-STATIC-OBB-AUDIT.csv`。其中 center distance 分别记录路径后轴参考点距离和 `makeBody()` 后的车身 OBB 中心距离。

## 阶段一结论

### 当前回溯事实

`evaluateVehicle()` 不因 WpType 改变直接 break。它先以 collision pair 初始化 other 的局部 `NearestCursor`，每个 self query 计算：

```text
distance <= vehicle_width + conflict_margin (=0.231 m)
AND
cos(self motion heading - other motion heading) < -0.5
```

任一条件失败即 break。检查顺序是 distance 在前、direction 在后，因此同时失败时也记录 `RELATION_DISTANCE_LOST`。

plan 474--479 的 V1 数据：

| plan | collision_s | boundary_s/type | cusp_s | traversal changes | end reason | end dot | end distance |
|---:|---:|---|---|---:|---|---:|---:|
| 474 | 1.412 | 0.737 F | 0.700--0.706 | 0 | distance lost | -0.620 | 0.237 |
| 475 | 1.362 | 0.737 F | 同上 | 0 | distance lost | -0.620 | 0.237 |
| 476 | 1.261 | 0.736 F | 同上 | 0 | distance lost | -0.620 | 0.238 |
| 477 | 1.160 | 0.735 F | 同上 | 0 | distance lost | -0.620 | 0.238 |
| 478 | 1.060 | 0.735 F | 同上 | 0 | distance lost | -0.620 | 0.239 |
| 479 | 0.876 | 0.726 F | 同上 | 1 | distance lost | +0.620 | 0.238 |

plan 474--478 在 `s≈0.712 F` 已先因 distance 超过 0.231 m 终止，所以下一个 `s≈0.687 R` 的 direction 翻转没有进入生产结果，`self_traversal_changes` 仍为 0。plan 479 的第一处 LOST 已落在 R 段，但 distance 判断优先，仍记录 distance lost。

因此“不会因 WpType 本身 break，跨 cusp 后 motionHeading 会造成 direction lost”成立；“生产 end reason 通常是 direction lost”不成立。

### cusp 前静态 OBB

V1 的 WpType 转换位于 `0.700 R -> 0.70625 F`。静态 SAT 显示：

- `sV1=0.7375 F`：与 `sV0=3.3875 F` overlap，body-center distance 0.2037 m，dot=-0.6296。
- `sV1=0.7125 F`：relation 已因后轴点距离 0.2367 m 超限，但完整 OBB 仍 overlap。
- `sV1=0.7000 R`：dot 翻为 +0.6296，完整 OBB 仍 overlap。
- `sV1=0.6875--0.1500 R`：与同一 V0 局部分支持续存在 OBB overlap。
- `sV1=0.1375 R`：该局部分支首次 OBB clear。

离线二维连通真值中，本次局部 component 为：

```text
V0 s=[0.8000, 3.5875]
V1 s=[0.1500, 3.2125]
```

所以 cusp 前约 0.55 m 的 REVERSE prefix 确实仍占据 V0 本次局部未来路径会占据的空间。判断为：**部分成立**。底层“opposing 结束不等于静态几何占用结束”成立；关于实际 end reason 和基于 direction-lost 的触发条件不成立。

## 阶段二架构判断

两阶段方案仍属于 stateless per-vehicle TTC boundary correction，不需要引入 ConflictZone、SharedSegment、reservation、priority 或状态机，符合 bridge correction 的职责边界。

它能修正本场景缺失的几何边界：plan 474 可从约 0.737 m 延伸到约 0.150 m；plan 475 以后 V1 已处于静态 overlap 区，若下限严格限制为当前 `path_s`，`timeAtS()` 会正确得到 0 s。它是否最终解除 B42/B50 长期死锁仍需实施后的正式回归，当前证据不能提前宣称。

实际触发不能只检查 `RELATION_DIRECTION_LOST`。建议要求：

1. `bridge_related=true`；
2. 第一阶段因 distance 或 direction lost 结束；
3. 在第一处 LOST 附近、最多 `2 * bridge_backtrack_step` 的上游区间内确认 self WpType transition；
4. cusp 上游首批静态样本与当前 bridge 的 other 局部分支连续 overlap。

这样覆盖本场景，又不会让普通 crossing 或一般 direction lost 进入第二阶段。

## 阶段三实施设计

### 搜索边界

- self 下限必须为 `self.path_s`。当前 `timeAtS()` 对 `target_s <= prediction.front().s` 直接返回 0；若先扫描到历史路径再调用它，会把“历史 boundary”错误解释成“当前已在 boundary”，因此必须在扫描阶段禁止 `query_s < self.path_s`。
- other 只允许使用 `other_prediction.front().s` 到 `other_prediction.back().s`，即当前 15 s NOMINAL prediction 的实际可达 path-s；这天然排除历史路径和 horizon 外远端 traversal。
- 复用从 `other_collision_s` 初始化的 local cursor。每个 self sample 只扫描 cursor 匹配点附近的固定小窗口，不扫描 other 整条路径。

建议 local window 使用现有几何派生，不新增参数：

```text
r = hypot(vehicle_length/2 + margin,
          vehicle_width/2 + margin)
local_window_s = r_self + r_other + 2 * bridge_backtrack_step
```

当前数值 `r=0.17056 m`、body-center broad-phase bound `rA+rB=0.34112 m`、local window 约 0.391 m。0.025 m 采样时每个 self query 最多约 33 个 other 候选，仍为常数窗口。

### broad phase 与 margin

两车都继续使用：

```cpp
makeBody(pose, map_param, 0.5 * config.conflict_margin)
```

不增加第二套 margin。cheap reject 应比较 OBB body center，而不是后轴参考点：

```text
body_center_distance > hypot(half_l_a, half_w_a)
                     + hypot(half_l_b, half_w_b)
=> 一定不 overlap
```

通过 broad phase 后仍必须调用现有 `overlaps()` SAT。

### 连续性与 boundary

- `opposing_boundary_s` 保持现有第一阶段结果。
- 第二阶段从其上游一个 step 开始；overlap 时继续回溯并更新 `geometric_boundary_s`，首次 clear 时停止。
- 至少要求两个连续 self samples overlap，或累计延伸长度不小于 `2 * step`，才设置 `geometric_extension_applied=true`；这避免单个离散 SAT 命中造成延伸，不需要 component/BFS。
- clear 终止后 final boundary 是最后一个 overlap sample，不是 clear sample。
- 若直到 `self.path_s` 仍 overlap，final boundary 为 `self.path_s`，终止原因 `self_current_position`。

### 近 C++ 伪代码

插入点：现有 opposing `while` 结束之后，设置 `boundary_type` 和调用 `timeAtS()` 之前。

```cpp
const double opposing_boundary_s = result.near_boundary_s;
double final_boundary_s = opposing_boundary_s;

const bool cusp_near_lost = findSelfTypeTransition(
    self.track,
    max(self.path_s, result.end_query_s - 2.0 * step),
    opposing_boundary_s);

if (result.bridge_related &&
    isRelationLost(result.backtrack_end_reason) &&
    cusp_near_lost) {
    result.geometric_extension_attempted = true;

    const double self_floor = clamp(self.path_s, 0.0, self.track.length());
    const double other_floor = other_prediction.front().s;
    const double other_ceiling = other_prediction.back().s;
    LocalCursor cursor = cursor_from_first_phase;
    double query_s = opposing_boundary_s;
    int consecutive_overlap = 0;
    double last_overlap_s = opposing_boundary_s;

    while (query_s > self_floor + eps) {
        query_s = max(self_floor, query_s - step);
        const OBB self_body = makeBody(
            self.track.poseAtS(query_s), map_param,
            0.5 * config.conflict_margin);

        const NearestMatch nearest = nearestOnPathLocalClamped(
            self.track.poseAtS(query_s), cursor,
            other_floor, other_ceiling);

        bool any_overlap = false;
        for (double other_s : localSamples(
                 nearest.s, local_window_s, step,
                 other_floor, other_ceiling)) {
            const OBB other_body = makeBody(
                other.track.poseAtS(other_s), map_param,
                0.5 * config.conflict_margin);
            if (bodyCenterDistance(self_body, other_body) >
                circumscribedRadius(self_body) +
                circumscribedRadius(other_body)) {
                continue;
            }
            if (overlaps(self_body, other_body)) {
                any_overlap = true;
                result.geometric_matched_other_s = other_s;
                break;
            }
        }

        ++result.geometric_samples;
        if (!any_overlap) {
            result.geometric_end_reason = STATIC_OVERLAP_CLEARED;
            result.geometric_end_query_s = query_s;
            break;
        }

        ++result.geometric_overlap_samples;
        ++consecutive_overlap;
        last_overlap_s = query_s;
        if (query_s <= self_floor + eps) {
            result.geometric_end_reason = SELF_CURRENT_POSITION;
            break;
        }
    }

    if (consecutive_overlap >= 2 &&
        opposing_boundary_s - last_overlap_s >= 2.0 * step - eps) {
        result.geometric_extension_applied = true;
        result.geometric_boundary_s = last_overlap_s;
        final_boundary_s = last_overlap_s;
    }
}

result.opposing_boundary_s = opposing_boundary_s;
result.near_boundary_s = final_boundary_s;
result.boundary_type = self.track.typeAtS(final_boundary_s);
const double boundary_ttc = timeAtS(self_prediction, final_boundary_s);
result.corrected_ttc = min(original_ttc, boundary_ttc);
```

`evaluateVehicle()` 需要增加 `other_prediction` 参数；pair wrapper 仍分别调用 `evaluateVehicle(A,B,...)` 和 `evaluateVehicle(B,A,...)`，因此两车继续保留独立 boundary 和 corrected TTC。RuleEngine 当前只用 yielding 车辆的 corrected TTC 选择 baseline/effective 档位，并用 residual synchronized prediction 判断 priority safety；本方案不改变该逻辑。

### 最小诊断字段

建议在 `VehicleBridgeTtcCorrection` 增加：

```text
opposing_boundary_s
geometric_extension_attempted
geometric_extension_applied
geometric_boundary_s
geometric_samples
geometric_overlap_samples
geometric_sat_evaluations
geometric_broadphase_rejects
geometric_end_reason
geometric_end_query_s
geometric_matched_other_s
geometric_body_center_distance
geometric_self_type
geometric_other_type
```

end reason 最小集合：`not_triggered`、`static_overlap_cleared`、`self_current_position`、`other_prediction_range_exhausted`、`invalid_geometry`。`distance_pruned` 应作为计数而不是终止原因，因为 broad-phase reject 某一个 other candidate 不等于整个 local window clear。

### 预计修改文件

- 必需：`bridge_ttc_correction.cpp`、`bridge_ttc_correction.h`。
- 仅日志接入：`rule_engine.cpp`，把上述字段追加到现有 `[BRIDGE-TTC]`；不改变规则或控制输出。
- focused test：现有 `bridge_ttc_correction_test.cpp` 增加 cusp reverse-prefix、历史下限、远端 traversal 和孤立 overlap 用例。
- 不需要修改 `dynamic_speed_coordination.cpp`、`spatiotemporal_interaction.cpp`、配置或 YAML。

## 风险结论

1. 历史路径：必须以 `self.path_s` 硬截断；不能依赖 `timeAtS()` 事后兜底。
2. 远端错配：other prediction s 范围、collision-seeded cursor、固定 local window、first-clear 连续性四层共同约束。
3. 孤立 overlap：至少两个连续 self samples后才 applied。
4. 普通 crossing：`bridge_related + cusp-near-first-lost` 才触发。
5. 普通 reverse：必须从当前 opposing bridge boundary 连续 overlap 才能纳入。
6. 复杂度：第一阶段仍为 O(N)；第二阶段 O(N*K)，K 为固定局部窗口。本场景第一阶段 V1 为 8--29 samples；第二阶段约 10--24 self samples，K 上限约 33，最坏约 330--792 次候选检查，多数会被 0.341 m broad phase 拒绝，不退化为完整路径 O(N*M)。
