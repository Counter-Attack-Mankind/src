# EXP-044 UNLOAD_DWELL next-A1 departure waiter

- 日期：2026-09-08
- 分支：`fix`
- 基线 commit：`6bd8d46`
- 目的：让已有下一条 B→A1 缓存路径的 `UNLOAD_DWELL` 车辆提前建立并继承现有 A1 departure cluster。

## 修改范围

- `A1Coordinator::enforceFutureA1Admission()` 纳入 `UNLOAD_DWELL` waiter，通过现有只读 pickup-leg cache lookup 构造 `path_s=0`、`path_gen=current+1` 的 preview。
- preview 继续复用 `compute_full_conflict_zones()`、`selectFutureA1ProtectedZones()` 和 `futureA1StopS()`；没有新增闭包或近端库位规则。
- `DepartureClusterCommitment` 冻结 waiter 的缓存路径和 next-service generation，供 DWELL→TO_A1 身份继承及 RViz 截止线绘制。
- DWELL 期间的截止线、closure 和停止距离计算统一使用 B→A1 `s=0`、速度 0；切换 TO_A1 后恢复当前 `path_s` 和严格 generation 匹配。
- RViz 继续使用现有 `a1_waiter_stop` 与 `a1_frozen_zone` marker，仅为 DWELL waiter 改用冻结的 B→A1 preview path。

未修改 A1 owner ETA/竞争、普通 priority/TTC、bridge、Deadlock、阈值、路径生成或 departure release geometry。

## 验证

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
```

- Release 构建通过，`multi_vehicle_patrol_node` 及全部构建目标成功编译链接。
- 按要求未运行 CTest、batch 或仿真；运行效果仍为“未知，需要确认”。

