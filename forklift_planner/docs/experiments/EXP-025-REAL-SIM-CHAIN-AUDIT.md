# EXP-025 仿真/实车运行链路架构审查

## 0. 审查元数据与证据等级

| 项目 | 结果 |
| --- | --- |
| 审查日期 | 2026-08-26（Asia/Shanghai） |
| Git commit | `81fa70d09053ed2bc253a932ab11cb415e508a1c` |
| 审查开始时工作区 | dirty；已有未跟踪的 `EXP-024-BRIDGE-GAP-AND-OBB-CLUSTER-AUDIT.md`、`EXP-024-B42-B50-seed2025-15min-coord.log`、`EXP-024-B42-B50-seed2025-diagnostic.log`，本审查未覆盖或修改它们 |
| 生产代码修改 | 无 |
| SOURCE CONFIRMED | YES；审查了 launch、C++/Python、YAML、消息、CMake 和 package.xml |
| CONFIG CONFIRMED | YES（静态）；顶层及递归 include 的 XML 均可被 XML 解析器读取 |
| BUILD CONFIRMED | NO；当前 Windows PowerShell 环境没有 `catkin_make`，命令以 `CommandNotFoundException` 失败 |
| 既有构建产物 | `devel/lib` 中存在 2026-08-24/25 的 planner、PP、LQR 产物；这只能证明曾构建，不能代替本次基于当前源码的重构建 |
| ROS GRAPH CONFIRMED | NO；当前环境没有 `roslaunch`/ROS 运行命令，未启动 ROS graph |
| HARDWARE TESTED | NO |
| REAL VEHICLE TESTED | NO |

本报告的行号均指审查 commit/工作区中的当前文件。代码注释只作为线索；结论来自实际调用、订阅、发布和参数读取。

## 1. Executive Summary

### 1.1 直接结论

| 问题 | 结论 |
| --- | --- |
| 当前 sim/real 是否逻辑统一 | **否，仅部分共用代码。** 两边共用 `TaskAllocator`、`MissionPhase`、`RuleEngine`、priority、同步 OBB/TTC、bridge TTC 和动作枚举；但实际 tick 顺序、ordinary decision 的刷新语义、rolling rollout 和控制器执行链不同。 |
| 0.1 s 是否成立 | **源码/配置层两边成立。** `update_rate=10 Hz` 驱动 planner tick；PP/LQR 也各自 0.1 s 控制。动捕上游为 100 Hz/60 Hz，planner 在 10 Hz tick 使用回调缓存。未做 ROS 实测。 |
| 2.0 s 是否两边均为高层 commitment | **否。** 仿真普通协调在构建计划时决定一次并执行最多 20 个冻结帧；实车每 0.1 s 直接重新 `RuleEngine::decide()`，2 s 只控制 `/traj_i` 的重新 rollout/发布。 |
| 15.0 s 是否两边语义一致 | **否。** `rolling_horizon=15` 和 `prediction_horizon=15` 是两个参数。仿真计划把 pair horizon 从 15 s 随未来帧递减；实车的外层 15 s rollout 内每一拍又用新的 15 s `prediction_horizon`，可在 rollout 尾部推理到“当前约 30 s 后”。 |
| TTC 草履虫模式是否两边计算 | **是，源码计算链共用。** real/sim 都能进入同一个 `RuleEngine::resolvePairwiseConflicts()`；但执行时序不一致。 |
| bridge TTC 是否两边计算 | **是，源码调用链共用。** ordinary 的有效同步 OBB 冲突会调用 `evaluateBridgeTtcCorrection()`。但默认 PP 不订 `/coord_speed_i`，所以每拍 TTC 动作并非端到端即时进入默认执行器。 |
| `realbridge_exp.launch` 是否完整 | **NO。** 它不是当前仓库中可独立完成部署的入口：缺 `/rb_start`/`/estop` 发布节点，`cmd_timeout` 未接入，底盘无 watchdog，默认 PP 不订协调速度，`chassis`/`vrpn_client_ros` 被 `CATKIN_IGNORE`，Nokov 脚本未安装且运行依赖未声明完整。 |
| 是否建议直接上双车实车 | **不建议。** P0 部署门未关闭；应先完成架空轮、单车低速、急停/失联/进程崩溃测试并修复默认控制链的 fail-safe。 |

### 1.2 最重要的 sim/real 分叉

```text
Simulation rolling mode
  每 2 s（或计划失效）构建一次 15 s SimPlanFrame
  frame 0 决定 ordinary coordination
  frame 1..N 复用该 period 的 ordinary decision
  每 0.1 s 只恢复并执行一个计划样本

Real rolling mode
  每 0.1 s updateDwellAndTasks
  每 0.1 s RuleEngine::decide（重新决定 ordinary coordination）
  每 0.1 s 才 realAdvance（故 decide 使用上一次投影后的 path_s/speed）
  每 2 s rollWorldModel(15 s) 并发布 /traj_i
  rollout 内每 0.1 s 又重新 RuleEngine::decide，且每次重新打开 15 s pair horizon
```

证据：`multi_vehicle_patrol_node.cpp:2588-2683`、`:671-792`、`:1144-1173`、`:1218-1387`、`:1416-1430`。

## 2. 实车启动树

### 2.1 顶层实际展开

```text
roslaunch forklift_planner realbridge_exp.launch
│
├─ [A] load /forklift_map/*                  map_param.yaml
├─ [A] load /forklift_planner/*              planner_param.yaml
├─ [A] set a1_cycle_catalog_file / vehicle_count
├─ [A] load /chassis/*                       vehicle_chassis_gains.yaml
├─ [A] load /controller_0..7/*               vehicle_controller_gains.yaml
│
├─ [B] include vrpn_client_ros/sample.launch
│  └─ vrpn_client_node
│     ├─ VRPN server=$(arg server), port=3883
│     ├─ update_frequency=100 Hz
│     ├─ refresh_tracker_frequency=1 Hz
│     └─ publishes /vrpn_client_node/<tracker>/pose, frame=world; broadcast_tf=true
│
├─ [A] nokov_localization/nokov_localization_v2.py (name=/nokov)
│  ├─ once-after-2-s topic discovery under /vrpn_client_node
│  └─ publishes /object at a 60 Hz timer
│
├─ [A] chassis/chassis_node (name=/chassis)
│  ├─ subscribes /chassis
│  ├─ publishes /velocity
│  └─ fixed serial /dev/ttyUSB0 @ 57600
│
├─ [A] forklift_map/map_node
├─ [A, optional effect] rviz with forklift.rviz; required=false
│
├─ [A] forklift_planner/multi_vehicle_patrol_node
│  ├─ global real_mode=true
│  ├─ subscribes /object, /rb_start, /estop
│  └─ publishes /traj_i, /coord_speed_i, /coord_state_i and markers
│
└─ [B, conditional target 0..7] include one_controller.launch
   ├─ controller=pp (default)
   │  └─ pure_pursuit/pure_pursuit_node, name=/controller_i
   │     ├─ subscribes /traj_i and /object
   │     └─ publishes /chassis(target=i)
   └─ controller=lqr
      └─ lqr_controller/lqr_controller_node, name=/controller_i
         ├─ subscribes /traj_i, /coord_speed_i and /object
         └─ publishes /chassis(target=i)
```

来源：`realbridge_exp.launch:11-83`、`one_controller.launch:5-30`、`vrpn_client_ros/launch/sample.launch:3-23`。

### 2.2 A/B/C/D/E 分类

| 类别 | 当前事实 |
| --- | --- |
| A 顶层直接启动 | Nokov、chassis、map、RViz、planner；加载四类 YAML/参数 |
| B include 间接启动 | VRPN client；按 `veh_count`/`target_only` 启动 1..8 个 PP 或 LQR 控制器 |
| C 需要人工/单独节点 | `/rb_start` 与 `/estop` 的发布者。源码注释称 `estop_key.py`，但仓库中未找到该文件，顶层 launch 也未启动等效节点。logger/bag recorder 同样未启动。 |
| D 当前被隔离或无本次构建证据 | `experiment/chassis/CATKIN_IGNORE`、`experiment/vrpn_client_ros/CATKIN_IGNORE` 均存在；遵循仓库规则不得为全量编译删除。当前 `devel/lib` 也没有这两个包。Nokov 无 CATKIN_IGNORE，但 `catkin_install_python` 被注释，`devel/lib/nokov_localization` 未见该脚本。 |
| E 外部系统 | ROS master（由 roslaunch 处理）、VRPN/Nokov 服务端和刚体命名、串口设备/权限/固件、真实底盘、操作员启动/急停发布器；是否满足均未验证。 |

补充：`forklift_planner/package.xml` 只声明 planner 自身的核心依赖，没有声明 launch 使用的 `vrpn_client_ros`、`nokov_localization`、`pure_pursuit`、`lqr_controller`、`chassis`、`rviz`。因此安装/部署环境不能仅依据该包清单复现顶层 launch。

## 3. 实车 ROS 数据流

### 3.1 从动捕到轮子

```text
外部 VRPN server (10.1.1.198:3883 默认)
  ↓
/vrpn_client_node/<tracker>/pose (geometry_msgs/PoseStamped, frame=world)
  ↓ nokov_localization_v2.py 自动发现 Vehicle0..Vehicle7
/object (sandbox_msgs/AprilObject; id=0..7; x/y 原样转发; yaw=rad; frame=world)
  ├─→ MultiVehiclePatrolNode::objectCallback
  │     x/y ÷1000 → m，缓存真实 pose
  │     ↓ 每 0.1 s tick 的 realAdvance（在 decide 之后）
  │     path projection → path_s/current_speed
  │     ↓ RuleEngine（下一拍才使用这次投影状态）
  │     VehicleAction
  │     ├─→ /traj_i (sandbox_msgs/Trajectory, rolling 默认每 2 s/事件刷新)
  │     └─→ /coord_speed_i (std_msgs/Float64, 10 Hz)
  │
  ├─→ PP controller_i (默认)
  │     /traj_i + /object；不订 /coord_speed_i
  │     ↓ 10 Hz
  │     /chassis (sandbox_msgs/ChassisCommand, target=i)
  │
  └─→ LQR controller_i (可选)
        /traj_i + /coord_speed_i + /object
        ↓ 10 Hz
        /chassis (sandbox_msgs/ChassisCommand, target=i)

/chassis
  ↓ chassis_node
  throttle[m/s]×100 → int16；steering[rad]→负的 degree int16
  target → serial packet first byte
  ↓ /dev/ttyUSB0 @ 57600
  ↓ 底盘固件/车辆（未测试）

串口速度回报
  ↓ /velocity (std_msgs/Float64, raw/100；无 vehicle ID)
```

### 3.2 接口追踪表

| 发布者 → 订阅者 | topic / type | 单位与 frame | ID | 频率 | timeout / 丢失行为 |
| --- | --- | --- | --- | --- | --- |
| VRPN client → Nokov | `/vrpn_client_node/<name>/pose`; `geometry_msgs/PoseStamped` | 源码原样复制 VRPN position；frame=`world`；实际服务器长度单位 **UNKNOWN，需要现场确认** | tracker 名 `VehicleN` | client timer 100 Hz；tracker discovery 1 Hz | VRPN client 侧未见 pose watchdog；Nokov 只在启动约 2 s 后枚举一次话题，晚出现 tracker 可能不被订阅 |
| Nokov → planner/控制器 | `/object`; `sandbox_msgs/AprilObject` | x/y 原样发布；下游明确 `/1000`，故当前接口契约假定 mm；yaw 由 quaternion 解算为 rad；header=`world` | `type=VEHICLE`, `id=N` | Nokov timer 60 Hz | Nokov 不发布显式 stale 状态；planner 自己按 wall time 判断；PP/LQR 无 pose timeout |
| planner → controller N | `/traj_N`; `sandbox_msgs/Trajectory` | x/y=m、yaw=rad；header=`world`；point.velocity 为带符号的规划速度/方向 | topic 后缀与 `msg.target=N` 双重编码 | rolling 默认约 0.5 Hz，路径事件可提前刷新；latched | 控制器无 trajectory timeout；planner 不发缺 pose 车辆的轨迹 |
| planner → controller N | `/coord_speed_N`; `std_msgs/Float64` | 带符号 m/s | topic 后缀 N | planner 启动后 10 Hz，非 latched | planner stale/estop/guard 可置零；**PP 不订阅**；LQR 缓存最后值且无 timeout |
| PP/LQR N → chassis_node | `/chassis`; `sandbox_msgs/ChassisCommand` | throttle=m/s，steering=rad | `target=N`；所有控制器共用 topic | 控制 timer 10 Hz | PP/LQR 均无输入 freshness watchdog；chassis_node 无命令 watchdog |
| chassis hardware → consumers | `/velocity`; `std_msgs/Float64` | 串口 raw velocity `/100`，推断为 m/s，但协议/固件未验证 | **无 ID**，多车语义不明确 | 硬件回报频率 UNKNOWN | 未见 timeout；本链 planner/controller 也未订阅它 |

消息证据：`sandbox_msgs/msg/AprilObject.msg`、`Trajectory.msg`、`TrajectoryPoint.msg`、`ChassisCommand.msg`。节点证据：`multi_vehicle_patrol_node.cpp:2758-2790,2894-2924,3059-3126`、`pure_pursuit.cpp:274-280,320-366,634-675`、`lqr_controller_node.cpp:20-29,72-149`、`chassis_node.cpp:36-41,60-73,85-119`。

## 4. 坐标、单位与车辆 ID 审查

### 4.1 单位

- Planner/地图/path、`VehicleAgent.path_s`、`/traj_i`：m；yaw：rad；速度：m/s。
- `/object`：Nokov v2 对 VRPN x/y 不做缩放即发布；planner、PP、LQR 均 `/1000`。因此代码强制假定 `/object.x/y` 为 mm。VRPN client 自己只是复制 `tracker_pose.pos[]`，仓库没有证明外部服务器必定提供 mm；这是部署前必须用已知标尺确认的接口契约。
- `ChassisCommand` ROS 边界：throttle m/s、steering rad。`chassis_node` 将 throttle 乘 100 后截断为 `int16`，steering 转负的 degree 后截断；两者分别限幅到 ±26 和 ±28.65°。
- 规划 `max_steer_angle=0.40 rad`，底盘限幅为约 `0.50 rad`；LQR 构造 wheelbase 为 `0.1445 m`，planner/PP 为 `0.143 m`。这不是同一真值源。

### 4.2 frame 与原点

- VRPN launch 和 `/object` header 使用 `world`。
- planner/map markers 使用 YAML 的 `frame_id=map`。
- planner 发布 `/traj_i` 时写 `world`，但几何数值直接来自 map 坐标。
- PP/LQR/planner 回调均忽略输入 header frame，不做 tf 查询。
- 仓库范围没有找到 `static_transform_publisher`，也没有 `map↔world` 的 tf/tf2 lookup 或 x/y/yaw 人工平移旋转参数。VRPN 的 `broadcast_tf=true` 只广播 `world→tracker`，不能证明 `map==world`。

结论：**map 与 Nokov world 数值重合关系 NOT VERIFIED / DEPLOYMENT BLOCKER**。必须通过至少两个非共线已知点和已知车头方向，确认平移、旋转、尺度、轴向和 yaw 正方向；不能只在 RViz 目测一个点。

### 4.3 多车 ID

- VRPN tracker `VehicleN` → Nokov `AprilObject.id=N`。
- planner 数组直接以 `id` 索引；越界或非 VEHICLE 消息被丢弃。
- `/traj_N`、`/coord_speed_N` 与控制器私有参数 `target=N` 对应；Trajectory 还检查 `msg.target`。
- 所有控制器共用 `/chassis`，`ChassisCommand.target` 被写入串口包首字节，承担车辆寻址。
- `/velocity` 不带 target；在多车共享串口下无法由消息本身判断是哪台车的反馈。真实语义 **UNKNOWN，需要确认固件/协议**。

## 5. Simulation：一个真实 0.1 s tick

rolling 模式（当前 YAML `one_shot_traj=false`）实际顺序：

```text
tick_count++, sim_time += 0.1
  ↓
updateDwellAndTasks(0.1)
  ↓
simulationPlanNeedsRefresh?
  ├─ yes: publishHorizon
  │       → buildSimulationHorizonPlan
  │       → rollWorldModel(15 s, plan_frames!=nullptr)
  │          frame 0: RuleEngine::decide(remaining=15 s)
  │          frame 1..: decide(... reuse_ordinary_coordination=true,
  │                    remaining_horizon 逐拍减少)
  │       → 保存约 150 个 SimPlanFrame
  └─ no
  ↓
executeSimulationPlanSample
  → restore 对应 RuleEngine snapshot
  → 恢复本帧 action/requested_action/blocker/reason 等
  → sim_plan_cursor++
  ↓
advanceVehicles(0.1)
  → 加减速/曲率限速/路径积分
  → next-step OBB hard collision guard
  → 到达状态转换
  ↓
每 0.5 s 运行停用状态下的 deadlock recovery 检测入口
  ↓
日志、诊断、markers
```

刷新条件除 20 帧外，还包括计划未建立、路径/模式变化、future A1 owner 失效和 `force_horizon_refresh_`。因此“2 s”是最大常规 commitment，不是任何事件下都强制等待满 2 s。

关键点：未来 rollout 的每个 0.1 s frame 确实还调用 `RuleEngine::decide()`，但从 frame 1 起传入 `reuse_ordinary_coordination=true`，会恢复 frame 0 的 ordinary target，不再重新按 FAR→MID→NEAR 改普通 pair 的优先级/动作；A1、reservation 和 safety 层仍可在未来帧运行。执行阶段也不再次 decide，而是逐帧恢复计划。

## 6. Real：一个真实 0.1 s tick

启动前：timer 仍每 0.1 s 进入 tick，但 `rb_started_==false` 时只画 marker、尾迹和摆位日志，然后 return。启动必须收到外部 `/rb_start=true`。

rolling 模式启动后实际顺序：

```text
/object callbacks 异步缓存 real_x/y/yaw 和 last_seen
  ↓
tick_count++, sim_time += 0.1
  ↓
updateDwellAndTasks(0.1)
  ↓
RuleEngine::decide(agents, 0.1)
  → 使用上一次 realAdvance 得到的 path_s/current_speed
  → ordinary coordination 每拍重算
  ↓
realAdvance(0.1)
  → 用本拍缓存的 real_x/y 投影到路径
  → path_s 单调不减，current_speed=(Δs/0.1) 并 clamp
  → 到达状态转换
  ↓
每 0.5 s deadlock 检测入口
  ↓
每 2 s 或 force refresh:
  publishHorizon
  → rollWorldModel(15 s, plan_frames=nullptr)
  → 未来每 0.1 s 都重新 RuleEngine::decide(默认 prediction_horizon=15 s)
  → /traj_i
  ↓
publishRealOutputs(0.1)
  → realHardGuard
  → action→速度、曲率限速、方向、stale/estop/guard、加减速限制
  → /coord_speed_i + /coord_state_i
  ↓
markers / trail
```

证据：`multi_vehicle_patrol_node.cpp:2588-2651,2758-2804,2894-3126`。

直接回答：**实车没有与仿真相同的 2 s ordinary commitment window。** 实车每拍重新决定，且 decide 位于本拍真实 path projection 之前，协调状态滞后一个 planner tick。

## 7. Sim vs Real 差异矩阵

| 功能 | Simulation | Real | 一致 | 是否应统一 |
| --- | --- | --- | --- | --- |
| 状态源 | `advanceVehicles` 积分 | `/object` + `realAdvance` 投影 | 合理不同 | 保留 adapter 差异 |
| planner tick | 10 Hz | 10 Hz | 是（源码） | 是 |
| 当前状态进入 decision 的顺序 | 计划样本基于当前仿真状态构建 | 先 decide，后投影最新 mocap | 否 | 是；应先形成统一当前状态快照 |
| ordinary 决策刷新 | 通常 2 s/事件一次 | 每 0.1 s 一次 | 否 | 是 |
| rolling horizon | 15 s 计划 + 冻结前 2 s 样本 | 15 s 轨迹每 2 s发布 | 数值同、语义不同 | 是 |
| pair prediction horizon | rollout 中从 15 s 递减 | 顶层每拍 15 s；rollout 每个未来帧又开 15 s | 否 | 是 |
| TaskAllocator/MissionPhase | 同一对象/状态机 | 同一对象/状态机 | 是 | 保持 |
| RuleEngine | 同一类/实例路径 | 同一类/实例路径 | 是（代码） | 保持 |
| 同步 OBB / TTC | 共用 | 共用 | 是（计算） | 保持 |
| bridge TTC | ordinary baseline 冲突后共用 | 同一入口共用 | 是（计算） | 保持 |
| priority | `priorityWinner/unifiedPriority` | 同一函数 | 是（函数） | 保持，但刷新时序需统一 |
| action | 仿真执行计划帧的冻结 action | 每拍重算 action | 否 | 是 |
| actuator action 服从 | `advanceVehicles` 直接用 action | LQR 用 coord speed；默认 PP 不用 | 否 | 是，P0 |
| trajectory | 计划也用于 sim action frame | real 每 2 s另做 rollout 后发布 | 否 | 是，至少统一 plan artifact |
| hard guard | next-step planned OBB，可直接阻止积分 | 当前实测 OBB overlap；输出速度经 ramp；默认 PP忽略 coord speed | 否且 real 有断链 | 保留 real-only guard，但修复端到端 STOP |
| mocap/E-stop | 不适用 | real-only | 合理不同 | 必须保留并加强 |

## 8. 时间尺度与参数实效审查

### 8.1 参数追踪

| 参数 | C++ 默认 | YAML | launch 覆盖 | 读取/最终成员 | 实际调用 | sim | real |
| --- | ---: | ---: | --- | --- | --- | --- | --- |
| `PlannerParam::update_rate` | 10 Hz | 10 Hz | 否 | `PlannerParam::fromROSParam` → `pp_.update_rate=10` | planner timer、dt、rollout H、refresh ticks | 是 | 是 |
| `rolling_horizon` | 10 s | 15 s | 否 | `MultiVehicleConfig::fromROSParam` → `cfg_` → `rb_horizon_=15` | sim plan长度；real `/traj` rollout长度 | 是 | 是 |
| `rolling_refresh_period` | 2 s | 2 s | 否 | `cfg_` → `rb_horizon_refresh_period_=2` → `rb_horizon_refresh_=round(2×10)=20` | sim plan最大执行帧；real traj发布周期；TTC STOP decision period | 是 | 是，但不是 decision 周期 |
| `prediction_horizon` | 10 s | 15 s | 否 | `cfg_.prediction_horizon=15` | `RuleEngine::decide` 未 override 时的 pair horizon | fallback/one-shot；rolling plan 被 override | 顶层每拍及 real rollout每个未来帧 |
| `prediction_step` | 0.05 s | 0.05 s | 否 | clamp≥0.02 → 0.05 | `predictTrajectory`、同步 OBB samples | 是 | 是 |
| `dynamic_speed_far_threshold` | 10 s | 10 s | 否 | `cfg_=10` | FAR/MID 分类 | 是 | 是 |
| `dynamic_speed_near_threshold` | 5 s | 5 s | 否 | `cfg_=5` | MID/NEAR 分类 | 是 | 是 |
| `max_accel` | 0.20 m/s² | 0.20 | 否 | `cfg_=0.20` | prediction、sim advance、real coord ramp | 是 | 是；默认 PP actuator 不服从 coord ramp |
| `max_decel` | 0.30 m/s² | 0.30 | 否 | `cfg_=0.30` | prediction、STOP gate、sim/real coord ramp | 是 | 是；同上 |
| `dynamic_stop_time_margin` | 0.10 s | 0.10 s | 否 | `cfg_=0.10` | `evaluateTtcStopBoundary` | 是 | 是（计算） |
| `lat_accel_max` | 0.10 m/s² | 0.06 | **`a_lat=0.10`** | launch 最后覆盖为 0.10 | prediction/sim/real曲率限速 | 是 | 是（但 PP纵向另有实现） |

读取证据：`planner_param.h:16-37`、`multi_vehicle_config.cpp:31-178`；YAML：`planner_param.yaml:21,49-89`；使用证据：`multi_vehicle_patrol_node.cpp:98-101,676-677,753-761,1218-1231,2588-2683,2793-2798`、`rule_engine.cpp:3146-3151`。

### 8.2 两个“15 s”

1. `rolling_horizon` 决定 `rollWorldModel`/SimPlanFrame/发布轨迹的外层长度。
2. `prediction_horizon` 决定单次 `RuleEngine::decide` 内 NOMINAL future prediction 的长度。

仿真 rolling plan 显式传 `remaining_horizon = rolling_horizon - tau`，避免未来帧再打开完整窗口。实车 `rollWorldModel(..., plan_frames=nullptr)` 走 `decide(agents_,dt)`，每一未来帧都重新使用 `prediction_horizon=15`。因此当前确实存在两个 15 s 概念，且 real rollout 的组合语义不是目标设计的单一 15 s 观察窗。

## 9. TTC 草履虫模式生产调用链

### 9.1 ordinary ACTIVE-ACTIVE 真实路径

```text
RuleEngine::decide
  ↓ pairwise_horizon = override 或 prediction_horizon
resolvePairwiseConflicts
  ↓ 每车 predictTrajectory(NOMINAL, horizon, prediction_step=0.05)
detectPairInteractionFromPredictions(..., potential_zones={})
  ↓ 同步时刻 inflated OBB overlap
TimedConflictEvent(first_t, first_s_a, first_s_b, last_t, overlap geometry)
  ↓ ordinary=true 时
evaluateBridgeTtcCorrection
  ↓ collision_s / nearest opposing relation / backtrack / near_boundary_s
  ↓ corrected_ttc=min(original first_t, boundary_ttc)
effective_ttc = “让行侧”的 corrected_ttc
  ↓ classifyDynamicInterventionBand
FAR  effective_ttc >= 10      → priority=NOMINAL, other=NOMINAL
MID  5 <= effective_ttc < 10 → priority=NOMINAL, other=YIELD
NEAR effective_ttc < 5       → priority=NOMINAL, other=CREEP
  ↓ 仅 NEAR 检查 TTC braking boundary
effective_ttc <= 2.0 + (creep speed 0.05 / max_decel 0.30) + 0.10
即约 <= 2.267 s             → other=STOP
  ↓ applyActionRequest / applyRequestedActions
VehicleAction
```

源码：`spatiotemporal_interaction.cpp:132-176`、`rule_engine.cpp:1327-1346,1562-1577,1680-1855,1880-1910,3146-3158`、`bridge_ttc_correction.cpp:152-282,303-322`、`dynamic_speed_coordination.cpp:8-40,75-115,118-157`。

### 9.2 旧逻辑/其它权威核对

| 机制 | 当前生产状态 | 范围与影响 |
| --- | --- | --- |
| candidate 必须 15 s CLEAR | 生产不调用；`evaluateSelectedAction` 仅测试/诊断调用 | 不控制 ordinary |
| YIELD→CREEP→STOP 搜索梯 | 已从 production selection 移除 | ordinary 直接 band 映射 |
| alternate winner/order swap | ordinary production 未见 | winner 由 `priorityWinner` 固定 |
| ordinary `ConflictReservation` | `resolvePairwiseConflicts` 开头删除非 `a1_related` reservation；创建点只写 `a1_related` | ordinary 不由静态 reservation 控制 |
| 粗粒度资源仲裁 | `arbitrateResources` 调用被注释 | 不生效；deprecated 保留 |
| following override | `resolveFollowing`/`applyFollowingSuggestions` 有实现但 `decide` 未调用 | 当前 production 不生效 |
| forward clearance | `decide` 在 pair/A1/target 后调用 `enforceForwardClearance` | 两边均会触发；只应拒绝 next-step 物理非法动作，但会把 ordinary 动作加强为 STOP，并记录 duplicate authority metric |
| target occupancy/A1 special | 在 ordinary 后执行 | 两边均会触发；属于业务/特殊资源层，不是普通道路 band |
| cycle break | 仅配置 true 时调用；YAML false | 默认关闭 |
| current-pose replan recovery | 每 0.5 s 进入检测函数，但 YAML `enable_deadlock_recovery=false` 时在重规划前 return | 默认只检测；deprecated 代码仍存在但不执行 |

因此普通道路的主要控制权已经收敛到同步 OBB + direct TTC band；安全拒绝层和 A1/目标特殊层仍可合法加强动作。不能把它们描述成全部只有一个规则。

## 10. bridge TTC 在 sim/real 中的生效边界

`RuleEngine::resolvePairwiseConflicts()` 对 ordinary 且已有真实同步 OBB 冲突的 pair 调用 `evaluateBridgeTtcCorrection()`（`rule_engine.cpp:1712-1725`）。函数对双方分别：

```text
original first_t / collision_s
  → collision 点附近在另一条 path 上找最近点
  → 距离 <= vehicle_width + conflict_margin
  → actual motion heading dot < bridge_opposing_threshold
  → 沿本车 path 按 bridge_backtrack_step 回溯
  → relation 丢失前最后 near_boundary_s
  → 在 NOMINAL prediction 上插值得 boundary_ttc
  → corrected_ttc=min(original_ttc,boundary_ttc)
```

priority winner 确定后，代码选取让行侧的 corrected TTC 作为 `effective_ttc`（`rule_engine.cpp:1824-1835`）。sim 和 real 都调用该 RuleEngine，因此 **SOURCE CONFIRMED：两边都会计算 bridge correction**。

限制：

- 它只修正已经存在的 synchronized OBB conflict；baseline CLEAR 时不会调用。
- A1-related pair 走现有特殊 reservation 路径，不使用 ordinary bridge correction。
- real 每拍重算、sim 2 s commitment，时序仍不一致。
- 默认 PP 不订 `/coord_speed_i`；bridge 造成的每拍 action 只有在下一次 `/traj_i` rollout 重新发布后才可能间接改变 PP 的时间轨迹，不能宣称“bridge TTC 已端到端实时控制默认实车”。

## 11. `realbridge_exp.launch` 完整性审查

### 11.1 结论

**NO**：单独执行 `roslaunch forklift_planner realbridge_exp.launch` 不能由当前仓库证据保证完整实车进入可运行且 fail-safe 的状态。

### 11.2 自动覆盖事项

- roslaunch 可启动/连接 ROS master；
- 加载地图、planner、多车、controller 和 chassis YAML；
- 设置 `real_mode=true`、车辆数和四项 real 安全/到达参数；
- 启动 planner、map、RViz；
- 条件 include VRPN、1..8 个 controller；
- 试图启动 Nokov 和 chassis；
- start_slots/target_slots 来自 planner YAML，而不是 launch arg；
- planner 自己写协调日志/摆位文件，但没有独立 logger/bag 节点。

### 11.3 仍需人工/外部完成

1. 提供可被 ROS 找到并已构建的 VRPN client 与 chassis 包；当前仓库副本被 CATKIN_IGNORE。
2. 确保 Nokov Python 节点可执行，并安装 `numpy`、`scipy`、`jsk_recognition_msgs`、本地 `visualization.py`/`vehicle_param.py` 和 `sandbox_msgs`；这些未被 package.xml/CMake 完整声明或安装。
3. 启动外部 VRPN server，并确保 tracker 在 Nokov 启动约 2 s 的一次性枚举时已经出现。
4. 校准并验证 `world==map` 的数值关系，或提供正式变换。
5. 提供 `/rb_start` 和 `/estop` 发布者；仓库中未找到注释所称 `estop_key.py`。
6. 准备 `/dev/ttyUSB0`、57600 baud、`dialout`/udev 权限和正确固件/target 路由。
7. 修复/验证 controller 与 watchdog 后才允许解除架空轮测试。

### 11.4 依赖但未验证

- VRPN server IP、端口和单位；
- Vehicle0..7 刚体命名、yaw/轴方向和后轴参考点；
- 串口 target 字节与真实车辆 ID 映射；
- 底盘断串口、ROS publisher 消失、最后命令保持的固件行为；
- PP/LQR 的实车整定和 steering 符号；
- 当前 Linux 目标环境能否完整构建/解析 launch；
- RViz、logger 是否为现场必需（源码仅表明 RViz 对控制非必需）。

## 12. Launch/参数接入审查

### 12.1 顶层 arg

| arg/参数 | 声明 | 传入/设置 | 节点读取 | 最终使用 | 状态 |
| --- | ---: | ---: | ---: | ---: | --- |
| `server` | YES | VRPN include | YES | VRPN connection | LIVE |
| `controller` | YES | controller include selector | YES（launch eval） | PP/LQR 二选一 | LIVE |
| `lqr_q` | YES | `q_lat` | LQR YES | Q(0,0) | LIVE（仅 LQR） |
| `pose_timeout` | YES | global real_pose_timeout | planner YES | stale 判定 | LIVE，但默认 PP 端到端无效 |
| `arrive_tol` | YES | global real_arrive_tol | planner YES | real arrival | LIVE |
| `cmd_timeout` | YES | **未传入任何节点** | NO | NO | **DEAD PARAM** |
| `emerg_margin` | YES | global real_emergency_margin | planner YES | measured OBB guard | LIVE，但默认 PP STOP断链 |
| `veh_count` | YES | global vehicle_count + include 条件 | planner YES | agents/controllers 数 | LIVE |
| `target_only` | YES | planner private + include 条件 | planner YES | 单车过滤 | LIVE |
| `long_pid` | YES | `coord_speed_pid` private param | PP **NO** | NO | **DEAD PARAM** |
| `long_kp` | YES | 仅传到 include arg，未生成 node param | NO（此 arg） | 实际 kp 来自 gains YAML | **DEAD ARG / 被 YAML 独立定义** |
| `look_scale` | YES | 仅传到 include arg，未生成 node param | NO（此 arg） | 实际 lookahead_scale 来自 gains YAML | **DEAD ARG / 被 YAML 独立定义** |
| `a_lat` | YES | global lat_accel_max | planner YES | curvature speed | LIVE；覆盖 YAML 0.06 为 0.10 |

### 12.2 指定技术参数

| 参数 | 声明/加载位置 | 实际读取与使用 | 状态 |
| --- | --- | --- | --- |
| `coord_speed_pid` | `one_controller.launch` 设置 PP private param | PP 源码无读取；也无 `/coord_speed_i` subscriber | DEAD PARAM |
| `wheel_base` | planner YAML、map YAML、PP YAML 均有 | planner 已从 `PlannerParam` 移除，真正路径几何读 `MapParam` 的 map YAML；PP硬编码0.143；LQR硬编码0.1445 | DUPLICATE / 部分 DEAD / 非单一真值源 |
| `max_steer_angle` | planner YAML 0.40、map YAML 0.40 | planner YAML项不被 `PlannerParam` 读；MapParam项用于路径；chassis 独立硬限约0.50 | DUPLICATE / planner项 DEAD / actuator不一致 |
| `max_steer_rate` | planner YAML、map YAML | 仅 MapParam项用于路径几何；控制器/chassis未见 steer-rate limiter | DUPLICATE / planner项 DEAD / actuator未约束 |
| `path_resolution` | planner YAML、map YAML | 仅 MapParam读取使用；planner YAML项不被 PlannerParam读 | DUPLICATE / planner项 DEAD |
| `chassis/* gains` | real launch 加载 YAML | `chassis_node.cpp` 无 ROS param 读取 | 全部 DEAD PARAM |
| PP `wheel_base`, `control_dt`, `lookahead_distance_*`, `v_*` | PP YAML加载到私有 namespace | 当前 PP构造只读取 gains/tolerance/debug；这些名字未读，控制周期/限幅/轴距有硬编码 | DEAD/技术债（逐项以源码读取为准） |

## 13. Real Vehicle Deployment Gate

### 13.1 动捕门

| 检查 | 代码事实 | 结论 |
| --- | --- | --- |
| stale 阈值 | `real_pose_timeout=0.5 s`，launch可覆盖 | SOURCE CONFIRMED |
| startup 无 pose | start 前 planner不推进；但 `/rb_start=true` 即使缺车也允许 `rb_started_=true` | **DEPLOYMENT BLOCKER：启动门不是 all-pose hard gate** |
| ID缺失/越界 | 非 VEHICLE 或越界静默丢弃；对应 `real_pose_ok=false` | planner 会认为缺失；默认 PP安全效果仍断链 |
| 单车 pose 丢失 | planner只将该车 `/coord_speed_i` 目标压零 | 设计是 per-vehicle；默认 PP不订该 topic，**DEPLOYMENT BLOCKER** |
| planner 使用最新 pose | callback更新坐标；tick先 decide 后 realAdvance | decision 使用上一 tick 的 path_s/speed，约0.1 s延迟；P1 |
| controller pose watchdog | PP/LQR均无 | **DEPLOYMENT BLOCKER** |

### 13.2 实测碰撞硬护栏

- 使用实际 `/object` 的 `real_x/y/yaw`，不是规划 pose。
- OBB 来自 `MapParam` 的 vehicle geometry，默认 launch margin=0.08 m。
- 只检查当前实测 OBB overlap；没有独立 real next-step predictive guard。RuleEngine 内的 `enforceForwardClearance` 是基于 path state 的共同层。
- 可以绕过 RuleEngine 将输出目标置零，但除键盘 E-stop 外仍经过 `max_decel` ramp。
- 默认 PP不订 `/coord_speed_i`；rolling 模式 guard 也不会在每拍发布单点 hold trajectory。因此当前默认控制链不能证明该 guard 会立即停轮。

结论：**算法存在不等于实车保护生效；当前为 DEPLOYMENT BLOCKER。**

### 13.3 操作员急停

- planner订 `/estop`，true 时全车 speed target=0，且 `rb_cmd_speed` 瞬时归零。
- 发布者不在 launch，仓库中未找到注释所称键盘节点；依赖外部终端/焦点的细节 UNKNOWN。
- 默认 PP不订 speed；rolling 默认下不会因 estop edge 立即 `publishHoldAll()`，该函数只在 one-shot 分支使用。
- LQR会在下一控制 tick使用最新零 speed，但没有 `/estop` 独立硬通道，也无消息 timeout。

结论：**没有证据证明当前默认 PP 链的软件急停能使所有车立即归零；DEPLOYMENT BLOCKER。** 另需独立硬件急停，不应只依赖 ROS/GUI。

### 13.4 控制器与进程失效

| 故障 | PP | LQR | chassis/车辆 |
| --- | --- | --- | --- |
| trajectory timeout | 无；继续使用最后 latched trajectory | 无；继续最后 trajectory | 不感知 |
| coord_speed timeout | 不订阅 | 无；永久保留最后值 | 不感知 |
| pose timeout | 无；继续用旧 pose | 无；继续用旧 state | 不感知 |
| planner crash | 可继续跟最后15 s latched轨迹/时间参考 | 继续最后 speed/traj | 继续接 controller 命令 |
| controller crash | publisher停止 | publisher停止 | chassis无 command timeout，固件是否归零 UNKNOWN |
| chassis node crash | 无 ROS命令到串口 | 同左 | MCU最后命令行为 UNKNOWN |

这些均是上车前阻塞项。

### 13.5 底盘门

- `/dev/ttyUSB0`、57600 均硬编码，launch/YAML不可覆盖。
- 多车用共享串口和 `ChassisCommand.target`/packet首字节区分。
- open失败时进程返回1；运行时断线后的控制/归零行为没有显式处理。
- `cmd_timeout=0.4` 只是 launch 注释，未传递、未读取、未使用。
- `dialout`/udev/chmod 要求与设备稳定命名未在仓库自动化；需要现场处理。不要把一次性 `chmod` 当长期方案。

结论：**底盘 watchdog 与运行时断线 fail-safe NOT VERIFIED / DEPLOYMENT BLOCKER。**

## 14. 实车部署前 Checklist（按门控顺序）

以下顺序要求前一阶段通过并留存日志后才进入下一阶段：

- [ ] 在目标 Linux/ROS 环境记录 commit 与 dirty state；保留 `CATKIN_IGNORE` 的环境隔离事实，不擅自删除。
- [ ] 明确实车环境中 `vrpn_client_ros`、`chassis` 的正式来源并成功构建；确认 Nokov脚本可被 roslaunch 找到。
- [ ] 补齐并验证 Nokov Python/ROS runtime dependencies；运行 `roslaunch --nodes/--files` 保存展开结果。
- [ ] 验证 `/dev/ttyUSB*` 稳定映射、57600、udev/dialout权限、target→车辆映射；轮子架空。
- [ ] 验证 VRPN server、Vehicle0..N名称、100 Hz pose 和 Nokov 60 Hz `/object`；确保 tracker 在 Nokov一次性枚举前出现。
- [ ] 用已知1 m标尺验证 `/object` 为 mm；验证 yaw rad、正方向和车辆后轴参考点。
- [ ] 用至少两个非共线场地点验证 world→map 平移/旋转/尺度；未完成不得运动。
- [ ] 验证 `/object.id`、`/traj_i.target`、`/coord_speed_i`、`ChassisCommand.target`、串口车辆逐一对应。
- [ ] 实现/接入有状态的 `/rb_start` 与 `/estop` 节点；启动必须拒绝任何 enabled 车辆 pose 缺失。
- [ ] 修复默认 PP 对协调速度/STOP的服从，或明确改用经验证的控制器；禁止仅以 launch注释判定。
- [ ] 为 PP/LQR 增加 trajectory、coord speed、pose freshness watchdog；超时必须发布零命令。
- [ ] 为 chassis/固件增加按 target 的 command watchdog；验证 controller/planner/chassis crash 后轮端自动归零。
- [ ] 架空轮逐车测试正/负 throttle、steering符号、限幅、target隔离、串口拔出和节点 kill。
- [ ] 架空轮测试 planner stale、realHardGuard、软件E-stop；以 `/chassis` 和轮端结果证明归零延迟。
- [ ] 单车极低速测试路径起点、直行、转向、cusp倒车、终点停止；核对planner 0.143与LQR 0.1445差异。
- [ ] 单车测试 mocap loss、planner crash、controller crash、chassis restart；所有故障必须 fail-safe。
- [ ] 静止双车只发布状态，离线核对同步 OBB、priority、original/corrected TTC、FAR/MID/NEAR/STOP日志。
- [ ] 修复2 s commitment和real rollout horizon后，固定seed做 sim/real决策序列对照。
- [ ] 双车架空轮联调；确认一个target的命令不会驱动另一车。
- [ ] 双车场地低速测试；先远距离/FAR，再MID、NEAR，最后受控STOP；现场硬件急停始终可用。
- [ ] 只有全部 P0关闭、日志归档并经负责人批准后，才进入正式双车任务。

## 15. Findings

### P0 — 上实车前必须修

#### P0-01 默认 PP 未接 `/coord_speed_i`，规划安全 STOP 断链

- 现象：launch注释称PP纵向使用coord speed，但 PP 只订 `/traj_i` 和 `/object`。
- 证据：`pure_pursuit.cpp:274-280`；`coord_speed_pid` 无源码读取；planner的stale/guard/estop只在 `publishRealOutputs` 每拍修改 `/coord_speed_i`（`:3060-3108`）。
- 影响：默认 `controller:=pp` 下，0.5 s动捕失联、实测OBB guard、每拍TTC动作和rolling E-stop速度零值不能直接控制 `/chassis`。
- 建议方向：建立唯一、带freshness的纵向命令输入；PP必须消费并以 STOP为最高优先级，或发布独立安全制动通道。修改需完整审查控制器/底盘安全链。
- 阻塞实车：YES。

#### P0-02 `cmd_timeout` 为死参数，底盘无 watchdog

- 现象：launch声明0.4 s timeout，但未传入；chassis持续转发最后收到的命令，无按target计时归零。
- 证据：`realbridge_exp.launch:17`；`chassis_node.cpp:60-73,85-119`。
- 影响：controller崩溃或ROS图断链后的轮端行为取决于未知固件。
- 建议方向：controller和底盘/固件双层 watchdog；逐target最后命令时间；超时发送/保持零并记录故障。
- 阻塞实车：YES。

#### P0-03 world/map 坐标契约未实现或验证

- 现象：输入/轨迹写world，地图marker用map，无transform或校准参数，消费者忽略header。
- 证据：VRPN sample、Nokov v2、planner `objectCallback/publishHorizon` 和全仓 tf检索。
- 影响：即使尺度正确，平移/旋转误差也可使规划轨迹与真实仓库错位。
- 建议方向：正式定义world→map变换，在单一边界转换并验证；禁止隐含“两个frame数值相同”。
- 阻塞实车：YES。

#### P0-04 操作员启动/急停入口不完整

- 现象：planner必须收 `/rb_start` 才推进；仓库无发布节点。start回调允许缺pose启动；E-stop对默认PP无即时纵向权威。
- 证据：`setupRealIO`与`rbStartCallback/estopCallback`（`:2783-2790,2926-2943`）；仓库文件检索无`estop_key.py`。
- 影响：单独launch不会进入运行状态；软件急停不能被证明有效。
- 建议方向：将可靠操作面和状态机纳入部署入口；all-enabled-pose硬门；独立硬件E-stop和端到端停轮验收。
- 阻塞实车：YES。

#### P0-05 controller输入无freshness fail-safe

- 现象：PP/LQR无pose/traj timeout，LQR无coord timeout且未收到coord时回退自身throttle。
- 证据：两控制器构造、回调和control timer源码。
- 影响：planner/动捕故障后可能继续输出非零 `/chassis`。
- 建议方向：统一 command validity epoch、三类时间戳和 fail-closed；禁止无coord时自动回退非零速度的实车默认行为。
- 阻塞实车：YES。

#### P0-06 顶层launch依赖在当前仓库构建/安装边界内不闭合

- 现象：chassis、VRPN被CATKIN_IGNORE；Nokov脚本未安装且依赖未完整声明；planner package未声明实车launch runtime依赖。
- 证据：两个CATKIN_IGNORE、各CMake/package.xml、当前devel/lib盘点。
- 影响：新环境即使核心planner成功构建，顶层launch仍可能找不到节点/模块。
- 建议方向：不要删除环境隔离文件；在正式实车workspace建立有版本的overlay/依赖清单和launch preflight。
- 阻塞实车：YES，直到目标实车环境给出 BUILD/ROS GRAPH证据。

### P1 — sim/real 逻辑不一致

#### P1-01 real ordinary decision 为0.1 s，sim为2 s commitment

- 证据：`tick():2632-2643` 对比 `:2654-2683`；`rollWorldModel:746-765`。
- 影响：priority/action抖动、TTC band切换和行为序列不能由当前sim直接代表real。
- 建议方向：两边消费同一 RollingCoordinationPlan；状态adapter不同，决策artifact和commit规则相同。
- 阻塞实车：双车协调验证前 YES。

#### P1-02 real rollout 每未来帧重新打开15 s窗口

- 证据：real `publishHorizon` 调 `rollWorldModel` 不传plan_frames；`:760-762` 使用默认decide；RuleEngine默认 `cfg_.prediction_horizon`。
- 影响：外层15 s轨迹的尾部决策可能看到当前约30 s后的状态；与sim递减窗口不同。
- 建议方向：复用仿真的remaining horizon和period ordinary decision，不复制新规则。
- 阻塞实车：双车时序一致性验证前 YES。

#### P1-03 real先decide后投影最新状态

- 证据：`tick():2633-2635`。
- 影响：TTC/priority基于上一tick的path_s/speed；10 Hz下约0.1 s额外延迟。
- 建议方向：先形成传感器状态快照/投影，再做任务事件与统一协调；到达状态转换顺序需专门回归。
- 阻塞实车：与P0修复一起验证。

#### P1-04 PP/LQR执行语义不同

- 现象：PP按时间轨迹和位置PI决定纵向；LQR用coord speed。两者并非“只换横向控制器”。
- 影响：同一RuleEngine action在两种controller下结果不同。
- 建议方向：抽出共同纵向速度/STOP authority，PP/LQR仅实现横向或明确记录不同控制架构。
- 阻塞实车：默认控制器验收前 YES。

### P2 — 参数/launch技术债

- `cmd_timeout`、`long_pid/coord_speed_pid`、`long_kp` arg、`look_scale` arg、chassis gains为死参数。
- planner YAML仍保留已下沉至MapParam的wheelbase/steer/path resolution重复项。
- PP/LQR/chassis仍有0.143/0.1445、0.40/0.50等硬编码差异。
- `a_lat` YAML 0.06被launch默认0.10静默覆盖。
- `/velocity`无ID，不能作为多车可靠反馈接口。
- Nokov只枚举一次已存在topic，启动竞争条件未处理。

建议：先建立参数消费者表和launch preflight；安全阈值/几何值的任何变更必须按仓库规则单独立实验、审批和回归。本报告不建议本轮直接改值。

### P3 — 文档/可维护性

- launch及源码注释“协调与sim一致”“PP纵向用coord_speed”“cmd_timeout防跑飞”与实际调用冲突。
- `executeSimulationPlanSample` 将执行日志source标为`REAL`，容易把“真实执行帧”误读成real_mode。
- `realbridge_exp.launch` 注释引用不存在的 `exp_scripts/run_realbridge.sh`。
- Nokov package/CMake未表达Python实际runtime依赖和脚本安装。

## 16. 最小修改建议（仅方案，不改生产代码）

建议按以下最小边界分阶段，不做大规模重构：

```text
SimulationStateAdapter          MocapStateAdapter
advanceVehicles result         /object freshness + frame/unit transform + projection
            \                  /
             Unified CurrentVehicleState snapshot
                           ↓
             RollingCoordinationPlan
             - created every 2 s or explicit invalidation
             - one 15 s absolute end horizon
             - one ordinary priority/action set per period
                           ↓
              shared RuleEngine / OBB / TTC / bridge TTC
                           ↓
                    VehicleAction plan frames
                     /                    \
          simulation executor        real command adapter
          advanceVehicles            shared longitudinal authority
                                     PP or LQR lateral controller
                                     + real-only safety overrides
                                     + controller/chassis watchdogs
```

最小实施顺序：

1. 先修安全断链：PP/LQR统一消费带epoch/freshness的速度/STOP，controller与chassis watchdog fail-closed；补部署启动/E-stop节点和硬件E-stop验收。
2. 明确world→map和mm→m唯一转换边界；禁止各消费者分别猜单位/frame。
3. 把仿真已有的 `SimPlanFrame`/period ordinary decision 提炼为模式无关的 plan artifact；real不再每0.1 s重新选择ordinary action。
4. real先更新/投影传感器状态，再创建或执行plan；保留mocap timeout、实测OBB guard等real-only safety层。
5. real rollout传递相同的absolute end/remaining horizon，消除15+15窗口。
6. 清理死参数前先补参数接入测试；几何/安全阈值不得在无实验批准下改变。
7. 固定seed做sim执行action序列与录制mocap replay的real decision序列对照，再进入架空轮和低速实车门。

## 17. 最终回答

1. 当前从动捕到轮子的代码链是：VRPN pose → Nokov `/object` → planner/控制器 → planner的RuleEngine与`/traj_i`/`/coord_speed_i` → PP或LQR →共享`/chassis(target)` → chassis serial →车辆。默认PP绕过`/coord_speed_i`。
2. `realbridge_exp.launch` 启动大部分节点声明，但缺操作员start/E-stop发布器、可靠依赖闭包、坐标标定和watchdog；当前结论为 **NO**。
3. sim/real共用任务状态机、RuleEngine、priority、同步OBB/TTC、bridge TTC代码；在状态进入时序、2 s commitment、15 s rollout和actuator服从上仍分叉。
4. 0.1 s源码层成立；2 s仅sim是ordinary decision commitment，real只是traj refresh；15 s数值存在但real有嵌套重开窗口，未满足统一语义。
5. TTC草履虫和bridge TTC已进入real的生产计算调用链，但默认PP执行端不保证每拍服从其speed/STOP结果。
6. 上车前必须关闭本报告全部P0，并完成第14节逐级安全验证；当前不建议直接双车。
7. 下一阶段最小修改应先修端到端安全命令权威，再复用统一RollingCoordinationPlan；不要把mocap adapter与仿真积分器强行合并，也不要删除real-only安全层。
