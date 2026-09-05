# EXP-034 DeadlockManager 独立重构

- 日期：2026-09-05
- 分支：`fix`
- 基线 commit：`2a0581b85bf1`
- 工作区：本实验保留原有用户修改，未执行 reset、checkout、stash、stage 或 commit。

## 目标与范围

删除历史 deadlock / cycle-break / stall-release / reverse / recovery /
predictive-hold 系统，新增由 `RuleEngine` 持有的独立 `DeadlockManager`。
第一版仅处理 ACTIVE + STOP + 互为 blocker + `path_s` 无推进的双车闭环，
执行 `RETREAT -> PASS -> CLEAR`。未引入强推、随机破环或临时换库位。

## 参数

- `deadlock_enabled: true`
- `deadlock_confirm_time: 4.0 s`
- `deadlock_retreat_search_step: 0.05 m`
- `deadlock_retreat_clearance: 0.02 m`

参数仅由 `MultiVehicleConfig` 读取，YAML 与源码默认值一致。

## 安全边界

- PASS 仅解除当前 deadlock pair 内的互相阻塞。
- 第三车冲突、target occupancy、forward-clearance 和 hard collision guard 仍可以拒绝通行。
- rolling rollout 使用 snapshot/restore，不向 live manager 累加虚拟 STOP/STOP 时间。
- 回退使用独立 motion directive，仿真中 `path_s` 减小；实车输出按当前路径段运动方向反号生成。

## 验证

环境：WSL `Ubuntu-20.04-ros`，ROS Noetic，Release。

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make -DCMAKE_BUILD_TYPE=Release
cd build/forklift_planner
ctest --output-on-failure
```

结果：

- `catkin_make`：成功。WSL/Windows 时钟存在约 10 s 偏差，make 输出 clock-skew 警告，但本轮受影响源文件与目标已重新编译且命令成功。
- CTest：10/10 通过，0 失败，2.14 s。
- 覆盖新增 manager 确认时间、候选选择、不可行结果、事务阶段以及实车投影的递减 `s` 语义。

## 未执行与结论边界

按本次任务要求未运行长时多车 batch、RViz 人工轨迹核对或实车试验。
因此仿真与单元回归已验证；实车控制器对逆向 rolling trajectory 的端到端执行能力
为“未知，需要确认”。
