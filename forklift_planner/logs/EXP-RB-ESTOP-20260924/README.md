# EXP-RB-ESTOP-20260924

- Purpose: close the software emergency-stop path from global `/estop` through
  the default Pure Pursuit controller to `/chassis`, without a real vehicle.
- Git commit: `6d250902eb47be98b60f19aa793b1053893e9722`.
- Workspace before experiment: clean (`git status --short` had no output).
- Applicable mode: realbridge real mode; default `controller:=pp` and all
  enabled vehicle targets.
- Parameter changes: none.
- Random seed: not applicable; no planning or coordination experiment was run.

## Modified code under review

- `MultiVehiclePatrolNode::estopCallback()` immediately publishes zero
  `/coord_speed_i` values and current-pose single-point hold trajectories.
- `MultiVehiclePatrolNode::publishHorizon()` rejects moving-horizon publication
  while global estop is asserted. Estop release invalidates the old real plan
  and forces a measured-state refresh.
- `PurePursuit::estop_callback()` immediately publishes a zero-throttle
  `ChassisCommand`; its timer continues publishing zero while stopped.
- Pure Pursuit clears its longitudinal PI/reference history on release and
  waits for the planner's refreshed trajectory before resuming control.
- `realbridge_keyboard_control.py` maps `0` to `/estop=true` and maps `1` to
  `/estop=false` followed by the existing `/rb_start=true` permission.

## Checks performed

- Python syntax compilation: PASS.
- ROS launch/package XML parsing: PASS.
- `git diff --check`: PASS.
- ROS Noetic catkin build: PASS.
  - Environment: WSL distribution `Ubuntu-20.04-ros`.
  - Command: `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo --pkg pure_pursuit forklift_planner -j2`.
  - `pure_pursuit_node`: compiled and linked.
  - `multi_vehicle_patrol_node`: compiled and linked.
  - The build emitted Windows/WSL clock-skew warnings but no compile or link
    errors, and completed at 100%.
- Static topic/publisher audit: PASS for the formal realbridge launch. It
  selects exactly one PP or LQR controller per vehicle target.
- ROS topic integration without `chassis_node`: PASS.
  - Started only `roscore` and `pure_pursuit_node` for target 0.
  - Published synthetic `/object` and `/traj_0` inputs.
  - Before estop: `/chassis throttle=0.26`.
  - After `/estop=true`: `/chassis throttle=0.0`.
  - After `/estop=false`, before a new trajectory: throttle remained `0.0`.
  - After a fresh `/traj_0`: throttle resumed to `0.26`.

## Checks not performed

- Full planner-to-controller realbridge graph: NOT RUN because it requires the
  experimental localization/runtime environment. The controller-side topic
  chain was tested independently as described above.
- Real-vehicle test: NOT RUN by design.

## Result

Static implementation complete, but runtime validation remains required in the
ROS Noetic environment. This record is not evidence of real-vehicle safety
certification.
