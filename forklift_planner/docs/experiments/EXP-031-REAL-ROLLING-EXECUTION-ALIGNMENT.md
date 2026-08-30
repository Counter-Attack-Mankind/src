# EXP-031 Real rolling execution alignment

## Baseline and scope

- Date: 2026-08-30 (Asia/Shanghai)
- Branch: `fix`
- Baseline commit: `277280989229735b0e4a803165b0de7a506b83cc`
- Objective: align the formal real B -> A1 -> B rolling executor with the
  simulation timing semantics: 0.1 s measured-state/control ticks, a 2.0 s
  frozen ordinary coordination decision, and a 15.0 s prediction/horizon.
- Retained changes: `multi_vehicle_patrol_node.cpp`,
  `realbridge_a1_cycle.launch`, and this record.
- No safety threshold, vehicle geometry, controller, RuleEngine, TTC, bridge,
  path-generation, or public-message implementation was changed.

## Parameter source

The experiment used the checked-in `map_param.yaml` and `planner_param.yaml`.
Relevant effective values were:

- `update_rate: 10.0 Hz`
- `rolling_refresh_period: 2.0 s`
- `rolling_horizon: 15.0 s`
- `prediction_horizon: 15.0 s`
- `prediction_step: 0.05 s`

The new launch overrides only deployment identity (`real_mode=true`,
`use_a1_cycle=true`, `one_shot_traj=false`, `vehicle_count`) and controller
selection/interface arguments. It does not override algorithm or motion
parameters.

## Implementation under test

At the start of each real rolling period, the common simulation horizon
builder performs one ordinary decision and freezes its
`RollingDynamicDecision`. Predicted frames remain local to horizon generation.
Every live tick first projects the latest `/object` measurement through
`realAdvance()`, then invokes `RuleEngine::decide()` with
`reuse_ordinary_coordination=true` and the frozen decision. Non-ordinary rules
therefore continue to evaluate measured state every tick. Structural identity
changes can invalidate the period; normal `path_s`, speed, wait-time, and TTC
band evolution cannot.

## Commands and results

### Build

```text
source /opt/ros/noetic/setup.bash
cd /mnt/d/desktop/叉车
catkin_make --pkg forklift_planner
```

Result: PASS. `multi_vehicle_patrol_node.cpp` compiled and
`multi_vehicle_patrol_node` linked successfully.

### Existing focused tests

- `future_a1_policy_test`: PASS.
- `prediction_execution_consistency_test`: PASS.
- `rolling_decision_timing_test`: FAIL, reproducible twice with
  `MID fixture did not select YIELD/CREEP`. This executable and its RuleEngine
  dependencies were not changed or rebuilt by this patch, so the observed
  failure is recorded but not attributed to the executor change. Baseline
  status before this experiment is unknown and requires confirmation.

### Simulation smoke run

```text
timeout 20s roslaunch forklift_planner multi_vehicle_a1_rviz.launch start_rviz:=false
```

Result: PASS for startup/smoke scope. Map and planner nodes started, and the
simulation emitted 15.0 s / 150-frame plans at simulation times 0.10, 2.10,
4.10, 6.10, 8.10, and 10.10 s. The process was intentionally terminated by
the 20 s timeout; this is not a full batch regression.

### Synthetic measured-state real-chain check

The planner was started in real A1-cycle rolling mode with one vehicle. A
fixed B38 pose was published as `sandbox_msgs/AprilObject` on `/object` at
10 Hz, then `/rb_start` was asserted. No controller or chassis command was
connected.

Observed evidence:

- `rostopic hz /object`: stable average `10.000 Hz`, 0.100 s interval.
- Real plans were logged at planner times 50.00, 52.00, 54.00, ... s.
- Every log reported `horizon=15.00 frames=150 commit_frames=20`.
- `/traj_0` steady adjacent publication intervals were approximately
  `2.001 s` (the first sampling interval was partial).
- The per-vehicle real status continued updating between plan publications,
  while measured `path_s` remained 0.00 because the synthetic pose was fixed.

Result: PASS for the synthetic planner-only scope. It demonstrates 10 Hz
measurement projection and approximately 2 s horizon publication without
installing predicted frames into real state. It is not a real-vehicle test.

### Formal launch validation

- XML parse: PASS.
- Full `roslaunch --nodes ... realbridge_a1_cycle.launch veh_count:=1`: BLOCKED
  in the current VM because package `chassis` is intentionally excluded by
  `experiment/chassis/CATKIN_IGNORE`; the real experiment dependency chain is
  unavailable in this environment. The isolation file was not changed.

## Conclusion

Implementation and planner-only checks: PASS. Overall validation: PARTIAL,
because the formal launch could not be resolved against the intentionally
absent real-vehicle packages, the existing rolling timing unit fixture fails,
and no real vehicle or controller/chassis chain was exercised.
