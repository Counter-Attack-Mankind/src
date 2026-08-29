# EXP-036: A1 control stop boundary and coordination module

- Date: 2026-08-29
- Baseline commit: `e448851d64f7a9a1946a85e2327e9172eda3064e`
- Workspace: existing uncommitted changes in
  `dynamic_speed_coordination.cpp` and
  `spatiotemporal_interaction.cpp` were preserved.
- Purpose: separate the bare physical ConflictZone entry from the A1 waiter
  control stop target, migrate A1 resource policy into `A1Coordinator`, and
  keep real-mode departure protection until fresh physical pose confirms that
  the protected cluster has been cleared.
- Safety parameter: new `a1_control_stop_margin=0.04 m`; it is an independent
  upstream A1 control target offset. It does not expand ConflictZone/OBB and
  does not replace the braking trigger
  `v^2/(2*max_decel) + v*dt`.
- Scenario: Ubuntu 20.04 ROS Noetic, two vehicles, `random_seed=2025`,
  `start_slots=[42,50]`, 30 simulated minutes, `dt=0.1 s`.
- Command: `roslaunch forklift_planner multi_vehicle_phase2_batch.launch
  minutes:=30 vehicle_count:=2 random_seed:=2025 start_slot_a:=42
  start_slot_b:=50`.
- Build: `catkin_make -DCATKIN_ENABLE_TESTING=ON --pkg forklift_planner -j4`
  passed.
- Existing checks passed: `conflict_zone_closure_test`,
  `future_a1_policy_test`, `bridge_ttc_correction_test`,
  `spatiotemporal_interaction_test`, and
  `prediction_execution_consistency_test`.
- Batch result: 18000/18000 ticks, 1800.0 simulated seconds,
  `hard_guard=0`, deadlock detections=0, reciprocal STOP cycles=0. V0/V1
  completed 19/18 tasks; maximum recorded waits were 44.9/53.4 seconds.
- A1 service summary: create=38, hold=849, release=35, invalidate=2;
  launch allow=37, hold=3, and all three holds later released.
- Stop-boundary diagnostics: 12 Future admission STOP decisions, maximum
  `stop_line_overshoot=0.000 m`, minimum `physical_entry_remaining=0.058 m`,
  and no `already_inside=true`. Departure-cluster rollouts had at most
  0.002 m control-line overshoot while retaining at least about 0.038 m to
  the bare physical entry. All 12 first admission triggers reported
  `braking_feasible=false`, so Future authority timing remains a diagnostic
  item even though no physical entry or collision occurred in this run.
- Ordinary dynamic-speed execution remained active (four near periods, one
  resolved transition sequence); no obvious TTC/bridge regression appeared
  in this scenario.
- Result: PASS for this simulation scenario. Real-mode release protection was
  compiled and code-reviewed only; no physical-vehicle validation was
  performed, so real-vehicle validation remains unknown and required.
- Artifact: `final_coordination.zip` contains the full coordination log.
