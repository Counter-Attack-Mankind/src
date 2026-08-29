# EXP-038: A1 boundary correction gate

- Date: 2026-08-29
- Baseline commit: `6a8a136357c349780baf8c747510c92248b14418`
- Scope: retire A1 legacy reservation authority and feed the frozen first
  departure closure into the ordinary synchronized OBB/bridge/TTC chain as a
  waiter danger-boundary correction.
- Build: `catkin_make --pkg forklift_planner -j2` passed in
  Ubuntu 20.04 / ROS Noetic.
- Focused existing tests passed: `future_a1_policy_test`,
  `conflict_zone_closure_test`, `spatiotemporal_interaction_test`,
  `bridge_ttc_correction_test`, `prediction_execution_consistency_test`,
  `dynamic_speed_rule_engine_test`, and `rolling_decision_timing_test`.
- Gate command: `roslaunch forklift_planner
  multi_vehicle_phase2_batch.launch minutes:=2 vehicle_count:=2
  random_seed:=2025 start_slot_a:=42 start_slot_b:=50`.
- Gate result: **FAIL**. The 120 s run had zero hard-guard collisions and no
  A1 reservation creation/skip. A1 correction progressed through
  FAR/NOMINAL, MID/YIELD, NEAR/CREEP and STOP; V1 stopped at `path_s=4.167`,
  before the frozen first-closure physical entry `4.425`.
- First failure cause: the disconnected ordinary zone is
  owner `[0.800,1.800]`, waiter `[3.900,4.700]`. V1 is safely outside the
  first A1 closure but already occupies this ordinary zone when V0 departs.
  The retained priority physical-TTC guard stops V0 at `path_s=0.591`, before
  the first-closure release boundary `0.650`; V1 remains stopped by the A1
  boundary. The run therefore ends in STOP/STOP with a wedge episode.
- The disconnected zone was not added to the A1 closure. No 30-minute run was
  attempted because the short safety gate did not pass.
- Full coordination evidence: `coordination.log` in this directory.
