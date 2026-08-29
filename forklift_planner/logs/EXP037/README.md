# EXP-037: A1 first-departure closure audit

- Date: 2026-08-29
- Baseline commit: `6a8a136357c349780baf8c747510c92248b14418`
- Scope: first connected A1 departure closure selection, transaction-level
  farthest owner release boundary, and audit of the proposed next-A1 flow.
- Geometry: bare-body OBB, existing `0.025 m` ConflictZone sampling and
  `2.25 * step` two-arc merge gap. No new geometry/dropout threshold was
  introduced.
- Build: `catkin_make -DCATKIN_ENABLE_TESTING=ON --pkg forklift_planner -j4`
  passed in Ubuntu 20.04 / ROS Noetic.
- Existing focused checks passed: `future_a1_policy_test`,
  `conflict_zone_closure_test`, `spatiotemporal_interaction_test`, and
  `prediction_execution_consistency_test`. The Future A1 check includes a
  three-zone closure connected on both owner/waiter arc coordinates and a
  one-sided-transitivity rejection case.
- Gate scenario: `roslaunch forklift_planner
  multi_vehicle_phase2_batch.launch minutes:=2 vehicle_count:=2
  random_seed:=2025 start_slot_a:=42 start_slot_b:=50`.
- Gate result: FAIL. The run completed 1200/1200 ticks but recorded a hard
  collision at tick 450 (`sim_t=45.0 s`). The frozen first closure was
  `zones=[0]`, waiter physical/control boundaries were `4.425/4.385 m`, and
  the owner transaction boundary was `0.650 m`. It released at owner
  `path_s=0.663 m`; the subsequent disconnected conflict was detected by the
  ordinary rolling TTC layer too late to avoid collision. This is evidence
  of a release-to-ordinary-coordination handoff gap, not evidence that the
  disconnected remote zone belongs in the first A1 closure.
- No 30-minute run was attempted after the two-minute safety gate failed.
- The WSL `/tmp` coordination log was lost when the distro instance stopped;
  the reproducible command and extracted lifecycle/collision evidence above
  are retained, but the full log is not a durable artifact for this run.
