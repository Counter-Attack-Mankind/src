# EXP-A1-REVIEW-20260914

- Purpose: read-only review of A1 departure intrusion correction and rollout/REAL execution.
- Git commit: `3cc207c0ed6d9b6b3bb16ab54660441a79c17a53`.
- Workspace before experiment: clean (`git status --short` had no output).
- Environment: WSL distribution `Ubuntu-20.04-ros`, ROS Noetic.
- Build: `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo --pkg forklift_planner -j2`.
- Configuration: `vehicle_count=3`, `random_seed=2022`, `reproducible_task_random=true`, `start_slots=[38,20,36]`, `use_a1_cycle=true`, simulation batch mode.
- Duration: 3000 ticks, `dt=0.1 s`, simulated duration 300 s.
- Persistent command inputs were supplied through a temporary launch file that was removed after the run. No production source/configuration was changed.

## Result

- A1 service metrics: create=7, hold=141, change=0, release=6, invalidate=0.
- A1 launch metrics: allow=6, hold=2, A1-prefix hold=2, retries=261, released-after-hold=2, max hold=13.2 s.
- Three intrusion corrections were created during rollout:
  - plan 76: owner V1, waiter V2, waiter_s=0.813, target_s=0.405, `a1_intrusion_retreat`.
  - plan 91: owner V2, waiter V0, waiter_s=0.357, target_s=0.230, `a1_intrusion_retreat`.
  - plan 113: owner V1, waiter V0, waiter_s=0.698, target_s=0.230, `a1_intrusion_retreat_sweep_blocked`.
- The corresponding departure clusters were installed into REAL by snapshot restore. Intrusion correction itself is not in the snapshot and is recomputed by the REAL executor.
- End state at 300 s: V0 remained at s=0.698 with `a1_intrusion_retreat_sweep_blocked` and wait=93.1 s.
- Hard collision guard events: 1, first at tick 233, pair V0-V2. This run is evidence for diagnosis only and is not a passing coordination regression.

## Related executable tests

- `bridge_ttc_correction_test`: PASS.
- `dynamic_speed_rule_engine_test`: PASS.
- `future_a1_policy_test`: PASS.
- `deadlock_manager_test`: FAIL (`deterministic minimum retreat was not selected`).

Artifacts: `coordination.log`, `screen.log`, and `debug/forklift_onset.log`.
