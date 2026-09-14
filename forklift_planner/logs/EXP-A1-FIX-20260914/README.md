# EXP-A1-FIX-20260914

## Purpose

Validate the approved Bridge TTC and A1 admission/intrusion changes with the
reproducible three-vehicle case, especially the distinction between rollout
discovery and REAL execution.

## Baseline and workspace

- Git commit: `3cc207c0ed6d9b6b3bb16ab54660441a79c17a53`
- The run used the working-tree changes in:
  - `a1_coordinator.h/.cpp`
  - `rule_engine.cpp`
  - `multi_vehicle_patrol_node.cpp`
  - `dynamic_speed_rule_engine_test.cpp`
- No safety threshold was changed.

## Reproduction

- Environment: WSL Ubuntu 20.04 ROS
- Vehicle count: 3
- Random seed: 2022
- Reproducible task random: true
- Start slots: `[38, 20, 36]`
- Simulation mode (`real_mode=false`)
- Final evidence runs: `run4` (4 minutes) and `run5` (3 minutes)
- Launch parameters were loaded from the repository map/planner YAML, with the
  values above overridden by the experiment launch.

## Build and focused tests

- `catkin_make --pkg forklift_planner -j2`: PASS
- `dynamic_speed_rule_engine_test`: PASS
- `bridge_ttc_correction_test`: PASS
- `dynamic_speed_coordination_test`: PASS
- `future_a1_policy_test`: PASS
- `rolling_decision_timing_test`: PASS
- `prediction_execution_consistency_test`: PASS
- `deadlock_manager_test`: FAIL at the pre-existing assertion
  `deterministic minimum retreat was not selected`; this prevents claiming a
  complete coordination regression pass.

## Runtime evidence

The final three-minute run (`run5/coordination.log`) proves that rollout state
is not the only place where intrusion correction exists:

- Plan 77 rollout creates V2 correction at `waiter_s=0.834`, `target_s=0`,
  `motion=RETREAT`; the same plan's REAL frame logs the same RETREAT.
- REAL execution then reduces V2 to `waiter_s=0.000` by plan 82, where the
  correction changes to `HOLD/a1_intrusion_target_braking` at the target.
- Plan 92 similarly creates V0 correction at `waiter_s=0.357`, `target_s=0`,
  and the same REAL plan applies RETREAT; plan 93 reaches `waiter_s=0.001`.
- Source launch admission is exercised in REAL: e.g. plan 41 holds V0 in
  DWELL with zero speed and `source_slot_hold=1`, and plan 49 later installs
  the path at `path_s=0` after admission becomes clear. Equivalent hold/release
  transitions occur for V2 (plans 57/71) and V0 (plans 78/86).

The four-minute run (`run4/coordination.log`) covers the blocked-sweep handoff:

- Plan 114 reports V0 `HOLD/a1_intrusion_retreat_sweep_blocked` with the real
  blocker `V1`, rather than `V-1`.
- DeadlockManager observes pair V0-V1 as a candidate in that REAL plan,
  confirms it at plan 115 after 4 seconds, and selects its existing
  `no_component_priority_swap` recovery (pass V0).
- Once the sweep becomes clear, plan 163 changes the REAL correction to
  RETREAT and plan 168 logs intrusion clear.

## Known failure and conclusion

Both final scenario runs still report the pre-existing first hard collision at
tick 233 (`sim_t=23s`, V0-V2), so the batch process exits as failed and this
experiment is not a clean full regression. The requested behaviors themselves
are evidenced: Bridge no longer changes the ordinary winner, B0-B9 launches
are held at the source, REAL intrusion retreat reaches `s=0`, and a blocked
sweep preserves its blocker and can be handed to the existing DeadlockManager.

Intermediate directories `run2` and `run3` record the diagnosis iterations.
The top-level first run is invalid for behavioral acceptance because an early
implementation changed a held vehicle to `TO_A1` before installing a path; that
implementation was reverted before `run2`-`run5`.
