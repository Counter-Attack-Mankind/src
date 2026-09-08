# EXP-20260908-A1-LATE-OWNER-REGRESSION

- Baseline branch/commit: `fix` / `6abe7d6`
- Working tree: implementation under test; prior runtime logs/cache preserved
- Configuration: simulation batch, 2 vehicles, 120 simulated minutes, `dt=0.1 s`
- Seeds: 2026, 2025, 2024
- Watchdog: fail on hard guard or any ACTIVE vehicle without path/task/phase progress for more than 60 s

## Batch results

| Seed | Status | Ticks | Sim time | Hard guard | Tasks V0/V1 | Max wait V0/V1 | Wedge episodes |
|---|---|---:|---:|---:|---:|---:|---:|
| 2026 | PASS | 72000 | 7200 s | 0 | 96 / 88 | 34.6 / 52.4 s | 6 |
| 2025 | PASS | 72000 | 7200 s | 0 | 96 / 87 | 38.9 / 31.9 s | 9 |
| 2024 | PASS | 72000 | 7200 s | 0 | 91 / 89 | 50.0 / 39.5 s | 5 |

No `A1-STOP-TTC` records were emitted. The spatial launch gate emitted `a1_stop_s_before_slot_clear` where a parked vehicle could not clear its slot before the frozen `waiter_stop_s`.

Normal deadlock PASS transactions cleared only after measured passer `path_s >= pass_clear_s`; representative seed-2026 values were `pass_clear_s=3.105`, `CLEAR pass_s=3.12366`.

The three deterministic runs did not naturally trigger `A1_LATE_OWNER`; the dedicated unit scenario covers fixed owner/intruder roles, owner-parallel retreat, target behind `waiter_stop_s`, and CLEAR without cooldown.

## Build and targeted tests

- `catkin_make -DCMAKE_BUILD_TYPE=Release --pkg forklift_planner`: PASS
- `deadlock_manager_test`: PASS
- `future_a1_policy_test`: PASS
- `dynamic_speed_rule_engine_test`: PASS
- `rolling_decision_timing_test`: PASS
- `bridge_ttc_correction_test`: PASS
- `dynamic_speed_coordination_test`: PASS
- `prediction_execution_consistency_test`: PASS
- `spatiotemporal_interaction_test`: FAIL at detector assertion `detector did not retain only the first overlap event`. No spatiotemporal detector source was modified by this change, but this test was not run before the implementation, so whether the failure predates it is unknown.
