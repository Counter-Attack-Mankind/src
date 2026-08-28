# EXP-034 — ordinary TTC physical-priority safety and rolling STOP hold

- Date: 2026-08-28
- Baseline Git commit: `291ece8`
- Workspace: modified by this task; no unrelated changes were reset or removed
- Purpose: verify yielding effective TTC, priority future-vs-current physical
  TTC, one-sided bridge correction reuse, and a full 2 s TTC STOP hold.
- Parameter changes: none. Safety thresholds and vehicle geometry retain the
  configured values.
- Environment: WSL `Ubuntu-20.04-ros`, ROS Noetic, Release build.

## Scenario

- `vehicle_count=2`
- `random_seed=2025`
- `start_slots=[42,50]`
- `prediction_horizon=15.0 s`
- `rolling_refresh_period=2.0 s`
- simulated duration: `900 s` (`9000` ticks at `0.1 s`)

Command:

```text
roslaunch forklift_planner multi_vehicle_phase2_batch.launch minutes:=15 vehicle_count:=2 random_seed:=2025 start_slot_a:=42 start_slot_b:=50 coord_log_file:=.../logs/EXP034/coordination.log debug_log_dir:=.../logs/EXP034/debug node_output:=log
```

## Verification

- Build: `catkin_make -DCMAKE_BUILD_TYPE=Release --pkg forklift_planner` — PASS.
- Focused executables — PASS:
  `dynamic_speed_coordination_test`, `dynamic_speed_rule_engine_test`,
  `rolling_decision_timing_test`, `spatiotemporal_interaction_test`,
  `bridge_ttc_correction_test`, and
  `prediction_execution_consistency_test`.
- Batch completed: `9000/9000` ticks; hard-guard collision events: `0`;
  reciprocal STOP cycles: `0`; tasks: V0=`10`, V1=`12`.
- Ordinary `[DYN-SPEED] selected=STOP/STOP`: `0`.
- Priority physical safety STOP in this seed: `0`; yielding TTC safety STOP:
  `14`. The physical branch is covered by the focused test because this batch
  did not produce a priority-future/current-OBB hit.
- Plans 474–478 repeatedly detect the synchronized conflict while V1 remains
  stationary at `s_b=0.000`; the held vehicle therefore remains visible to
  collision prediction. Each rolling decision remains `NOMINAL/STOP`.
- An independent immediately preceding run with identical seed and slots
  produced an identical coordination log after normalizing only the
  experiment-directory name; its superseded intermediate artifacts were not
  retained.

## Conclusion

Keep the change. The fixed scenario completed without collision or reciprocal
ordinary TTC STOP. The focused tests verify that STOP prediction decelerates a
moving vehicle before holding stationary, that the hold lasts exactly one
2-second rolling period, and that a held vehicle remains a valid current OBB
for priority physical safety evaluation.
