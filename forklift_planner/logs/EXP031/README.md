# EXP-031 — baseline-only TTC rolling control, seed 2025 / slots 42,50

- Baseline commit: `e050c8a`
- Worktree under test: uncommitted baseline-only TTC control changes in
  `dynamic_speed_coordination.{h,cpp}`, `rule_engine.cpp`, and the two existing
  focused tests.
- Purpose: validate removal of the ordinary-road residual 15 s rollout while
  retaining one 15 s NOMINAL/NOMINAL baseline, per-vehicle bridge-corrected
  TTC, fixed priority, and the 2 s rolling refresh.
- Parameters: `vehicle_count=2`, `random_seed=2025`,
  `start_slots=[42,50]`, `batch_minutes=15`, simulation `dt=0.1 s`,
  prediction horizon `15 s`, rolling refresh `2 s`.
- Launch:
  `roslaunch forklift_planner multi_vehicle_phase2_batch.launch minutes:=15 vehicle_count:=2 random_seed:=2025 start_slot_a:=42 start_slot_b:=50`
- Primary log: `coordination.log`
- Debug snapshots: `debug/`

## Result

- Completed all requested `9000` ticks / `900.0 s`; process exited cleanly.
- Hard-guard collision events: `0`.
- Tasks completed: V0=`0`, V1=`0`.
- Maximum wait: V0=`898.0 s`, V1=`900.0 s`.
- Wedge episodes: `1`; deadlock detection ticks: `1746`.
- Plan 1 selected `NOMINAL/CREEP` from corrected baseline TTC
  `3.651/2.598 s`.
- Plan 2 selected `STOP/STOP`: priority TTC `1.630 s` was below its NOMINAL
  stop threshold `2.767 s`, and yielding TTC `1.791 s` was below its CREEP
  stop threshold `2.267 s`.
- Plans 3–450 remained `STOP/STOP`. Logs explicitly report
  `residual_evaluation=DISABLED`, so this outcome was produced by the two
  independent baseline TTC stop checks, not by a residual rollout.

## Conclusion

Build and focused behavior tests pass, but this fixed-seed scenario does not
pass the throughput/deadlock regression criterion. The requested pure baseline
TTC rule is implemented as specified; in this geometry, both independent
baseline stopping boundaries become true on the second rolling period and
remain true while both vehicles are stationary.
