# EXP-FMT-063 — FutureMissionTrajectory Phase 6.3

## Purpose

Validate a closed-loop, counterfactual FutureConflictCluster admission model
without changing RuleEngine, ConflictReservation, holder/waiter selection, or
the normal vehicle control path.

## Baseline

- Git commit: `cd758ff3a6b073280f46b2f014115b5753e207ff`
- Worktree: modified by the in-progress FutureMissionTrajectory Phase 0–6.3
  shadow implementation; no RuleEngine source diff.
- Environment: WSL `Ubuntu-20.04-ros`, ROS Noetic 1.17.4.
- Scenario: two vehicles, 120 simulated minutes, seed 2026, A1 cycle enabled,
  10 Hz, 15 s horizon, 2 s refresh, recovery disabled.
- The temporary experiment launch overrode only vehicle count and the private
  Phase 6.3 counterfactual switch. It was not committed.

## Build and deterministic tests

Command:

```text
source /opt/ros/noetic/setup.bash
catkin_make --pkg forklift_planner -DCATKIN_ENABLE_TESTING=ON
cd build && ctest --output-on-failure
```

Result: build passed; 9/9 registered tests passed. The Phase 6.3 test covers
waiter STOP/holder GO, all-member release, waiter resume, invalid admission,
and prevention of false release after a waiter has entered a member zone.

## 120-minute baseline result

- Tasks: V0=70, V1=68.
- Maximum continuous wait: 1708.7 s (V1).
- STOP vehicle-ticks: 59699.
- FIRST-WEDGE events: 1.
- Deadlock episodes: 1; detection ticks: 3328.
- Hard-guard collision: 0; recovery: 0.
- At 5484.3 s, cluster `[2,3]` created a shadow lock with holder V1,
  waiter V0 and stop boundary 2.640 m.
- By 5502.5 s both vehicles were inside different member zones and mixed
  zone holders were observed. The known permanent wait loop followed around
  5536 s.
- End state: V0 STOP `clear_block_V1` at s=4.403 m; V1 STOP
  `time_brake_V0` at s=0.442 m.

## 120-minute counterfactual result

- Tasks: V0=5, V1=4.
- Maximum continuous wait: 6827.1 s (V0).
- STOP vehicle-ticks: 138524.
- FIRST-WEDGE events: 1 at 397.9 s.
- Deadlock episodes: 1; detection ticks: 13595.
- Hard-guard collision: 0; recovery: 0.
- Counterfactual interventions: 508 STOP ticks, 44 GO overrides, 15 cluster
  releases and 15 waiter resumes.
- Dynamic admission failures: 6 late-braking events and 1 actual member-zone
  entry violation.
- First decisive failure: a new cluster was created at 345.0 s with its stop
  boundary only 0.01 m before entry. The waiter could not reach that boundary
  under the configured deceleration and entered the member zone at 345.3 s.
- A later lifecycle transition produced a new cluster whose entry was at the
  beginning of the new track. Admission was then unavailable because V0 was
  already inside. The persistent loop was V0 `clear_block_V1` and V1
  `time_brake_V0`; reconstruction from FIRST-WEDGE wait times places the
  closed loop at approximately 377.3 s.
- End state: V0 stopped at s=4.502/5.089 m with 6827.1 s wait; V1 stopped at
  s=0.000/6.956 m with 6822.7 s wait.

## Conclusion

Phase 6.3 does **not** satisfy the Phase 7 entry criteria. The experiment
cannot prove that cluster admission resolves the 5536 s deadlock because the
counterfactual world diverges into a new permanent deadlock at 397.9 s.

Two missing properties are demonstrated by code and run evidence:

1. Admission validity currently means geometrically before entry; it does not
   guarantee that the configured dynamics can stop at the boundary.
2. Cluster identity and closure are scoped to the current future segment pair.
   A holder can clear one segment-level cluster, transition to a new mission
   segment, and create a new conflict resource after the waiter has resumed.
   A lifecycle-spanning resource closure is therefore not yet represented.

The Phase 6.3 simulator remains default-off and diagnostic-only. No production
coordination rule was changed.
