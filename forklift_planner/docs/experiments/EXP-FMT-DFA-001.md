# EXP-FMT-DFA-001 — Dynamic Feasible Cluster Admission Shadow

## Scope

Evaluate a dynamics-aware, counterfactual cluster-admission constraint without
modifying `RuleEngine`, `ConflictReservation`, priority selection, or the
normal default-off execution path.

## Baseline and environment

- Git base: `cd758ff3a6b073280f46b2f014115b5753e207ff` plus the in-progress
  FutureMissionTrajectory shadow worktree.
- WSL `Ubuntu-20.04-ros`, ROS Noetic.
- Two vehicles, seed 2026, A1 cycle enabled, recovery disabled.
- 120 simulated minutes, 10 Hz, 15 s prediction horizon, 2 s refresh.
- Temporary launch overrode only vehicle count and the private counterfactual
  switch; no production launch or YAML value was changed.

## Implementation under test

`ClusterAdmissionConstraint` records the resource, holder/waiter, complete
member-zone list, cluster entry/exit, stop line, clearance, path generation,
current speed, curvature speed limit, acceleration/deceleration limits,
required/available braking distance, feasibility reason, and holder lifecycle.

`ClusterAdmissionEvaluator` mirrors the real `brakeBefore` stopping-distance
formula (`v^2/(2a) + v*dt`) and rejects constraints which are late, already
inside, path-invalid, single-zone, or created after mixed holders already
exist. It never relaxes a real RuleEngine action. A discrete stop-line
overshoot remains stopped only while the actual cluster entry is still
dynamically reachable without violation.

The counterfactual simulator rejects an opposite holder constraint while a
constraint for the same vehicle pair is active. This prevents the shadow layer
itself from creating simultaneous opposite pair locks; it does not alter real
reservations.

## Build and tests

```text
source /opt/ros/noetic/setup.bash
catkin_make --pkg forklift_planner -DCATKIN_ENABLE_TESTING=ON
cd build && ctest --output-on-failure
```

Build passed. All 10 registered tests passed. New deterministic coverage
includes feasible/infeasible braking, the exact runtime brake threshold,
discrete stop-line overshoot, late mixed-holder rejection, and opposite active
constraint rejection.

## 120-minute baseline

- Tasks: V0=70, V1=68.
- Maximum continuous wait: 1708.7 s (V1).
- STOP vehicle-ticks: 59699.
- FIRST-WEDGE: 1; deadlock episodes: 1; deadlock detection ticks: 3328.
- Hard-guard collisions: 0; recovery: 0.
- The known resource failure remained: cluster `[2,3]` became mixed-holder in
  the 5484–5536 s interval and then remained deadlocked.

## 120-minute dynamic counterfactual result

- Tasks: V0=5, V1=4.
- Maximum continuous wait: 6840.1 s (V1).
- STOP vehicle-ticks: 138890.
- FIRST-WEDGE: 1 at 126.5 s; permanent deadlock detected at 390.6 s.
- Hard-guard collisions: 0; recovery: 0.
- Dynamic STOP ticks: 69228; action changes: 68815.
- Cluster releases / waiter resumes: 9 / 9.
- Actual cluster-entry violations: 0; holder GO overrides: 0.

The final decisive control was recreated at 358.3 s immediately after a prior
cluster release. At deadlock detection, the minimum wait cycle was:

- V0: `STOP time_brake_V1`, blocker V1, `s=0.240/3.524`.
- V1: `STOP counterfactual_cluster_wait_V0`, blocker V0,
  `s=2.217/5.089`.

The real RuleEngine then selected V1 as holder for both current single zones,
while the persistent counterfactual constraint still held V0 as cluster
holder. Because the shadow layer is monotone and is forbidden to relax the
real V0 STOP, neither vehicle can clear the lifecycle that releases the
constraint.

## Conclusion

Dynamic braking feasibility is necessary but not sufficient for formal
cluster admission. The implementation successfully avoids infeasible braking,
hard collisions, and actual member-zone entry, but it cannot guarantee
progress while its persistent holder is not the same authority used by the
real RuleEngine on subsequent refreshes.

Phase 7 entry criteria are **not met**. Formal integration would require one
atomic arbitration interface where cluster admission and the real holder are
the same decision, with lifecycle release/revalidation performed before action
merge. Adding more shadow STOP filters cannot prove liveness and risks hiding
the authority mismatch.
