# EXP-032 Real diagnostic logging and arbitrary single-vehicle launch

## Baseline and scope

- Date: 2026-08-30 (Asia/Shanghai)
- Branch: `fix`
- Baseline commit: `b830fb40232f9bf46d86e94a9f53b174f1470a67`
- Scope: diagnostic logging and launch selection only.
- Explicitly unchanged: RuleEngine/TTC/bridge/A1 arbitration, real projection
  selection, PP steering and longitudinal formulas, 2 s / 15 s rolling
  semantics, and vehicle kinematic parameters.

The pre-existing uncommitted removal of Markdown fences from
`realbridge_a1_cycle.launch` was preserved.

## Log lifecycle

The formal launch passes `$(find forklift_planner)/../log` to the planner and
PP controllers. This filesystem path resolves to the workspace `src/log`
directory. Both nodes create the directory if absent.

- Planner coordination and per-enabled-vehicle projection files open once at
  node startup with truncate mode.
- Each PP controller opens its fixed open-loop and tracking files once at node
  startup with truncate mode.
- During the node lifetime, samples and every received rolling trajectory are
  appended to the already-open streams; a trajectory refresh does not reopen
  or truncate a file.

## Validation

### Build

```text
catkin_make --pkg forklift_planner pure_pursuit
catkin_make --pkg forklift_planner
```

Result: PASS for both executables.

### Launch/XML checks

- `realbridge_a1_cycle.launch`: XML PASS.
- `one_controller.launch`: XML PASS.
- `roslaunch --nodes ... one_controller.launch target:=0`: `/controller_0`.
- `roslaunch --nodes ... one_controller.launch target:=5`: `/controller_5`.
- Static formal-launch condition evaluation:
  - `target_only=0`: `controller_0`.
  - `target_only=5`: `controller_5`.
  - `target_only=-1 veh_count=2`: `controller_0`, `controller_1`.
  - Declared controller targets: 0 through 7.

Full formal real launch execution remains unavailable in this VM because the
real chassis/VRPN packages are intentionally isolated with `CATKIN_IGNORE`.

### PP fixed-file lifecycle check

A temporary V5 PP node wrote to `/tmp/codex-pp-diagnostic`. Two synthetic
trajectories were published during the same node lifetime. The directory
contained only fixed `controller_V5_open_loop.csv` and
`controller_V5_tracking.csv` files. The open-loop file had one header followed
by two rows with `trajectory_seq=1` and two rows with `trajectory_seq=2`,
confirming that the second refresh appended instead of truncating.

### Planner V5 identity check

A temporary planner was started with `vehicle_count=1` and `target_only=5`,
writing to `/tmp/codex-planner-diagnostic`. It reported that only V5 receives
a task and that its route uses `start_slots[5] -> target_slots[5]`. It created
`real_projection_V5.csv` and no projection CSV for V0 through V4. No measured
pose or real chassis/controller was connected.

## Result

Diagnostic implementation and static/synthetic checks: PASS. Overall real
experiment validation: PARTIAL because no real vehicle, chassis, motion
capture, or complete formal launch chain was exercised. The reported stutter
and A1 turning behavior has not been fixed.
