# EXP-VEHICLE-IDS-20260924

- Purpose: select sparse physical vehicle IDs in real mode without changing
  simulation numbering or A1/TTC/bridge/deadlock rules.
- Baseline commit: `e33625b4bbe94713adc4d9a9d31f90350f3af5f3`.
- Initial worktree: `realbridge_a1_cycle.launch` contained user-owned comment
  edits; its obsolete `veh_count/target_only` instructions were replaced as
  part of the requested launch migration.
- Parameters: real-mode `vehicle_ids=4,5,7`; simulation seed `2026`, two
  vehicles, 60 simulated seconds.

## Results

- `catkin_make`: PASS (100%); WSL reported clock-skew warnings.
- Python syntax and launch XML parsing: PASS.
- ROS topic/controller integration without `chassis_node`: PASS.
  - planner topics: `/traj_{4,5,7}`, `/coord_speed_{4,5,7}`,
    `/coord_state_{4,5,7}`;
  - controller nodes: `/controller_{4,5,7}`;
  - no V0/V1/V2 planner topics or controller nodes were present.
- Fixed-seed simulation batch: PASS, 600/600 ticks, zero hard-guard collision
  events. This is a regression aid, not complete test coverage.
- CTest: 7/8 passed. `spatiotemporal_interaction_test` failed, and failed the
  same way when rerun alone: `detector did not retain only the first overlap
  event`. The changed files do not modify that detector or test; this existing
  failure was not changed in this experiment.

## Not performed

- Full hardware realbridge launch and serial chassis execution: not run because
  the current environment lacks the real-vehicle runtime/hardware. Status:
  unknown, needs confirmation on the vehicle system.
