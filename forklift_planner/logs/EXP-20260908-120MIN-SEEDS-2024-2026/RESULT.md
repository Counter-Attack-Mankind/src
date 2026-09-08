# EXP-20260908-120MIN-SEEDS-2024-2026

- Branch/commit: `fix` / `6abe7d6`
- Workspace before test: clean
- Mode: simulation batch, 2 vehicles, 120 simulated minutes, `dt=0.1 s`
- Seeds: 2026, 2025, 2024
- Stall criterion: an `ACTIVE` vehicle has no `path_s`, `path_gen`, task-count, or mission-phase progress for more than 60 s
- Runs: (1) 60 s stress watchdog, (2) full 120 min without early watchdog exit

## Results

| Seed | Full 120 min | 60 s watchdog | Hard guard | Tasks V0/V1 | Max wait V0/V1 | Verdict |
|---|---:|---|---|---:|---:|---|
| 2026 | 72000/72000 ticks | stopped at 4055.0 s | 1, first tick 40550 | 94 / 87 | 40.6 / 33.9 s | No 60 s stall observed, but run is not safety-successful because hard guard fired |
| 2025 | 72000/72000 ticks | stopped at 4991.0 s | 1, first tick 49910 | 92 / 87 | 47.4 / 41.3 s | No 60 s stall observed, but run is not safety-successful because hard guard fired |
| 2024 | 72000/72000 ticks | `NO_PROGRESS_V1` at 4614.2 s | 0 | 86 / 86 | 179.5 / 134.6 s | Failed 60 s stall criterion; repeated deadlock recovery did not promptly restore normal pair progress |

## Seed 2024 failure evidence

At 4554.2 s both vehicles became `ACTIVE + TO_A1 + STOP`:

- V0: `s=1.495`, blocker V1, initially `a1_admission_invariant_violation`, then `a1_stop_ttc_STOP_V1`.
- V1: `s=0.000`, blocker V0, `a1_admission_invariant_violation`.
- A departure cluster remained active with V1 as owner.

At 4614.2 s:

- V0: `s=0.971`, `STOP`, `wait=60.1 s`, then in restart hold after repeated short retreats.
- V1: `s=0.000`, `STOP`, `wait=60.1 s`, no progress for 60.1 s.
- The manager repeatedly emitted `CONFIRMED -> SELECT -> RETREAT_DONE -> PASS_START -> CLEAR`, while V1 remained at `s=0` and the same pair immediately formed again.

The full run eventually resumed and completed 86 tasks per vehicle, so this was not permanent loss of activity. It was nevertheless a sustained pair-level loss of normal mission progress exceeding one minute. The ordinary full batch exited successfully for seed 2024 because it does not treat no-progress as a failure; the stress-watchdog run does.

## Seed 2026 and 2025 safety failures

- Seed 2026, 4055.0 s: V0 `TO_A1` advanced to `s=1.556` while V1 `TO_B` was stopped at `s=5.608`; hard collision guard stopped both.
- Seed 2025, 4991.0 s: V0 `TO_A1` was transitioning from CREEP to STOP at `s=2.467`, while V1 `TO_B` reached `s=2.639`; hard collision guard fired.

Both full runs later continued to 7200 s and their maximum waits stayed below 60 s. They pass the narrow 60 s no-progress criterion but fail the broader safety-success criterion.
