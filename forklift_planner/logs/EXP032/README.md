# EXP-032 — original/effective TTC split

- Baseline commit: `e050c8a`; tested with the current uncommitted TTC changes.
- Scenario: 2 vehicles, seed `2025`, start slots `[42,50]`, 15 min / 900 s,
  15 s baseline and 2 s rolling refresh.
- Build and focused tests passed.
- Primary evidence: `coordination.log`.
- Plan 2: yielding changed to STOP while priority remained NOMINAL.
- Plans 3–5: yielding remained inside its source-slot sweep
  (`yielding_source_slot_clear=false`), so priority emergency remained
  ineligible even as original TTC fell below its NOMINAL stop threshold.
- The initial conflict subsequently cleared; unlike EXP-031, the run continued
  through plan 483 instead of remaining STOP/STOP through plan 450.
- The batch completed normally. The captured coordination file does not contain
  the console-only batch summary, so task-count and hard-guard totals are
  unknown from the persisted artifact and are not claimed here.
