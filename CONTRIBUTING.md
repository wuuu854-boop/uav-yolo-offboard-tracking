# Contributing

This is a small project, so the process is short.

## Before you open a PR

```bash
make check      # lint, format, links, doc claims, unit tests
```

CI runs the same commands plus a ROS 2 build. A PR that changes `src/` also runs
`ROS build`, which builds `px4_msgs` and `uav_interfaces` and executes the node
against real messages.

## What is welcome

- A fix for a failure in `docs/troubleshooting.md` that the entry does not cover.
- A skill in `skills/` for a problem you actually hit, with the command that
  verifies the fix.
- A bug in the projection, frame conversion or state machine, with a test that
  fails before your change and passes after.
- A run of the nine SITL trials on your machine, as a CSV.

## What needs evidence

Numbers. If a PR changes a claim in the README, it needs the row in
`results/*.csv` that supports it, or a test that computes it. "It looked better"
is not a reviewable statement.

## Style

- Commits: `type(scope): summary` with `feat`, `fix`, `docs`, `test`, `chore`,
  `ci`. The body explains **why**, not what the diff already shows.
- Docs are in English; comments in code may be either, but be consistent within a
  file.
- Do not commit weights, bags, flight logs, `.env` files or anything from
  `results/` that a script can regenerate.

## Reporting a problem

Use the issue templates. A useful report has: the exact command, the full error,
the versions from `python3 scripts/doctor.py`, and whether it also happens with
the baseline detector backend.
