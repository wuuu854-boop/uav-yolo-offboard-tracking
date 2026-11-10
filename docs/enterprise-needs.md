# What enterprises actually complain about

Two sources from the utility-inspection industry, read against this repository.
The point is to find problems that are (a) real, (b) named by people who pay for
the work, and (c) small enough to validate here instead of described.

## Source 1: a drone service provider's five data failures

[Detect, *5 Data Quality Failures Costing DSPs Millions*](https://detectinspections.com/blog/5-data-quality-failures)
(updated July 2026) is written by a vendor, so the numbers are theirs, and they
are the numbers a customer would quote back at you:

| Their failure | What it actually is | This project |
|---|---|---|
| 1. No standardised capture methodology | every pilot invents the shot list; coverage depends on who flew | **partly**: the mission and camera are config-driven and reproducible (`config/`, `docs/experiment-protocol.md`), but there is no shot-sheet concept |
| 2. Pilot variability | quality drops as you hire | **n/a** — no pilot; the equivalent is trial-to-trial variance, which the results files make visible |
| 3. Manual QA collapses under volume | 3–4 days of sorting per flight day | **no**: there is no QA pass over the data, only over the flight |
| 4. Misaligned expectations on resolution, metadata, format | utilities reject *formats*, not images | **partly**: the projections and poses are structured, but the deliverable is a CSV for us, not a GIS-ready layer for a customer |
| 5. Photo-to-structure association errors | GPS misassociation is 35% of their rework; 15–25% of imagery gets reworked | **this is the interesting one**: associating a detection with the correct asset is exactly what `projection.py` does, and the landing-error metric is a measured association error |

That last row is the reason to keep reading: the single most expensive failure in
their pipeline is a georeferencing problem, and this repository already has the
geometry, the frame conventions and the tests for it.

## Source 2: four utilities describing what they want next

[Unleash Live, *What US Utilities Are Actually Asking For Now*](https://unleashlive.com/blog/what-us-utilities-are-actually-asking-for-now/)
(July 2026) reports from sessions with Duke Energy, Southern Company, Entergy and
PG&E:

- **Entergy projects 500 TB of inspection data by 2027** and asks for GIS-linked
  prioritisation instead of raw footage.
- **The binding constraint is the gap between "a camera saw something" and "a
  person decided what to do"** — an architecture problem, not a storage problem.
- **FAA Part 108 moves compliance responsibility onto the operator.** Their
  advice: document model governance *before* it is mandatory.
- **Duke treats AI as institutional memory**, holding the judgement of retiring
  line workers.

Two of those four translate directly into work this repository can do without a
customer or an aircraft: a **structured, georeferenced deliverable**, and a
**model card** that a regulator could read.

## What that means for the build order

| Enterprise ask | Smallest honest version here | Effort |
|---|---|---|
| GIS-ready, structured output | **done**: `scripts/export_geojson.py`, sample in `results/detections.geojson` | small |
| QA over the data, not just the flight | **done**: `scripts/qa_report.py` — coverage, longest gap, track continuity, association error, exit code for CI | small |
| Model governance | **done**: [`model-card.md`](model-card.md) | small |
| Association accuracy | measured per detection now; per-detection confidence is not modelled | medium |
| Resolution/metadata standards | partially in `config/camera.yaml`; a "shot sheet" concept does not exist | large |

## The QA report's runs

`scripts/qa_report.py` over existing captures. The asset position is now an
input (`--asset east,north`) because the probe below proved that assuming
`targets.yaml` is a trap.

| Capture | Asset given | Level coverage | Longest gap | Track ids | Association median / p95 | Verdict |
|---|---|---|---|---|---|---|
| moving-target flight, 120 frames | as configured `(3.5, 0)` | 73% | 0.49 s | 1 | 6.09 / 7.14 m | FAIL: association |
| north-cube flight, 105 frames | true `(0, 3.5)` | **88%** | 4.42 s | 2 | **0.62** / 5.36 m | FAIL: gap, p95 |
| north-cube flight, same frames | as configured `(3.5, 0)` | 88% | 4.42 s | 2 | 4.95 / 5.14 m | FAIL: association |

Reading the middle row: with the asset where the cube actually was, the median
association drops from **4.95 m to 0.62 m**. That is the projection being right
and the input having been wrong. The p95 of 5.36 m is one of the two track ids
disagreeing — a second, spurious track — which is the track-count column doing
its job.

The first row is the report catching a real thing: the asset had been driven 3 m
away from its declared position, and every detection is legitimately metres from
it.

The lesson from the fourth measurement stands and is now implemented: a QA metric
is only meaningful against a known asset position. Coverage is also reported per
phase (ground / climb / level / descent) and the verdict judges the survey phase,
because takeoff and landing frames drag a whole-flight average down without
telling anyone anything.

Sample reports: `results/qa_moving.json`, `results/qa_north.json`.

`scripts/qa_report.py` over two existing captures:

The first three are the ones that fit the project's own rule — every claim
measurable — and they are what an enterprise reviewer would look for before
trusting the numbers.
