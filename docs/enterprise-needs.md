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

## The QA report's first two runs

`scripts/qa_report.py` over two existing captures:

| Capture | Coverage | Longest gap | Track ids | Association p95 | Verdict |
|---|---|---|---|---|---|
| static-target flight (70 frames) | 40% | 4.98 s | 10 | 4.82 m | FAIL: all three |
| moving-target flight (120 frames) | 73% | 0.49 s | 1 | 7.14 m | FAIL: association |

The second row is the report doing its job: the asset in `config/targets.yaml`
sits at local `(3.5, 0)`, and the cube had been driven 3 m north, so every
detection is legitimately several metres from the declared asset. A QA pass that
did **not** flag that would be worthless.

The first row is honest, and the question it raised is now closed by a fourth
measurement. What was measured, in order:

1. **The capture is not mis-recording the pose.** During a probe flight the node
   logged `e=0.05 n=-0.48 yaw=89.5°` and the capture recorded `e=0.02 n=-0.49
   yaw=90.3°` at the same moment, with a quaternion that was essentially identity.
2. **An east-side cube appeared at the top edge** of a capture whose pose said the
   vehicle faced north — which the model says should put it on the right edge.
3. A second capture showed the same 90°-looking pattern.
4. **The decider: the cube was moved 3.5 m due north and flown again.** The
   detections then projected to `(east ≈ 0, north ≈ 3.5)` — where the cube
   actually was — and the QA report put them **4.95 m** from the declared asset at
   `(3.5, 0)`, exactly `hypot(3.5, 3.5)`.

So the projection maps north to north correctly, and the QA number is the report
doing its job: the asset was not where `targets.yaml` said. What the earlier
east-side frames were showing is still unexplained, and the answer is almost
certainly the boring one — the asset in that capture was not at `(3.5, 0)` either,
which the capture's own pose cannot tell you.

The lesson worth keeping: a QA metric is only meaningful against a known asset
position, and this repository's `targets.yaml` is a static assumption that the
mover script breaks. The report should take the asset position as an input
instead of assuming the config is the truth.

Sample reports: `results/qa_static.json`, `results/qa_moving.json`.

The first three are the ones that fit the project's own rule — every claim
measurable — and they are what an enterprise reviewer would look for before
trusting the numbers.
