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
| GIS-ready, structured output | export detections as GeoJSON with pose, timestamp, confidence and track id; validate the schema | small |
| QA over the data, not just the flight | a run report: frames received vs used, detection rate, association error, gaps | small |
| Model governance | a model card: intended use, metrics, failure modes, version, evaluation protocol | small |
| Association accuracy | already measured as landing error; the missing piece is reporting it per detection, not per trial | medium |
| Resolution/metadata standards | partially in `config/camera.yaml`; a "shot sheet" concept does not exist | large |

The first three are the ones that fit the project's own rule — every claim
measurable — and they are what an enterprise reviewer would look for before
trusting the numbers.
