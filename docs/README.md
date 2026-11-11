# Documentation

| Document | Answers |
|---|---|
| [architecture](architecture.md) | Data flow, frame conventions, topic contract, why two nodes |
| [avoidance](avoidance.md) | Forward depth camera, the reactive stop gate, the SITL evidence, and what it does not cover |
| [detour](detour.md) | The L-shaped detour: why an L, the phases, the one run, and that it is not a planner |
| [model](model.md) | The YOLO detector: dataset, training, the decoder bug it exposed, and how it was made to fly |
| [deployment](deployment.md) | Detector throughput per input size and thread count, and what it does not measure |
| [market](market.md) | What UAV-autonomy postings ask for, where this project stands, what to build next |
| [review-guide](review-guide.md) | What each module guarantees, and the test that proves it |
| [experiment-protocol](experiment-protocol.md) | How a trial is run, what each metric means, spawn poses |
| [scenario](scenario.md) | Camera maths, range budget, why the targets are 20 m apart |
| [adoption](adoption.md) | What to change for your own target, camera or scene |
| [troubleshooting](troubleshooting.md) | Fifteen failures with the upstream source of each |
| [verification-log](verification-log.md) | What each non-obvious claim was checked against |
| [prior-art](prior-art.md) | The existing projects, what they lack, and the gap |
| [market](market.md) | Job requirements mapped to files in this repository |
| [enterprise-needs](enterprise-needs.md) | What utility-inspection companies actually complain about, and which of it this project can address |
| [shot-sheet](shot-sheet.md) | The written standard for a usable capture: views, resolution, framing |
| [model-card](model-card.md) | Model governance: intended use, data provenance, metrics, failure modes |
| [px4-notes](px4-notes.md) | PX4 specifics: uORB chain, versions, timestamps, ULog |
| [ai-usage](ai-usage.md) | What was delegated to an LLM and what was verified by hand |
| [setup-log](setup-log.md) | The versions actually used, as a fill-in template |
| [pre-release-checklist](pre-release-checklist.md) | The gate before a release |
| [release-notes-v0.1.0](release-notes-v0.1.0.md) | Draft body for the first release |

Procedures live in [`../skills/`](../skills/README.md); they are the "how", this
folder is the "what and why".
