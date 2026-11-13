# Long-term plan

This is the plan after v0.35. It is deliberately gated: no phase starts until the
previous phase's acceptance evidence exists. The project has already learned the
hard way that a 5-run sample can hide a 15–25% failure rate.

## Where we are (v0.35)

| Capability | Status | Evidence |
|---|---|---|
| gate-safe doorway, stand-off | ✅ **20/20, 0 contacts** | `results/repeat_report.md` |
| double-safe two doorways | ✅ **20/20, 0 contacts** | `results/repeat_report.md` |
| target1 offset doorway | ✅ **20/20 landings / 19/20 final-row** | `results/repeat_report.md` |
| depth projection convention | ✅ point gate PASS (83.4% within 1 m) | `results/map_gate.md` |
| occupancy grid from one frame | ✅ **PASS** precision 0.825 (2.5 m height floor) | `results/map_cells.json` |
| multi-frame grid / second frame pair | ⏳ next | Phase A remaining work |
| occupancy-map planning / SITL | ⛔ not started | — |
| dynamic obstacles, 3D, real sensor | ❌ not started | — |

The verified scope is static, hand-entered gate geometry at constant altitude,
with ideal Gazebo depth. That is the baseline to protect.

## Principles

1. **20-run acceptance** for any robustness claim; 5 runs are smoke tests only.
2. **Every number from `results/`**, produced by a committed script; no typed
   metrics.
3. **Keep failures.** A failed run is evidence, not something to overwrite.
4. **One change at a time.** Do not change margin, speed, timeout and map together.
5. **Safety gate before flight.** Map work must pass offline checks before SITL.
6. **No goalpost moving.** Do not lower a threshold merely to make a run pass.

## Phase A — Clean occupancy grid (next)

**Goal:** turn the recorded points into a grid that passes the existing gate.

Work:

- multi-frame accumulation: same wall observed from several poses, count repeats;
- stronger outlier rejection: height/region filters and connected-component size;
- optional ray/free-space carving from the known camera pose;
- keep the known-box precision, box coverage and start-free checks.

**Exit criteria:** `make map-gate` PASS with precision ≥0.80, 3/3 boxes covered,
start free, and a second recorded frame/pose pair also PASS.

**Status (v0.37):** the single recorded frame now passes at precision 0.825 after
raising the height floor to 2.5 m. The second frame/pose pair is still pending, so
Phase A is not fully closed and Phase B has not started.

**Do not:** loosen tolerance from 1.0 m to 1.25 m to pass. That was explicitly
rejected in v0.35.

## Phase B — Occupancy-map planner, offline then SITL

**Goal:** plan on the gated grid and fly it safely.

Work:

- add grid A* (or convert cells to a blocked set) with the same line-of-sight
  smoothing as the box planner;
- replay the recorded map offline and compare the path with the known geometry;
- add a map-ready/stale gate before the offboard advances;
- SITL: start empty, wait for map, plan, replan at a fixed rate;
- acceptance: 20 runs, 0 contacts, target error ≤1.5 m, minimum clearance ≥0.4 m.

**Exit criteria:** 20/20 with 0 contacts on the map mission, with the map gate
PASS in the same commit. Otherwise the feature stays unverified.

**Risk:** map latency and false cells; mitigation is the Phase A gate and a
"wait for map" state, not faster flight.

## Phase C — Dynamic obstacles

**Goal:** detect a moving obstacle and replan without contact.

Work:

- add a Gazebo actor/box moving across the corridor;
- timestamp map updates and detect changed cells;
- replan on change; bound replan rate and path churn;
- acceptance: 20 runs with the actor at fixed speed/phase; 0 contacts, no
  unbounded hover.

**Exit criteria:** 20/20, 0 contacts; report detection latency and replan count.

## Phase D — 3D / altitude reasoning

**Goal:** handle low walls or overhangs by changing altitude, not only 2-D detour.

Work:

- height-aware obstacle representation (wall top/bottom, vehicle altitude);
- layered 2.5-D grid or 3D A* with vertical clearance;
- acceptance: low-wall and overhang scenarios, 20 runs each, 0 contacts.

**Exit criteria:** the planner either climbs over a low wall or rejects the route
with a safe hold; both documented.

## Phase E — Real sensor and hardware

**Goal:** move from ideal Gazebo depth to a real depth camera and a real vehicle.

Work:

- RealSense/D435i or equivalent; calibration, time sync, holes/noise;
- map gate on real frames before flight;
- hardware-in-the-loop with a safety pilot, propeller guards, tether/fence;
- flight envelope: altitude, speed, geofence, failsafe, manual takeover;
- acceptance: repeated tethered flights, then free flights with a safety pilot;
- report sim-to-real gaps: detection range, map precision, control latency.

**Exit criteria:** a written flight-test report with real frames, contact-free
runs, and explicit differences from SITL. No claim transfers automatically from
SITL to hardware.

## Phase F — Mission-level product

Only after A–E:

- multi-target patrol and inspection;
- detection + avoidance + landing in one mission;
- operator UI / logs / replay;
- scheduled regression on a self-hosted SITL runner.

## Cross-cutting work

- **CI:** keep unit/static CI green; add a self-hosted SITL job for the 20-run
  suites (GitHub-hosted runners have no Gazebo/PX4).
- **Statistics:** n=20 is the minimum; for release claims prefer n=30–50 and
  report the failure rate, not just pass count.
- **Reproducibility:** every acceptance folder keeps per-run summary, landing,
  trace and log; commit summaries/landings, not huge logs.
- **Documentation:** one page per capability with verified/not-verified tables;
  failures and troubleshooting entries are part of the deliverable.
- **Safety:** target_0 is 0.8 m from the gate wall; exact landing is inside the
  propeller envelope. Stand-off is the correct product behavior, not a bug.

## Effort estimate

| Phase | Effort | Main risk |
|---|---:|---|
| A: clean grid | 2–3 weeks | outlier filtering is iterative |
| B: grid planner + SITL | 3–4 weeks | map latency/false cells |
| C: dynamic obstacle | 4–6 weeks | actor timing and replan churn |
| D: 3D | 4–6 weeks | representation and altitude control |
| E: hardware | 6–10 weeks | sim-to-real, safety, weather |
| F: mission product | ongoing | integration |

These are effort estimates, not dates.

## Stop/go gates

Do **not** start Phase B until Phase A passes. Do **not** start Phase C until
Phase B has 20/20 and 0 contacts. Do **not** claim hardware readiness from SITL.
If a phase cannot meet its gate after a bounded effort, write the negative result
and stop; do not lower the gate.

## What not to do

- no 5-run “verified” claims;
- no threshold lowering to pass a gate;
- no parameter sweeps across margin/speed/timeout at once;
- no exact target_0 landing claims;
- no occupancy-map SITL before the grid gate passes.
