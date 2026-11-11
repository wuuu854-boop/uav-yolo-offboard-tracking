# What the market asks for, and where this project stands

A sample of current UAV-autonomy postings (November 2026), read against this
repository. The point is not to collect buzzwords; it is to decide what to build
next and what to stop pretending.

## Sources

- Kapxha Dynamics, *UAV Autonomy/R&D Engineer* — defence-tech, India:
  PX4/ArduPilot custom modes and failsafes, MAVLink/MAVROS integration,
  detection and tracking, **Jetson + TensorRT deployment**, EKF tuning and
  GPS-denied navigation, ROS 2 obstacle avoidance, Gazebo SITL/HITL first.
  <https://wellfound.com/jobs/4817734-r-d-engineer-uav-robotics>
- Karta Innovations, *Aerial Robotics Engineer* — autonomy stack across
  simulation and flight.
  <https://wellfound.com/jobs/3975786-2-aerial-robotics-engineer-mid-senior-position>
- IMPACT Drones (YC), *Autonomy Engineer, Interceptor Drones* — tracking and
  interception of moving targets.
  <https://www.ycombinator.com/companies/impact-drones/jobs/yVR7VxO-autonomy-engineer-interceptor-drones>
- ASUS, *Senior Computer Vision Engineer* — model optimisation and deployment.
  <https://jobs.jobvite.com/asus/job/o8NpAfwE>
- Ambarella, edge AI for drones ahead of CES — the silicon vendors are pushing
  onboard inference, not cloud.
  <https://www.securityinfowatch.com/ces-show/news/55341169/ambarella-ambarella-doubles-down-on-edge-ai-for-drones-ahead-of-ces-citing-growing-demand-for-autonomous-aerial-systems>

This is a sample, not a survey: five postings and one vendor announcement. It is
enough to see which capabilities repeat, not enough to estimate salaries or
frequencies.

## The demand, condensed

| Capability | How it appeared | This project | Evidence |
|---|---|---|---|
| PX4 / ArduPilot, custom modes, failsafe | "work inside ArduPilot / PX4 … failsafe and geofence logic" | **PX4 via uXRCE-DDS, Offboard mode**, geofence and mission timeout in our own control loop | `docs/architecture.md`, `control/geofence.py` |
| MAVLink / MAVROS | "flight controller to companion computer integration over MAVLink" | **Not used** — we speak uXRCE-DDS/ROS 2 directly | — |
| Detection + tracking | "object detection and tracking" | **Both**, single target, IoU tracker plus per-target lock | `perception/`, `control/tracking.py` |
| Camera calibration | "camera calibration" | Intrinsics and mount offset in config, projection round-trip tested | `config/camera.yaml`, `tests/test_projection_forward.py` |
| Edge deployment, Jetson, TensorRT | "model deployment on NVIDIA Jetson with TensorRT" | **CPU ONNX Runtime measured**; TensorRT/INT8 **not done** | `docs/deployment.md` |
| Real-time performance | "real-time cognitive cycle" | 8.1 ms / 119 FPS at 320 px, 4 threads; end-to-end 143 ms | `docs/deployment.md` |
| Simulation first, SITL → real | "simulation first (Gazebo, SITL and HITL), then real flight" | **Gazebo Harmonic SITL**, 9+6 closed-loop trials, reproducible patch | `docs/experiment-protocol.md` |
| State estimation, EKF, VIO | "EKF tuning … GPS-denied navigation" | **PX4's EKF only**, no tuning, no VIO | — |
| Obstacle avoidance, LiDAR/depth | "path planning and obstacle avoidance with ROS 2" | **Reactive stop + L detour + depth side choice + back-off/search + known-map A\* + one-shot depth-to-box replan + rotated-box A\* (single and two boxes) + doorway landing** (v0.9–v0.22); no occupancy map or perception-driven multi-obstacle planning, no dynamics, no 3-D | `docs/avoidance.md`, `docs/detour.md`, `docs/side-selection.md`, `docs/blocked.md`, `docs/backoff.md`, `docs/backoff-sweep.md`, `docs/adaptive.md`, `docs/search.md`, `docs/global.md`, `docs/perception-planning.md`, `docs/rotated.md`, `docs/rotated-multi.md`, `docs/gate.md` |
| Multi-target / counter-drone | "detection, tracking and classification of airborne targets" | Single target per run | `control/selection.py` |
| Dataset + labelling pipeline | not named, but implied by "model deployment" | **Geometry-based auto-labelling** on real frames | `scripts/label_real_frames.py` |

## What this tells us to build next

Ordered by (market weight × what is feasible without hardware we do not have):

1. **Multi-target mission** — *delivered in part*: `patrol_waypoints` flies an
   ordered route and lands after the last waypoint (one SITL run, 0.44 m from the
   second target). The tracker gained a constant-velocity model plus a
   nearest-centre bootstrap pass, and a **real moving-target recording** now backs
   the claim: `scripts/move_target.py` drove `target_0` 3 m north at 0.5 m/s while
   the drone approached, the flight recorded 120 airborne frames, and the replay
   kept **one id across 88 detections** (`results/v07_moving_target.txt`).

   The honest reading: at 0.5 m/s and ~4 Hz the plain IoU association already
   holds the id — the motion model was not required for this run. It earns its
   place when the per-frame shift exceeds the overlap window, which is exactly
   what `tests/test_tracking_motion.py` sets up (30 px per frame on 40 px boxes).
   The first attempt at this recording captured 120 ground-level frames because
   the recorder attached before takeoff; the recorder now waits until the vehicle
   is airborne (`--min-altitude`).
2. **TensorRT / INT8 path** — the most repeated phrase in the postings. Blocked
   on a Jetson or an NVIDIA GPU; the honest interim is the CPU benchmark we now
   have, plus an export path that is ready for it. *Needs hardware.*
3. **MAVLink bridge** — many stacks expect it even when the autopilot speaks
   DDS. A read-only MAVLink telemetry mirror is small and testable against
   `pymavlink`. *Feasible now, medium value.*
4. **EKF/estimator work** — real value, but it is PX4-internal tuning and needs
   flight data to mean anything. *Needs a vehicle.*
5. **Obstacle avoidance** — *phases 1–8 delivered*: a forward depth camera, a
   latched stop (2.204 m clearance, zero contacts), a fixed-side L detour
   (0.527 m target error), a depth-based side choice (left chosen at the stop,
   0.714 m target error, zero contacts), a both-sides-blocked hold (35.0 s hold,
   zero contacts, mission timeout), one 2 m back-off (0.831 m target error,
   zero contacts), a bounded 0.2 m back-off sweep (0.852 m target error), a
   measured 2.0 m back-off (0.618 m target error) and a side-step search
   recovery from a genuine both-blocked stop (0.669 m target error, zero
   contacts) and a known-map A* path around the wall (0.24 m target error, zero
   contacts; reactive stop disabled, map hand-entered) plus a three-box
   multi-obstacle A* run (0.616 m target error, zero contacts), a one-shot
   depth-to-box replan (0.777 m target error, zero contacts) and a rotated-wall
   A* run (0.218 m target error, zero contacts) and a two-rotated-box run
   (0.586 m target error, zero contacts on either sensor) and a doorway landing
   (0.49 m target error, zero contacts). Still missing: occupancy-map perception,
   replanning after the handoff, dynamic or 3-D obstacles, and a real sensor.

## What we deliberately stop pretending

The README says SITL only, and this page says why that matters: three of the
five postings are about things that happen on a vehicle — hardware buses,
vibration, EKF convergence, field trials. No amount of simulation closes that.
The value of this repository is that everything it claims is measured and
reproducible, not that it covers the field.
