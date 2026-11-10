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
| Obstacle avoidance, LiDAR/depth | "path planning and obstacle avoidance with ROS 2" | **Not implemented** | — |
| Multi-target / counter-drone | "detection, tracking and classification of airborne targets" | Single target per run | `control/selection.py` |
| Dataset + labelling pipeline | not named, but implied by "model deployment" | **Geometry-based auto-labelling** on real frames | `scripts/label_real_frames.py` |

## What this tells us to build next

Ordered by (market weight × what is feasible without hardware we do not have):

1. **Multi-target mission** — *partly delivered*: `patrol_waypoints` now flies an
   ordered route and lands after the last waypoint (one SITL run, 0.44 m from the
   second target). The tracker also gained a constant-velocity model plus a
   nearest-centre bootstrap pass, so a moving target keeps its id across frames
   where the box moves faster than IoU can follow — unit-tested only: no moving
   target has been flown yet, and the README says so.
2. **TensorRT / INT8 path** — the most repeated phrase in the postings. Blocked
   on a Jetson or an NVIDIA GPU; the honest interim is the CPU benchmark we now
   have, plus an export path that is ready for it. *Needs hardware.*
3. **MAVLink bridge** — many stacks expect it even when the autopilot speaks
   DDS. A read-only MAVLink telemetry mirror is small and testable against
   `pymavlink`. *Feasible now, medium value.*
4. **EKF/estimator work** — real value, but it is PX4-internal tuning and needs
   flight data to mean anything. *Needs a vehicle.*
5. **Obstacle avoidance** — a depth camera or LiDAR in Gazebo would let us build
   it, but it is a second project, not an increment. *Defer.*

## What we deliberately stop pretending

The README says SITL only, and this page says why that matters: three of the
five postings are about things that happen on a vehicle — hardware buses,
vibration, EKF convergence, field trials. No amount of simulation closes that.
The value of this repository is that everything it claims is measured and
reproducible, not that it covers the field.
