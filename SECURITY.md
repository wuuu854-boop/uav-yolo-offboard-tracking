# Security policy

## Scope

This repository contains simulation and offline tooling: a PX4/Gazebo SITL
scenario, ROS 2 nodes, and Python modules for detection, projection and recording.
It runs no network service, stores no credentials, and ships no model weights.

The parts that touch real hardware are the ROS 2 nodes and `scripts/run_e2e.sh`.
If you fly a modified copy, the usual cautions apply: the controller commands
position setpoints and will arm a vehicle, so review the arming path before
pointing it at hardware you care about.

## Reporting a vulnerability

Open a draft security advisory privately:

<https://github.com/wuuu854-boop/uav-yolo-offboard-tracking/security/advisories/new>

Please do not open a public issue for anything that could arm or move a vehicle
unexpectedly, or that could make the recorder write outside `results/`.

What helps: the commit, a minimal reproduction, and whether it affects the
simulation only or could affect a real flight.

## Supported versions

| Version | Supported |
|---|---|
| `main` | yes |
| v0.1.x (latest release) | yes, fixes only |
| older tags | no |

## What is in this repository by design

- No secrets. `.env` is git-ignored and `.env.example` carries placeholders only.
- No personal data beyond the maintainer's GitHub handle.
- No binaries other than the demo GIF; models are downloaded or generated
  locally, never committed.
