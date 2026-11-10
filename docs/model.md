# The YOLO detector: what was trained, what it does, and what it does not

The v0.1.0 trials were flown with the colour baseline. This page is about the
other backend — the one the README's comparison table needs — and about an
honest negative result at the end.

## Why train at all

A COCO-pretrained YOLO detects these synthetic cubes zero-shot never: the world
contains flat coloured squares, which are not in its classes. Training a small
model for this scene is the shortest path to a real YOLO row in the table.

## Dataset

`scripts/make_synthetic_dataset.py --train 500 --val 100 --size 640`

- seeded (7 for train, 8 for val), so the split is reproducible;
- nadir view: flat grey ground, gaussian noise σ≈4;
- one or two squares, side 28–130 px, rotation ±20°, one of the three target
  colours with ±12 per-channel jitter, a brighter inner face half the time, and
  a soft shadow offset by (0.14, 0.16)·side;
- YOLO labels from the polygon's bounds, normalised.

Four unit tests keep the geometry and the label normalisation honest.

## Training

| Item | Value |
|---|---|
| Ultralytics | 8.4.208 |
| torch | 2.14.1+cpu |
| Model | `yolo11n.pt` (pretrained), fine-tuned |
| Input | 320 px |
| Epochs / batch | 30 / 32 |
| Time | ~20 min on 24 CPU threads |

Validation on the 100 held-out images:

| Metric | Value |
|---|---|
| Precision | 0.974 |
| Recall | 0.951 |
| mAP@50 | 0.962 |
| mAP@50-95 | 0.665 |

## Export

```bash
yolo export model=best.pt format=onnx imgsz=640 opset=12 simplify=True
```

11 MB, input `(1, 3, 640, 640)`, output `(1, 5, 2100)` for one class. The file
is git-ignored; the commands above reproduce it.

## A decoder bug this exposed

`decode()` accepted a `(1, 4 + C, N)` export and treated the five channels as
five boxes. Every detection carried coordinates from the wrong axis and a
"confidence" above 1. The YOLO backend could not have flown before the fix, and
no unit test noticed because they all built `(N, 4 + C)` tensors by hand.

Two tests now cover channel-first tensors, including a single-box export.

## Held-out comparison (batch, 100 images, 640 px)

| Backend | Detected | p50 | p95 |
|---|---|---|---|
| Colour baseline | 100/100 | 1.6 ms | 2.1 ms |
| YOLO11n, 640 ONNX | 100/100 | 7.8 ms | 9.1 ms |

The baseline is ~5× faster. Both find every square in this distribution.

## On real Gazebo frames

Sampled from `assets/01-demo.gif` (the committed demo, 123 frames at 640×480):

| Frame | Baseline | YOLO |
|---|---|---|
| no cube | 0 | 0 |
| cube visible | 1 near the top centre | 1 within ~20 px of the baseline |

So the model transfers to the renderer for the true positive. The problem is
what else it finds.

## The negative result: YOLO does not close the loop here

Three SITL trials with `backend: yolo` (640 input) did not converge. The trace
shows the vehicle chasing an estimate north-west of the spawn while the target
sat east, then losing it and searching:

```
GOTO target missing 2.0s -> HOVER(target_lost) -> SEARCH at (-4.5, 2.2)
```

The likely cause: the synthetic dataset only contains shadows *attached to*
squares, so the model is free to call the vehicle's own shadow — a dark
quadrilateral in every frame — a target. The baseline cannot make that mistake
because it matches a specific hue.

## Round two: hard negatives

`scripts/harvest_negatives.py` takes the 123 frames of the committed demo GIF —
real Gazebo renders — keeps the 45 where the colour baseline finds nothing (the
target is out of view, the vehicle's shadow is not) and writes them as
background examples with empty labels. Training data goes from 500 to 545
images, same schedule.

| Metric (held-out synthetic) | Round one | Round two |
|---|---|---|
| Precision | 0.974 | **0.984** |
| Recall | 0.951 | 0.937 |
| mAP@50 | 0.962 | **0.981** |
| mAP@50-95 | 0.665 | 0.671 |

The false positives are gone: the trials now **land** instead of wandering
(`results/v03_yolo_trials.csv`). They do not land consistently:

| Attempt | Landing error | Within 1.5 m |
|---|---|---|
| 1 | 2.208 m | no |
| 2 | 2.362 m | no |
| 3 | 2.278 m | no |
| 4 | 0.515 m | **yes** |
| 5 | 0.638 m | **yes** |
| 6 | 2.176 m | no |
| 7 | 1.654 m | no |

Two of seven, against nine of nine for the baseline.

## The controlled comparison

Separate flights cannot say where the offset comes from: the two backends saw
different frames. `scripts/capture_pose_frames.py` records the camera frame and
the odometry together during one trial (`CAPTURE_DIR=... bash
scripts/run_trials.sh`), and `scripts/compare_backends.py` runs both backends on
each recorded frame and projects their boxes with the recorded pose.

30 frames had a detection from both (`results/backend_comparison.txt`), recorded
during a baseline trial; a second capture during a YOLO trial reproduced it over
30 further frames (`results/yolo_close_range.txt`):

| Quantity | Baseline trial | YOLO trial |
|---|---|---|
| Mean delta, YOLO − baseline | (+0.49, +0.35) m | (+0.52, +0.39) m |
| Std of that delta | (0.11, 0.20) m | (0.14, 0.17) m |

So the per-frame projection differs by about 0.6 m, consistently, in one
direction — the YOLO box centre sits slightly off the cube's centre, which is
what a box that also covers part of the shadow looks like. It is stable across
both backends' flights, including the one that landed at 0.515 m.

That is **not** the 2.2 m the failing trials land with. The detector is not the
source of the variance: the same frames give the same 0.6 m.

## Replaying a failing trial

Three further trials were run with the recorder on (`CAPTURE_DIR`), which produced
one success (0.64 m) and two failures (2.18 m, 1.65 m). Replaying the failures'
frames (`results/yolo_failure_trial2.txt`, `..._trial3.txt`) shows where the
2.2 m comes from, and it is not the detector:

```
trial 2, last frames
frame    alt  drone e  drone n  base err  yolo err  delta e  delta n
0066    4.93     3.20     1.58      1.86      2.62     0.79     0.29
0067    4.92     3.95     1.67      0.27      1.43     0.98     0.65
0068    4.94     4.57     1.69      1.40      0.20     1.18     0.73
0069    4.96     5.06     1.66      2.29      1.35     0.96     0.93
```

- The vehicle is at `north ≈ 1.66` while the target is at `north = 0`: it flew
  past the target and settled a metre and a half to the side.
- **Both** backends' errors swing by a metre from one frame to the next
  (`base err` 0.27 → 1.40 → 2.29), while their difference stays at 0.5–1.2 m.

A projection that assumes a level, nadir camera cannot hold still while the
vehicle pitches to accelerate at 1 m/s: 10° of pitch at 5 m moves the ground
point by `5·tan(10°) ≈ 0.88 m`. That is the same order as the swing, and it
affects both backends equally — which is exactly the pattern above.

So the remaining YOLO variance is a **projection** limitation shared with the
baseline, amplified for YOLO because its estimate starts 0.6 m off and the
arrival check has less margin.

## The fix: attitude in the projection

`projection.py` now takes the vehicle's PX4 roll and pitch, builds the
body-to-ENU rotation instead of multiplying two tangents, and intersects the ray
with the ground. Level flight is bit-identical to the old path — every existing
projection test passes untouched — and eight new tests pin the attitude
behaviour (`tests/test_projection_attitude.py`, `tests/test_frames_attitude.py`).
The node feeds it the quaternion-derived angles, and the capture tool records
them so the same comparison can be replayed.

Three trials per backend with the fix (`results/v04_attitude.csv`):

| Backend | Landing errors | Within 1.5 m |
|---|---|---|
| Baseline | 0.862, 1.003, 0.655 m | 3/3 |
| YOLO | 0.640, 1.572, 1.493 m | 2/3 |

The 2.2 m failure mode is gone: the worst YOLO run is now 1.57 m instead of
2.36 m, and the distribution is tight around the baseline's. The baseline did not
change (0.66–1.00 m), which is the control that says the fix is not moving
everything.

YOLO still starts 0.6 m off, so it sits at the edge of the 1.5 m tolerance.
Removing that last bias is a detector job, not a projection one — the box
covering part of the shadow — and the honest summary stays: **the measured loop
is the baseline.**

The measured closed loop therefore remains the colour baseline, and the honest
summary of the YOLO backend is: **it flies, and the reason it missed is now
measured, fixed, and re-measured.**

**The measured closed loop therefore remains the colour baseline.** The YOLO row
stays out of the results table until it can match it, and `results/v03_yolo_trials.csv`
is committed so the claim can be checked rather than trusted.

## The boxes are the problem

The 0.6 m difference was measured through `bbox_cx`/`bbox_cy`, which hides what
the boxes actually look like. Printing the raw pixels tells a blunter story
(`results/yolo_pixel_boxes.txt`, 1280×960 frames, cube ≈ 54 px):

| Frame | Baseline (u, v) | YOLO (u, v) | YOLO w × h |
|---|---|---|---|
| 0048 | 661, 51 | 704, 17 | **436 × 115** |
| 0057 | 638, 169 | 690, 130 | **450 × 207** |
| 0063 | 732, 189 | 803, 139 | **464 × 236** |
| 0066 | 741, 467 | 805, 405 | **463 × 493** |
| 0069 | 490, 585 | 523, 512 | **490 × 535** |

The model predicts boxes half the frame wide for a 54 px cube. The centre lands
near the cube, which is why the closed loop still flew — the projection only uses
the centre — but the box is worthless for anything else, and the centre wanders
with the box.

Both models do this (round one and round two, at 320 and at 640), so it is not
the hard negatives and not the export size. It is domain shift: the same model
returns tight, confident boxes on the synthetic validation set (mAP@50 0.98) and
on the 640×480 demo GIF, and inflated boxes on the live 1280×960 stream.

## What this does not change

The attitude fix above is real and measured independently: the baseline's three
trials are unchanged and the YOLO tail improved. What it does change is the
interpretation — the YOLO backend is not "0.6 m biased", it is "detecting the
region rather than the object". The honest summary is that the YOLO row stays out
of the table, and the next step is real-domain supervision: labelled frames from
the simulator, not another synthetic retrain.

## Reproduce

```bash
python3 scripts/make_synthetic_dataset.py --out /root/synthetic_dataset --train 500 --val 100
yolo detect train model=yolo11n.pt data=/root/synthetic_dataset/dataset.yaml imgsz=320 epochs=30
yolo export model=runs/detect/train/weights/best.pt format=onnx imgsz=640 opset=12 simplify=True
cp best.onnx models/yolo11n.onnx
python3 scripts/verify_model.py --model models/yolo11n.onnx --imgsz 640
DETECTOR_CONFIG=/tmp/detector-yolo.yaml bash scripts/run_trials.sh --count 3
```
