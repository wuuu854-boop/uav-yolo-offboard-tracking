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

The false positives are gone: three SITL trials now **land** instead of
wandering (`results/v03_yolo_trials.csv`).

| Trial | Landing error | End-to-end p50 |
|---|---|---|
| 1 | 2.208 m | 166 ms |
| 2 | 2.362 m | 171 ms |
| 3 | 2.278 m | 160 ms |

All three miss the 1.5 m tolerance in the same direction by about the same
amount, which is the signature of a systematic offset rather than noise: the
YOLO estimate is consistently ~2.2 m from where the baseline's estimate puts the
same cube. That is a solvable bug, not a modelling gap — the next step is to
record one hover at a known pose, run both backends on the same frames, and
compare their projected ground points.

**The measured closed loop therefore remains the colour baseline.** The YOLO row
stays out of the results table until it can match it, and `results/v03_yolo_trials.csv`
is committed so the claim can be checked rather than trusted.

## Reproduce

```bash
python3 scripts/make_synthetic_dataset.py --out /root/synthetic_dataset --train 500 --val 100
yolo detect train model=yolo11n.pt data=/root/synthetic_dataset/dataset.yaml imgsz=320 epochs=30
yolo export model=runs/detect/train/weights/best.pt format=onnx imgsz=640 opset=12 simplify=True
cp best.onnx models/yolo11n.onnx
python3 scripts/verify_model.py --model models/yolo11n.onnx --imgsz 640
DETECTOR_CONFIG=/tmp/detector-yolo.yaml bash scripts/run_trials.sh --count 3
```
