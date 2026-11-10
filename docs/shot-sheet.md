# What a shot sheet is, and what this one requires

The first item on the utility industry's list of costly failures is "no
standardised capture methodology": every pilot invents the shot list, and the
deliverable depends on who flew. A shot sheet is the written standard — for each
structure type, which views must be captured, at what resolution, and framed how.
This repository's version is smaller than a real one, but it is the same shape,
and it is checkable.

## The spec, in this repository

`config/shot_sheet.yaml`:

```yaml
structure_type: cube
views:
  - name: nadir
    camera: down
    max_gsd_cm: 1.3      # ground sample distance, centimetres per pixel
    min_frames: 3
    margin_px: 24        # the target must be this far inside the frame
```

Four fields, each with a reason:

| Field | Why it exists |
|---|---|
| `camera` | which mounting the view needs; this platform has one, nadir |
| `max_gsd_cm` | the resolution threshold below which the asset is not assessable |
| `min_frames` | one lucky frame is not evidence; a view is captured when it repeats |
| `margin_px` | a target clipped by the frame edge is not a usable view, even if detected |

## How the check works

`shot_sheet_check.py` replays the detector over a capture and, for every frame in
which the asset is visible:

1. computes the ground sample distance from the recorded altitude and the camera
   focal length — `gsd = altitude / fx`, no magic numbers;
2. checks that the detection's box lies at least `margin_px` inside the frame;
3. attributes the frame to the nearest declared asset;
4. counts the frames that satisfy each view.

A view passes when at least `min_frames` frames satisfy it. An asset passes when
all of its required views pass. The report lists the frames that were rejected
and why — too coarse, clipped, or no detection — because "you have 3 of 4 views"
is only actionable with the reason attached.

## What this does not do

- It does not check that the *right* structure was photographed. That is the
  association problem, and it is measured separately in `qa_report.py`.
- It does not cover oblique views, which need a gimbal this platform does not
  have. A real shot sheet for a lattice tower has a dozen of them.
- It does not replace a human reviewer. It replaces the part of the review that
  is arithmetic.
