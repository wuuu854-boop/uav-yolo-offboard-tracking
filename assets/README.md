# assets/

Binary demo material referenced by the README. Keep this directory small: every
file here is downloaded on every clone.

## Naming

`01-demo.gif`, `02-detection.png`, `03-trajectory.png` — two-digit prefix so the
order is stable, lowercase, hyphens between words.

## Limits

| Type | Spec | Why |
|---|---|---|
| GIF | 1280×720, 15 fps, ≤20 s, **≤8 MB** | GitHub renders inline; bigger files make the first screen empty for seconds |
| PNG | ≤1600 px wide, ≤1 MB | screenshots of a plot or the detection overlay |
| MP4 | not committed | link to the Release or a video host instead |

Compress a screen recording:

```bash
ffmpeg -i demo.mp4 -vf "fps=15,scale=1280:-1" -loop 0 assets/01-demo.gif
```

## What the demo GIF must show

1. Gazebo window with the UAV moving.
2. The detection overlay with a bounding box.
3. Ideally the terminal printing the landing error at the end.

A GIF that only shows a flying drone proves nothing about the AI part.
