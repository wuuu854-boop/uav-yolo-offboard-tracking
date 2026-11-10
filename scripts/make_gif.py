#!/usr/bin/env python3
"""Assemble saved frames into the demo GIF.

python3 scripts/make_gif.py assets/frames assets/01-demo.gif --width 640 --duration 120
"""

from __future__ import annotations

import argparse
import pathlib

from PIL import Image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=pathlib.Path)
    parser.add_argument("out", type=pathlib.Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--duration", type=int, default=120, help="ms per frame")
    parser.add_argument("--max-frames", type=int, default=200)
    args = parser.parse_args()

    files = sorted(args.frames.glob("*.jpg"))[: args.max_frames]
    if not files:
        print(f"no frames in {args.frames}")
        return 1

    frames = []
    for path in files:
        image = Image.open(path).convert("RGB")
        ratio = args.width / image.width
        frames.append(image.resize((args.width, int(image.height * ratio)), Image.LANCZOS))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.out,
        save_all=True,
        append_images=frames[1:],
        duration=args.duration,
        loop=0,
        optimize=True,
    )
    print(f"wrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB, {len(frames)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
