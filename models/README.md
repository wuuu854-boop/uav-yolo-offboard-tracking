# models/

Model weights live here locally but never in git: `.gitignore` keeps everything
in this directory out except this README and `md5sums.txt`.

## Expected file

| File | Size | Source |
|---|---|---|
| `yolo11n.onnx` | ~5–12 MB | `python3 scripts/export_onnx.py`, or the Release asset |

## Checksums

After downloading or exporting, record the checksum so a corrupted download is
detected instead of silently producing empty detections:

```bash
cd models && md5sum *.onnx > md5sums.txt
```

`scripts/download_data.sh` verifies this file automatically when present.
