"""Contract tests for the custom message definitions.

The detector and the control node agree on a message shape; if someone edits a
.msg file without updating both sides, this test fails before the simulator does.
Parsing the .msg text keeps the test runnable without ROS installed.
"""

from __future__ import annotations

from pathlib import Path

MSG_DIR = Path(__file__).resolve().parents[1] / "src" / "uav_interfaces" / "msg"


def _fields(message_name: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in (MSG_DIR / message_name).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        field_type, field_name = line.split()
        fields[field_name] = field_type
    return fields


def test_target_message_keeps_the_fields_both_nodes_use():
    fields = _fields("Target.msg")
    assert fields["header"] == "std_msgs/Header"
    for name in ("confidence", "bbox_cx", "bbox_cy", "bbox_w", "bbox_h"):
        assert fields[name] == "float32", f"{name} must stay float32"
    assert fields["image_width"] == "uint32"
    assert fields["image_height"] == "uint32"
    # 0 means "not tracked"; ids start at 1 so that default is unambiguous.
    assert fields["track_id"] == "int32"


def test_detection_array_wraps_targets():
    fields = _fields("TargetArray.msg")
    assert fields["targets"] == "Target[]"
    assert fields["header"] == "std_msgs/Header"
