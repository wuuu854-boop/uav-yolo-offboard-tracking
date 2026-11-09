"""The detector wrapper: letterbox -> blob -> decode, with a fake session."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from uav_tracking.perception.yolo import YoloOnnxDetector


class FakeSession:
    """Stands in for onnxruntime.InferenceSession."""

    def __init__(self, output: np.ndarray) -> None:
        self.output = output
        self.calls = 0
        self.last_blob: np.ndarray | None = None

    def get_inputs(self) -> list[SimpleNamespace]:
        return [SimpleNamespace(name="images")]

    def run(self, _outputs, feed):  # noqa: ANN001 - mirrors the ORT signature
        self.calls += 1
        self.last_blob = feed["images"]
        return [self.output]


def _one_box_output(centre_u: float, centre_v: float, confidence: float = 0.9) -> np.ndarray:
    # (1, 4 + 3 classes, 1 box)
    return np.array(
        [[[centre_u], [centre_v], [64.0], [32.0], [confidence], [0.0], [0.0]]],
        dtype=np.float32,
    )


def test_a_full_frame_is_letterboxed_and_decoded_back():
    session = FakeSession(_one_box_output(320.0, 320.0))
    detector = YoloOnnxDetector(session=session, input_size=640)

    # 1280x960 -> scale 0.5 gives 640x480, so the canvas is padded by 80 px at
    # the top. A box centred at (320, 320) in the canvas therefore sits at
    # ((320 - 80) / 0.5) = 480 in the original frame.
    frame = np.zeros((960, 1280, 3), dtype=np.uint8)
    detections = detector.infer(frame)

    assert session.last_blob is not None
    assert session.last_blob.shape == (1, 3, 640, 640)
    assert session.last_blob.max() <= 1.0  # normalised

    assert len(detections) == 1
    assert detections[0]["bbox_cx"] == pytest.approx(640.0)
    assert detections[0]["bbox_cy"] == pytest.approx(480.0)
    assert detections[0]["bbox_w"] == pytest.approx(128.0)


def test_the_session_is_only_called_once_per_frame():
    session = FakeSession(_one_box_output(320.0, 320.0))
    detector = YoloOnnxDetector(session=session)

    detector.infer(np.zeros((480, 640, 3), dtype=np.uint8))
    detector.infer(np.zeros((480, 640, 3), dtype=np.uint8))

    assert session.calls == 2


def test_a_missing_model_is_reported_before_importing_onnxruntime(tmp_path):
    with pytest.raises(FileNotFoundError, match="export_onnx"):
        YoloOnnxDetector(model_path=tmp_path / "absent.onnx")


def test_neither_model_nor_session_is_a_value_error():
    with pytest.raises(ValueError):
        YoloOnnxDetector()
