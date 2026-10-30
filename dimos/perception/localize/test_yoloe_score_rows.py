# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


def _patch_yoloe_load(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "dimos.perception.detection.detectors.yoloe.get_data",
        lambda _name: Path("/tmp/models_yoloe"),
    )
    fake_ultralytics = MagicMock()
    fake_ultralytics.YOLOE = MagicMock(return_value=MagicMock())
    monkeypatch.setitem(__import__("sys").modules, "ultralytics", fake_ultralytics)


@pytest.fixture
def prompt_yoloe(monkeypatch: pytest.MonkeyPatch) -> Yoloe2DDetector:
    _patch_yoloe_load(monkeypatch)
    return Yoloe2DDetector(prompt_mode=YoloePromptMode.PROMPT, conf=0.4, max_area_ratio=None)


def test_query_score_rows_requires_prompt_mode(monkeypatch: pytest.MonkeyPatch) -> None:
    _patch_yoloe_load(monkeypatch)
    detector = Yoloe2DDetector(prompt_mode=YoloePromptMode.LRPC, conf=0.5)
    image = Image(data=np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB)
    with pytest.raises(ValueError, match="PROMPT"):
        detector.query_score_rows(image, ["chair"], threshold=0.1)


def test_query_score_rows_sparse_matrix_for_winner_class(
    prompt_yoloe: Yoloe2DDetector, monkeypatch: pytest.MonkeyPatch
) -> None:
    image = Image(data=np.zeros((32, 32, 3), dtype=np.uint8), format=ImageFormat.RGB)
    det = Detection2DBBox(
        bbox=(1.0, 2.0, 10.0, 12.0),
        track_id=-1,
        class_id=0,
        confidence=0.77,
        name="houseplant",
        ts=0.0,
        image=image,
    )
    set_prompts = MagicMock()
    monkeypatch.setattr(prompt_yoloe, "set_prompts", set_prompts)
    monkeypatch.setattr(
        prompt_yoloe,
        "predict_image",
        lambda _image: ImageDetections2D(image=image, detections=[det]),
    )

    boxes, rows = prompt_yoloe.query_score_rows(
        image, ["person", "houseplant", "desk"], threshold=0.2
    )

    assert boxes.shape == (1, 4)
    assert rows.shape == (1, 3)
    np.testing.assert_allclose(boxes[0], [1.0, 2.0, 10.0, 12.0])
    np.testing.assert_allclose(rows[0], [0.0, 0.77, 0.0])
    set_prompts.assert_called_once_with(text=["person", "houseplant", "desk"])


def test_dandetect_registry_includes_yoloe() -> None:
    from dimos.perception.localize.dandetect import DETECTORS

    assert "yoloe" in DETECTORS
