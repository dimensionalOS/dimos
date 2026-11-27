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

from collections.abc import Callable
from pathlib import Path
import sys
from typing import Any
from zipfile import ZipFile

import numpy as np
import pytest
import torch

from dimos.models.segmentation.yoloe import YoloeBoxSegmenter
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.detectors import ultralytics_utils
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.person import Detection2DPerson


@pytest.mark.parametrize(
    "factory", [Yolo2DDetector, YoloPersonDetector, Yoloe2DDetector, YoloeBoxSegmenter]
)
def test_without_ultralytics_the_error_names_the_extra(
    factory: Callable[[], Any], monkeypatch: pytest.MonkeyPatch
) -> None:
    # None makes `import ultralytics` fail, as if the package were not installed.
    monkeypatch.setitem(sys.modules, "ultralytics", None)

    with pytest.raises(ImportError, match="Install the yolo extra"):
        factory()


@pytest.mark.parametrize(
    ("factory", "constructor", "kwargs"),
    [
        (Yolo2DDetector, "YOLO", {"task": "detect"}),
        (YoloPersonDetector, "YOLO", {"task": "track"}),
        (Yoloe2DDetector, "YOLOE", {}),
    ],
)
def test_custom_model_directory_keeps_its_positional_argument(
    factory: Callable[..., Any],
    constructor: str,
    kwargs: dict[str, str],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mocker: Any,
) -> None:
    weights = tmp_path / "custom.pt"
    weights.touch()
    (tmp_path / "botsort.yaml").touch()
    backend = mocker.MagicMock()
    monkeypatch.setitem(sys.modules, "ultralytics", backend)

    detector = factory(tmp_path, weights.name, "cpu")

    getattr(backend, constructor).assert_called_once_with(weights, **kwargs)
    assert detector.device == "cpu"


@pytest.mark.self_hosted
def test_legacy_person_and_image_converters() -> None:
    # Only the self-hosted jobs install the optional Ultralytics result type.
    from ultralytics.engine.results import Results

    image = Image(data=np.zeros((16, 16, 3), dtype=np.uint8), format=ImageFormat.RGB)
    result = Results(
        image.data,
        path="frame",
        names={0: "person"},
        boxes=torch.tensor([[1.0, 2.0, 10.0, 12.0, 0.75, 0.0]]),
        keypoints=torch.ones(1, 17, 3),
    )
    person = Detection2DPerson.from_yolo(result, 0, image)
    (detection,) = ImageDetections2D.from_ultralytics_result(image, [result]).detections

    assert isinstance(detection, Detection2DPerson)
    assert person.bbox == detection.bbox == (1.0, 2.0, 10.0, 12.0)
    assert person.confidence == detection.confidence == 0.75


@pytest.mark.self_hosted
def test_interrupted_checkpoint_download_is_retried(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: Any
) -> None:
    monkeypatch.setattr(ultralytics_utils, "CACHE_DIR", tmp_path)
    name = "yolo11n.pt"
    cached = tmp_path / "ultralytics" / name

    def interrupted(staged: Path) -> str:
        staged.write_bytes(b"partial")
        raise OSError("disconnected")

    def successful(staged: Path) -> str:
        with ZipFile(staged, "w") as archive:
            archive.writestr("weights", b"complete")
        return str(staged)

    download = mocker.patch(
        "ultralytics.utils.downloads.attempt_download_asset", side_effect=interrupted
    )
    with pytest.raises(OSError, match="disconnected"):
        ultralytics_utils.checkpoint_path(name)
    assert not cached.exists()

    # Prior releases wrote directly to the cache and could leave this partial file.
    cached.write_bytes(b"PK\x03\x04")
    download.side_effect = successful
    with ZipFile(ultralytics_utils.checkpoint_path(name)) as archive:
        assert archive.read("weights") == b"complete"
    assert ultralytics_utils.checkpoint_path(name) == cached
    assert download.call_count == 2
