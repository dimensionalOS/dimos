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

import pytest

from dimos.models.segmentation.yoloe import YoloeBoxSegmenter
from dimos.perception.detection.detectors import ultralytics_utils
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector


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


@pytest.mark.parametrize("in_directory", [True, False])
def test_missing_custom_checkpoint_is_not_downloaded(in_directory: bool, tmp_path: Path) -> None:
    args = ("custom.pt", tmp_path) if in_directory else (str(tmp_path / "custom.pt"),)

    with pytest.raises(FileNotFoundError, match="custom.pt"):
        ultralytics_utils.checkpoint_path(*args)

    assert list(tmp_path.iterdir()) == []


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
