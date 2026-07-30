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

from pathlib import Path

from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.utils.model_artifacts import YOLO11_ARTIFACTS


def test_yolo_2d_detector_resolves_pinned_default(mocker, tmp_path: Path) -> None:
    cached_model = tmp_path / "yolo11n.pt"
    resolver = mocker.patch(
        "dimos.perception.detection.detectors.yolo.resolve_model_family_artifact",
        return_value=cached_model,
    )
    yolo = mocker.patch("dimos.perception.detection.detectors.yolo.YOLO")

    Yolo2DDetector(device="cpu")

    resolver.assert_called_once_with(YOLO11_ARTIFACTS, "yolo11n.pt", None)
    yolo.assert_called_once_with(cached_model, task="detect")


def test_yolo_2d_detector_accepts_custom_local_model(mocker, tmp_path: Path) -> None:
    custom_model = tmp_path / "custom.pt"
    resolver = mocker.patch(
        "dimos.perception.detection.detectors.yolo.resolve_model_family_artifact",
        return_value=custom_model,
    )
    mocker.patch("dimos.perception.detection.detectors.yolo.YOLO")

    Yolo2DDetector(model_path=tmp_path, model_name="custom.pt", device="cpu")

    resolver.assert_called_once_with(YOLO11_ARTIFACTS, "custom.pt", tmp_path)


def test_yolo_person_detector_uses_pinned_pose_model_and_local_tracker(
    mocker,
    tmp_path: Path,
) -> None:
    cached_model = tmp_path / "yolo11n-pose.pt"
    resolver = mocker.patch(
        "dimos.perception.detection.detectors.person.yolo.resolve_model_family_artifact",
        return_value=cached_model,
    )
    yolo = mocker.patch("dimos.perception.detection.detectors.person.yolo.YOLO")

    detector = YoloPersonDetector(device="cpu")

    resolver.assert_called_once_with(YOLO11_ARTIFACTS, "yolo11n-pose.pt", None)
    yolo.assert_called_once_with(cached_model, task="track")
    assert detector.tracker.name == "botsort.yaml"
    assert detector.tracker.is_file()
