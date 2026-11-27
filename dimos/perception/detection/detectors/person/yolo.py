# Copyright 2025-2026 Dimensional Inc.
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

import yaml

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.detectors.ultralytics_utils import (
    cache_path,
    detections_from_results,
    import_ultralytics,
)
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.utils.gpu_utils import is_cuda_available
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Frames a lost track is kept for. The Ultralytics default is 30.
TRACK_BUFFER = 300


def _tracker_config() -> Path:
    """Ultralytics' BoT-SORT settings with a longer track buffer, written to the cache."""
    packaged = Path(import_ultralytics().__file__).parent / "cfg" / "trackers" / "botsort.yaml"
    settings = yaml.safe_load(packaged.read_text()) | {"track_buffer": TRACK_BUFFER}
    path = cache_path("botsort_person.yaml")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(settings))
    return path


class YoloPersonDetector(Detector):
    def __init__(
        self,
        model_name: str = "yolo11n-pose.pt",
        device: str | None = None,
    ) -> None:
        self.model = import_ultralytics().YOLO(cache_path(model_name), task="track")

        self.tracker = _tracker_config()

        if device:
            self.device = device
            return

        if is_cuda_available():  # type: ignore[no-untyped-call]
            self.device = "cuda"
            logger.info("Using CUDA for YOLO person detector")
        else:
            self.device = "cpu"
            logger.info("Using CPU for YOLO person detector")

    def process_image(self, image: Image) -> ImageDetections2D:
        """Process image and return detection results.

        Args:
            image: Input image

        Returns:
            ImageDetections2D containing Detection2DPerson objects with pose keypoints
        """
        results = self.model.track(
            source=image.to_opencv(),
            verbose=False,
            conf=0.5,
            tracker=self.tracker,
            persist=True,
            device=self.device,
        )
        return detections_from_results(image, results)

    def stop(self) -> None:
        """
        Clean up resources used by the detector, including tracker threads.
        """
        if hasattr(self.model, "predictor") and self.model.predictor is not None:
            predictor = self.model.predictor
            if hasattr(predictor, "trackers") and predictor.trackers:
                for tracker in predictor.trackers:
                    if hasattr(tracker, "tracker") and hasattr(tracker.tracker, "gmc"):
                        gmc = tracker.tracker.gmc
                        if hasattr(gmc, "executor") and gmc.executor is not None:
                            gmc.executor.shutdown(wait=True)
            self.model.predictor = None
