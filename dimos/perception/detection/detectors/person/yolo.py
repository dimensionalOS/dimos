# Copyright 2025 Dimensional Inc.
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

from ultralytics import YOLO  # type: ignore[attr-defined]

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.data import get_data
from dimos.utils.gpu_utils import is_cuda_available
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class YoloPersonDetector(Detector):
    def __init__(
        self,
        model_path: str = "models_yolo",
        model_name: str = "yolo11n-pose.pt",
        device: str | None = None,
    ) -> None:
        self.model = YOLO(get_data(model_path) / model_name, task="track")

        self.tracker = get_data(model_path) / "botsort.yaml"

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
        return ImageDetections2D.from_ultralytics_result(image, results)

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
