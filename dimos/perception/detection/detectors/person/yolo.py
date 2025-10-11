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

from ultralytics import YOLO

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.detection.yolo.person")


class YoloPersonDetector(Detector):
    def __init__(self, model_path="models_yolo", model_name="yolo11n-pose.pt"):
        self.model = YOLO(get_data(model_path) / model_name, task="pose")

    def process_image(self, image: Image) -> ImageDetections2D:
        """Process image and return detection results.

        Args:
            image: Input image

        Returns:
            ImageDetections2D containing Detection2DPerson objects with pose keypoints
        """
        results = self.model(source=image.to_opencv())
        return ImageDetections2D.from_ultralytics_result(image, results)
