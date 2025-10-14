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

import onnxruntime
from ultralytics import YOLO

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.data import get_data
from dimos.utils.gpu_utils import is_cuda_available
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.detection.yolo.person")


class YoloPersonDetector(Detector):
    def __init__(self, model_path="models_yolo", model_name="yolo11s-pose.pt", device: str = None):
        """Initialize the YOLO person detector.

        Args:
            model_path (str): Path to the YOLO model weights in tests/data LFS directory
            model_name (str): Name of the YOLO model weights file
            device (str): Device to run inference on ('cuda' or 'cpu')
        """
        self.model = YOLO(
            get_data(model_path) / model_name,
            task="track",
        )
        self.tracker = get_data(model_path) / "botsort.yaml"

        if device:
            self.device = device
            return
        else:
            if is_cuda_available():
                if hasattr(onnxruntime, "preload_dlls"):  # Handles CUDA 11 / onnxruntime-gpu<=1.18
                    onnxruntime.preload_dlls(cuda=True, cudnn=True)
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
