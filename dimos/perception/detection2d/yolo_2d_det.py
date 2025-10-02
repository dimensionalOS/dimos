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

import os

import cv2
import onnxruntime
from ultralytics import YOLO

from dimos.perception.detection2d.utils import (
    extract_detection_results,
    filter_detections,
    plot_results,
)
from dimos.utils.data import get_data
from dimos.utils.gpu_utils import is_cuda_available
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.detection2d.yolo_2d_det")


class Yolo2DDetector:
    def __init__(self, model_path="models_yolo", model_name="yolo11n.onnx", device="cpu"):
        """
        Initialize the YOLO detector.

        Args:
            model_path (str): Path to the YOLO model weights in tests/data LFS directory
            model_name (str): Name of the YOLO model weights file
            device (str): Device to run inference on ('cuda' or 'cpu')
        """
        self.device = device
        self.model = YOLO(get_data(model_path) / model_name)

        module_dir = os.path.dirname(__file__)
        self.tracker_config = os.path.join(module_dir, "config", "custom_tracker.yaml")
        if is_cuda_available():
            if hasattr(onnxruntime, "preload_dlls"):  # Handles CUDA 11 / onnxruntime-gpu<=1.18
                onnxruntime.preload_dlls(cuda=True, cudnn=True)
            self.device = "cuda"
            logger.info("Using CUDA for YOLO 2d detector")
        else:
            self.device = "cpu"
            logger.info("Using CPU for YOLO 2d detector")

    def process_image(self, image):
        """
        Process an image and return detection results.

        Args:
            image: Input image in BGR format (OpenCV)

        Returns:
            tuple: (bboxes, track_ids, class_ids, confidences, names)
                - bboxes: list of [x1, y1, x2, y2] coordinates
                - track_ids: list of tracking IDs (or -1 if no tracking)
                - class_ids: list of class indices
                - confidences: list of detection confidences
                - names: list of class names
        """
        results = self.model.track(
            source=image,
            device=self.device,
            conf=0.5,
            iou=0.6,
            persist=True,
            verbose=False,
            tracker=self.tracker_config,
        )

        if len(results) > 0:
            # Extract detection results
            bboxes, track_ids, class_ids, confidences, names = extract_detection_results(results[0])
            return bboxes, track_ids, class_ids, confidences, names

        return [], [], [], [], []

    def visualize_results(self, image, bboxes, track_ids, class_ids, confidences, names):
        """
        Generate visualization of detection results.

        Args:
            image: Original input image
            bboxes: List of bounding boxes
            track_ids: List of tracking IDs
            class_ids: List of class indices
            confidences: List of detection confidences
            names: List of class names

        Returns:
            Image with visualized detections
        """
        return plot_results(image, bboxes, track_ids, class_ids, confidences, names)

    def stop(self):
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


def main():
    """Example usage of the Yolo2DDetector class."""
    # Initialize video capture
    cap = cv2.VideoCapture(0)

    # Initialize detector
    detector = Yolo2DDetector()

    enable_person_filter = True

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Process frame
            bboxes, track_ids, class_ids, confidences, names = detector.process_image(frame)

            # Apply person filtering if enabled
            if enable_person_filter and len(bboxes) > 0:
                # Person is class_id 0 in COCO dataset
                bboxes, track_ids, class_ids, confidences, names = filter_detections(
                    bboxes,
                    track_ids,
                    class_ids,
                    confidences,
                    names,
                    class_filter=[0],  # 0 is the class_id for person
                    name_filter=["person"],
                )

            # Visualize results
            if len(bboxes) > 0:
                frame = detector.visualize_results(
                    frame, bboxes, track_ids, class_ids, confidences, names
                )

            # Display results
            cv2.imshow("YOLO Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
