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

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.engine.results import Boxes, Keypoints, Results

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.detectors.types import Detector
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.detection2d.yolo.pose")


# Type alias for YOLO pose results
YoloPoseResults = List[Results]

"""
YOLO Pose Detection Results Structure:

Each Results object in the list contains:

1. boxes (Boxes object):
   - boxes.xyxy: torch.Tensor [N, 4] - bounding boxes in [x1, y1, x2, y2] format
   - boxes.xywh: torch.Tensor [N, 4] - boxes in [x_center, y_center, width, height] format
   - boxes.conf: torch.Tensor [N] - confidence scores (0-1)
   - boxes.cls: torch.Tensor [N] - class IDs (0 for person)
   - boxes.xyxyn: torch.Tensor [N, 4] - normalized xyxy coordinates (0-1)
   - boxes.xywhn: torch.Tensor [N, 4] - normalized xywh coordinates (0-1)

2. keypoints (Keypoints object):
   - keypoints.xy: torch.Tensor [N, 17, 2] - absolute x,y coordinates for 17 keypoints
   - keypoints.conf: torch.Tensor [N, 17] - confidence/visibility scores for each keypoint
   - keypoints.xyn: torch.Tensor [N, 17, 2] - normalized coordinates (0-1)
   
   Keypoint order (COCO format):
   0: nose, 1: left_eye, 2: right_eye, 3: left_ear, 4: right_ear,
   5: left_shoulder, 6: right_shoulder, 7: left_elbow, 8: right_elbow,
   9: left_wrist, 10: right_wrist, 11: left_hip, 12: right_hip,
   13: left_knee, 14: right_knee, 15: left_ankle, 16: right_ankle

3. Other attributes:
   - names: Dict[int, str] - class names mapping {0: 'person'}
   - orig_shape: Tuple[int, int] - original image (height, width)
   - speed: Dict[str, float] - timing info {'preprocess': ms, 'inference': ms, 'postprocess': ms}
   - path: str - image path
   - orig_img: np.ndarray - original image array

Note: All tensor data is on GPU by default. Use .cpu() to move to CPU.
"""
from dimos.perception.detection2d.type.detection2d import Detection2DBBox, Bbox


@dataclass
class Person(Detection2DBBox):
    """Represents a detected person with pose keypoints."""

    # Pose keypoints - additional fields beyond Detection2DBBox
    keypoints: np.ndarray  # [17, 2] - x,y coordinates
    keypoint_scores: np.ndarray  # [17] - confidence scores

    # Optional normalized coordinates
    bbox_normalized: Optional[np.ndarray] = None  # [x1, y1, x2, y2] in 0-1 range
    keypoints_normalized: Optional[np.ndarray] = None  # [17, 2] in 0-1 range

    # Image dimensions for context
    image_width: Optional[int] = None
    image_height: Optional[int] = None

    # Keypoint names (class attribute)
    KEYPOINT_NAMES = [
        "nose",
        "left_eye",
        "right_eye",
        "left_ear",
        "right_ear",
        "left_shoulder",
        "right_shoulder",
        "left_elbow",
        "right_elbow",
        "left_wrist",
        "right_wrist",
        "left_hip",
        "right_hip",
        "left_knee",
        "right_knee",
        "left_ankle",
        "right_ankle",
    ]

    @classmethod
    def from_yolo(cls, result: Results, person_idx: int, image: Image) -> "Person":
        """Create Person instance from YOLO results.

        Args:
            result: Single Results object from YOLO
            person_idx: Index of the person in the detection results
            image: Original image for the detection
        """
        # Extract bounding box as tuple for Detection2DBBox
        bbox_array = result.boxes.xyxy[person_idx].cpu().numpy()
        bbox: Bbox = (
            float(bbox_array[0]),
            float(bbox_array[1]),
            float(bbox_array[2]),
            float(bbox_array[3]),
        )

        bbox_norm = (
            result.boxes.xyxyn[person_idx].cpu().numpy() if hasattr(result.boxes, "xyxyn") else None
        )
        confidence = float(result.boxes.conf[person_idx].cpu())
        class_id = int(result.boxes.cls[person_idx].cpu())

        # Extract keypoints
        keypoints = result.keypoints.xy[person_idx].cpu().numpy()
        keypoint_scores = result.keypoints.conf[person_idx].cpu().numpy()
        keypoints_norm = (
            result.keypoints.xyn[person_idx].cpu().numpy()
            if hasattr(result.keypoints, "xyn")
            else None
        )

        # Get image dimensions
        height, width = result.orig_shape

        return cls(
            # Detection2DBBox fields
            bbox=bbox,
            track_id=person_idx,  # Use person index as track_id for now
            class_id=class_id,
            confidence=confidence,
            name="person",
            ts=image.ts,
            image=image,
            # Person-specific fields
            keypoints=keypoints,
            keypoint_scores=keypoint_scores,
            bbox_normalized=bbox_norm,
            keypoints_normalized=keypoints_norm,
            image_width=width,
            image_height=height,
        )

    def get_keypoint(self, name: str) -> Tuple[np.ndarray, float]:
        """Get specific keypoint by name.

        Returns:
            Tuple of (xy_coordinates, confidence_score)
        """
        if name not in self.KEYPOINT_NAMES:
            raise ValueError(f"Invalid keypoint name: {name}. Must be one of {self.KEYPOINT_NAMES}")

        idx = self.KEYPOINT_NAMES.index(name)
        return self.keypoints[idx], self.keypoint_scores[idx]

    def get_visible_keypoints(self, threshold: float = 0.5) -> List[Tuple[str, np.ndarray, float]]:
        """Get all keypoints above confidence threshold.

        Returns:
            List of tuples: (keypoint_name, xy_coordinates, confidence)
        """
        visible = []
        for i, (name, score) in enumerate(zip(self.KEYPOINT_NAMES, self.keypoint_scores)):
            if score > threshold:
                visible.append((name, self.keypoints[i], score))
        return visible

    @property
    def width(self) -> float:
        """Get width of bounding box."""
        x1, _, x2, _ = self.bbox
        return x2 - x1

    @property
    def height(self) -> float:
        """Get height of bounding box."""
        _, y1, _, y2 = self.bbox
        return y2 - y1

    @property
    def center(self) -> Tuple[float, float]:
        """Get center point of bounding box."""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2, (y1 + y2) / 2)


class YoloPoseDetector(Detector):
    def __init__(self, model_path="models_yolo", model_name="yolo11n-pose.pt"):
        self.model = YOLO(get_data(model_path) / model_name, task="pose")

    def process_image(self, image: Image) -> YoloPoseResults:
        """Process image and return YOLO pose detection results.

        Returns:
            List of Results objects, typically one per image.
            Each Results object contains:
            - boxes: Boxes with xyxy, xywh, conf, cls tensors
            - keypoints: Keypoints with xy, conf, xyn tensors
            - names: {0: 'person'} class mapping
            - orig_shape: original image dimensions
            - speed: inference timing
        """
        return self.model(source=image.to_opencv())

    def detect_people(self, image: Image) -> List[Person]:
        """Process image and return list of Person objects.

        Returns:
            List of Person objects with pose keypoints
        """
        results = self.process_image(image)

        people = []
        for result in results:
            if result.keypoints is None or result.boxes is None:
                continue

            # Create Person object for each detection
            num_detections = len(result.boxes.xyxy)
            for i in range(num_detections):
                person = Person.from_yolo(result, i, image)
                people.append(person)

        return people


def main():
    image = Image.from_file(get_data("cafe.jpg"))
    detector = YoloPoseDetector()

    # Get Person objects
    people = detector.detect_people(image)

    print(f"Detected {len(people)} people")
    for i, person in enumerate(people):
        print(f"\nPerson {i}:")
        print(f"  Confidence: {person.confidence:.3f}")
        print(f"  Bounding box: {person.bbox}")
        cx, cy = person.center
        print(f"  Center: ({cx:.1f}, {cy:.1f})")
        print(f"  Size: {person.width:.1f} x {person.height:.1f}")

        # Get specific keypoints
        nose_xy, nose_conf = person.get_keypoint("nose")
        print(f"  Nose: {nose_xy} (conf: {nose_conf:.3f})")

        # Get all visible keypoints
        visible = person.get_visible_keypoints(threshold=0.7)
        print(f"  Visible keypoints (>0.7): {len(visible)}")
        for name, xy, conf in visible[:3]:  # Show first 3
            print(f"    {name}: {xy} (conf: {conf:.3f})")


if __name__ == "__main__":
    main()
