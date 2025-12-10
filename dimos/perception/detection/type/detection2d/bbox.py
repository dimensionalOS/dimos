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

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from typing import TYPE_CHECKING, Any

from typing_extensions import Self

if TYPE_CHECKING:
    from ultralytics.engine.results import Results

    from dimos.msgs.sensor_msgs import Image

from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    PointsAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2  # type: ignore[import-untyped]
from dimos_lcm.vision_msgs import (  # type: ignore[import-untyped]
    BoundingBox2D,
    Detection2D as ROSDetection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
from rich.console import Console
from rich.text import Text

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.std_msgs import Header
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.types.timestamped import to_ros_stamp, to_timestamp
from dimos.utils.decorators.decorators import simple_mcache

Bbox = tuple[float, float, float, float]
CenteredBbox = tuple[float, float, float, float]


def _hash_to_color(name: str) -> str:
    """Generate a consistent color for a given name using hash."""
    # List of rich colors to choose from
    colors = [
        "cyan",
        "magenta",
        "yellow",
        "blue",
        "green",
        "red",
        "bright_cyan",
        "bright_magenta",
        "bright_yellow",
        "bright_blue",
        "bright_green",
        "bright_red",
        "purple",
        "white",
        "pink",
    ]

    # Hash the name and pick a color
    hash_value = hashlib.md5(name.encode()).digest()[0]
    return colors[hash_value % len(colors)]


@dataclass
class Detection2DBBox(Detection2D):
    bbox: Bbox
    track_id: int
    class_id: int
    confidence: float
    name: str
    ts: float
    image: Image

    def to_repr_dict(self) -> dict[str, Any]:
        """Return a dictionary representation of the detection for display purposes."""
        x1, y1, x2, y2 = self.bbox
        return {
            "name": self.name,
            "class": str(self.class_id),
            "track": str(self.track_id),
            "conf": f"{self.confidence:.2f}",
            "bbox": f"[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]",
        }

    def center_to_3d(
        self,
        pixel: tuple[int, int],
        camera_info: CameraInfo,  # type: ignore[name-defined]
        assumed_depth: float = 1.0,
    ) -> PoseStamped:  # type: ignore[name-defined]
        """Unproject 2D pixel coordinates to 3D position in camera optical frame.

        Args:
            camera_info: Camera calibration information
            assumed_depth: Assumed depth in meters (default 1.0m from camera)

        Returns:
            Vector3 position in camera optical frame coordinates
        """
        # Extract camera intrinsics
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]

        # Unproject pixel to normalized camera coordinates
        x_norm = (pixel[0] - cx) / fx
        y_norm = (pixel[1] - cy) / fy

        # Create 3D point at assumed depth in camera optical frame
        # Camera optical frame: X right, Y down, Z forward
        return Vector3(x_norm * assumed_depth, y_norm * assumed_depth, assumed_depth)  # type: ignore[name-defined]

    # return focused image, only on the bbox
    def cropped_image(self, padding: int = 20) -> Image:
        """Return a cropped version of the image focused on the bounding box.

        Args:
            padding: Pixels to add around the bounding box (default: 20)

        Returns:
            Cropped Image containing only the detection area plus padding
        """
        x1, y1, x2, y2 = map(int, self.bbox)
        return self.image.crop(
            x1 - padding, y1 - padding, x2 - x1 + 2 * padding, y2 - y1 + 2 * padding
        )

    def __str__(self) -> str:
        console = Console(force_terminal=True, legacy_windows=False)
        d = self.to_repr_dict()

        # Build the string representation
        parts = [
            Text(f"{self.__class__.__name__}("),
        ]

        # Add any extra fields (e.g., points for Detection3D)
        extra_keys = [k for k in d.keys() if k not in ["class"]]
        for key in extra_keys:
            if d[key] == "None":
                parts.append(Text(f"{key}={d[key]}", style="dim"))
            else:
                parts.append(Text(f"{key}={d[key]}", style=_hash_to_color(key)))

        parts.append(Text(")"))

        # Render to string
        with console.capture() as capture:
            console.print(*parts, end="")
        return capture.get().strip()

    @property
    def center_bbox(self) -> tuple[float, float]:
        """Get center point of bounding box."""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def bbox_2d_volume(self) -> float:
        x1, y1, x2, y2 = self.bbox
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        return width * height

    @simple_mcache
    def is_valid(self) -> bool:
        """Check if detection bbox is valid.

        Validates that:
        - Bounding box has positive dimensions
        - Bounding box is within image bounds (if image has shape)

        Returns:
            True if bbox is valid, False otherwise
        """
        x1, y1, x2, y2 = self.bbox

        # Check positive dimensions
        if x2 <= x1 or y2 <= y1:
            return False

        # Check if within image bounds (if image has shape)
        if self.image.shape:
            h, w = self.image.shape[:2]
            if not (0 <= x1 <= w and 0 <= y1 <= h and 0 <= x2 <= w and 0 <= y2 <= h):
                return False

        return True

    @classmethod
    def from_ultralytics_result(cls, result: Results, idx: int, image: Image) -> Detection2DBBox:
        """Create Detection2DBBox from ultralytics Results object.

        Args:
            result: Ultralytics Results object containing detection data
            idx: Index of the detection in the results
            image: Source image

        Returns:
            Detection2DBBox instance
        """
        if result.boxes is None:
            raise ValueError("Result has no boxes")

        # Extract bounding box coordinates
        bbox_array = result.boxes.xyxy[idx].cpu().numpy()
        bbox: Bbox = (
            float(bbox_array[0]),
            float(bbox_array[1]),
            float(bbox_array[2]),
            float(bbox_array[3]),
        )

        # Extract confidence
        confidence = float(result.boxes.conf[idx].cpu())

        # Extract class ID and name
        class_id = int(result.boxes.cls[idx].cpu())
        name = (
            result.names.get(class_id, f"class_{class_id}")
            if hasattr(result, "names")
            else f"class_{class_id}"
        )

        # Extract track ID if available
        track_id = -1
        if hasattr(result.boxes, "id") and result.boxes.id is not None:
            track_id = int(result.boxes.id[idx].cpu())

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=image.ts,
            image=image,
        )

    def get_bbox_center(self) -> CenteredBbox:
        x1, y1, x2, y2 = self.bbox
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        width = float(x2 - x1)
        height = float(y2 - y1)
        return (center_x, center_y, width, height)

    def to_ros_bbox(self) -> BoundingBox2D:
        center_x, center_y, width, height = self.get_bbox_center()
        return BoundingBox2D(
            center=Pose2D(
                position=Point2D(x=center_x, y=center_y),
                theta=0.0,
            ),
            size_x=width,
            size_y=height,
        )

    def lcm_encode(self):  # type: ignore[no-untyped-def]
        return self.to_image_annotations().lcm_encode()

    def to_text_annotation(self) -> list[TextAnnotation]:
        x1, y1, _x2, y2 = self.bbox

        font_size = self.image.width / 80

        # Build label text - exclude class_id if it's -1 (VLM detection)
        if self.class_id == -1:
            label_text = f"{self.name}_{self.track_id}"
        else:
            label_text = f"{self.name}_{self.class_id}_{self.track_id}"

        annotations = [
            TextAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=x1, y=y1),
                text=label_text,
                font_size=font_size,
                text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                background_color=Color(r=0, g=0, b=0, a=1),
            ),
        ]

        # Only show confidence if it's not 1.0
        if self.confidence != 1.0:
            annotations.append(
                TextAnnotation(
                    timestamp=to_ros_stamp(self.ts),
                    position=Point2(x=x1, y=y2 + font_size),
                    text=f"confidence: {self.confidence:.3f}",
                    font_size=font_size,
                    text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                    background_color=Color(r=0, g=0, b=0, a=1),
                )
            )

        return annotations

    def to_points_annotation(self) -> list[PointsAnnotation]:
        x1, y1, x2, y2 = self.bbox

        thickness = 1

        # Use consistent color based on object name, brighter for outline
        outline_color = Color.from_string(self.name, alpha=1.0, brightness=1.25)

        return [
            PointsAnnotation(
                timestamp=to_ros_stamp(self.ts),
                outline_color=outline_color,
                fill_color=Color.from_string(self.name, alpha=0.2),
                thickness=thickness,
                points_length=4,
                points=[
                    Point2(x1, y1),
                    Point2(x1, y2),
                    Point2(x2, y2),
                    Point2(x2, y1),
                ],
                type=PointsAnnotation.LINE_LOOP,
            )
        ]

    # this is almost never called directly since this is a single detection
    # and ImageAnnotations message normally contains multiple detections annotations
    # so ImageDetections2D and ImageDetections3D normally implements this for whole image
    def to_image_annotations(self) -> ImageAnnotations:
        points = self.to_points_annotation()
        texts = self.to_text_annotation()

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=points,
            points_length=len(points),
        )

    @classmethod
    def from_ros_detection2d(cls, ros_det: ROSDetection2D, **kwargs) -> Self:  # type: ignore[no-untyped-def]
        """Convert from ROS Detection2D message to Detection2D object."""
        # Extract bbox from ROS format
        center_x = ros_det.bbox.center.position.x
        center_y = ros_det.bbox.center.position.y
        width = ros_det.bbox.size_x
        height = ros_det.bbox.size_y

        # Convert centered bbox to corner format
        x1 = center_x - width / 2.0
        y1 = center_y - height / 2.0
        x2 = center_x + width / 2.0
        y2 = center_y + height / 2.0
        bbox = (x1, y1, x2, y2)

        # Extract hypothesis info
        class_id = 0
        confidence = 0.0
        if ros_det.results:
            hypothesis = ros_det.results[0].hypothesis
            class_id = hypothesis.class_id
            confidence = hypothesis.score

        # Extract track_id
        track_id = int(ros_det.id) if ros_det.id.isdigit() else 0

        # Extract timestamp
        ts = to_timestamp(ros_det.header.stamp)

        name = kwargs.pop("name", f"class_{class_id}")

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=ts,
            **kwargs,
        )

    def to_ros_detection2d(self) -> ROSDetection2D:
        return ROSDetection2D(
            header=Header(self.ts, "camera_link"),
            bbox=self.to_ros_bbox(),
            results=[
                ObjectHypothesisWithPose(
                    ObjectHypothesis(
                        class_id=self.class_id,
                        score=self.confidence,
                    )
                )
            ],
            id=str(self.track_id),
        )
