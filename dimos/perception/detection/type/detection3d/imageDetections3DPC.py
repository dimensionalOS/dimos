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

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos_lcm.foxglove_msgs import SceneUpdate
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined]
        Point as ROSPoint,
        Pose as ROSPose,
        Quaternion as ROSQuaternion,
        Vector3 as ROSVector3,
    )
    from std_msgs.msg import Header as ROSHeader  # type: ignore[attr-defined]
    from vision_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        BoundingBox3D as ROSBoundingBox3D,
        Detection3D as ROSDetection3D,
        Detection3DArray as ROSDetection3DArray,
        ObjectHypothesisWithPose as ROSObjectHypothesisWithPose,
    )
except ImportError:
    ROSPoint = None  # type: ignore[assignment, misc]
    ROSPose = None  # type: ignore[assignment, misc]
    ROSQuaternion = None  # type: ignore[assignment, misc]
    ROSVector3 = None  # type: ignore[assignment, misc]
    ROSHeader = None  # type: ignore[assignment, misc]
    ROSBoundingBox3D = None  # type: ignore[assignment, misc]
    ROSDetection3D = None  # type: ignore[assignment, misc]
    ROSDetection3DArray = None  # type: ignore[assignment, misc]
    ROSObjectHypothesisWithPose = None  # type: ignore[assignment, misc]

from dimos.msgs.sensor_msgs import PointCloud2
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC


class ImageDetections3DPC(ImageDetections["Detection3DPC"]):
    """Specialized class for 3D detections in an image."""

    def get_aggregated_objects_pointcloud(self) -> PointCloud2 | None:
        """Aggregate all detection pointclouds into a single colored pointcloud.

        Each detection's points are colored based on a consistent color derived
        from its track_id, similar to the 2D overlay visualization.

        Returns:
            Combined PointCloud2 with all detection points colored by object,
            or None if no detections.
        """
        if not self.detections:
            return None

        all_points = []
        all_colors = []

        for i, det in enumerate(self.detections):
            points, original_colors = det.pointcloud.as_numpy()
            if len(points) == 0:
                continue

            track_id = det.track_id if det.track_id >= 0 else i
            np.random.seed(abs(track_id))
            track_color = np.random.randint(50, 255, 3) / 255.0

            if original_colors is not None:
                blended_colors = 0.6 * original_colors + 0.4 * track_color
                blended_colors = np.clip(blended_colors, 0.0, 1.0)
            else:
                blended_colors = np.tile(track_color, (len(points), 1))

            all_points.append(points)
            all_colors.append(blended_colors)

        if not all_points:
            return None

        # Combine all points and colors
        combined_points = np.vstack(all_points)
        combined_colors = np.vstack(all_colors)

        # Get frame_id and timestamp from first detection
        frame_id = self.detections[0].frame_id
        ts = self.detections[0].ts

        # Create combined pointcloud
        combined_pc = PointCloud2.from_numpy(
            combined_points,
            frame_id=frame_id,
            timestamp=ts,
        )

        combined_pc.pointcloud.colors = o3d.utility.Vector3dVector(combined_colors)

        return combined_pc

    def to_ros_detection3d_array(self) -> ROSDetection3DArray:
        """Convert ImageDetections3DPC to ROS Detection3DArray message.

        Returns:
            Detection3DArray ROS message containing all 3D detections
        """
        detection_array = ROSDetection3DArray()

        # Set header from image or first detection
        detection_array.header = ROSHeader()
        if self.detections:
            detection_array.header.frame_id = self.detections[0].frame_id or ""
            ts = self.detections[0].ts
            detection_array.header.stamp.sec = int(ts)
            detection_array.header.stamp.nanosec = int((ts % 1) * 1_000_000_000)
        elif self.image:
            detection_array.header.frame_id = self.image.frame_id or ""
            if self.image.ts:
                detection_array.header.stamp.sec = int(self.image.ts)
                detection_array.header.stamp.nanosec = int((self.image.ts % 1) * 1_000_000_000)

        # Convert each detection
        for det in self.detections:
            ros_detection = ROSDetection3D()

            # Set header
            ros_detection.header = ROSHeader()
            ros_detection.header.frame_id = det.frame_id or ""
            ros_detection.header.stamp.sec = int(det.ts)
            ros_detection.header.stamp.nanosec = int((det.ts % 1) * 1_000_000_000)

            # Set bounding box
            ros_detection.bbox = ROSBoundingBox3D()
            ros_detection.bbox.center = ROSPose()
            ros_detection.bbox.center.position = ROSPoint()

            # Get center from pointcloud
            center = det.center
            ros_detection.bbox.center.position.x = float(center.x)
            ros_detection.bbox.center.position.y = float(center.y)
            ros_detection.bbox.center.position.z = float(center.z)

            # Identity orientation for axis-aligned bbox
            ros_detection.bbox.center.orientation = ROSQuaternion()
            ros_detection.bbox.center.orientation.x = 0.0
            ros_detection.bbox.center.orientation.y = 0.0
            ros_detection.bbox.center.orientation.z = 0.0
            ros_detection.bbox.center.orientation.w = 1.0

            # Set size from bounding box dimensions
            dims = det.get_bounding_box_dimensions()
            ros_detection.bbox.size = ROSVector3()
            ros_detection.bbox.size.x = float(dims[0])
            ros_detection.bbox.size.y = float(dims[1])
            ros_detection.bbox.size.z = float(dims[2])

            # Add class hypothesis
            if det.name:
                hypothesis = ROSObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = det.name
                hypothesis.hypothesis.score = float(det.confidence)
                ros_detection.results.append(hypothesis)

            # Set tracking ID
            if det.track_id >= 0:
                ros_detection.id = str(det.track_id)

            detection_array.detections.append(ros_detection)

        return detection_array

    def to_foxglove_scene_update(self) -> SceneUpdate:
        """Convert all detections to a Foxglove SceneUpdate message.

        Returns:
            SceneUpdate containing SceneEntity objects for all detections
        """

        # Create SceneUpdate message with all detections
        scene_update = SceneUpdate()
        scene_update.deletions_length = 0
        scene_update.deletions = []
        scene_update.entities = []

        # Process each detection
        for i, detection in enumerate(self.detections):
            entity = detection.to_foxglove_scene_entity(entity_id=f"detection_{detection.name}_{i}")
            scene_update.entities.append(entity)

        scene_update.entities_length = len(scene_update.entities)
        return scene_update
