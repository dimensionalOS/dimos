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

from dimos_lcm.vision_msgs.Detection3D import Detection3D as LCMDetection3D

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
        ObjectHypothesis as ROSObjectHypothesis,
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
    ROSObjectHypothesis = None  # type: ignore[assignment, misc]
    ROSObjectHypothesisWithPose = None  # type: ignore[assignment, misc]

from dimos.types.timestamped import to_timestamp


class Detection3D(LCMDetection3D):  # type: ignore[misc]
    msg_name = "vision_msgs.Detection3D"

    # for _get_field_type() to work when decoding in _decode_one()
    __annotations__ = LCMDetection3D.__annotations__

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        """Initialize with fresh mutable objects to avoid shared state."""
        super().__init__()
        # Create fresh instances to avoid shared mutable state from LCM class defaults
        from dimos_lcm.std_msgs import Header
        from dimos_lcm.vision_msgs import BoundingBox3D

        self.header = Header()
        from dimos_lcm.vision_msgs import ObjectHypothesisWithPose

        self.results: list[ObjectHypothesisWithPose] = []
        self.bbox = BoundingBox3D()
        self.id = ""

        # Apply any kwargs
        for key, value in kwargs.items():
            setattr(self, key, value)

    @property
    def ts(self) -> float:
        return to_timestamp(self.header.stamp)

    def to_ros_msg(self) -> ROSDetection3D:
        """Convert to ROS vision_msgs/Detection3D message.

        Returns:
            ROS Detection3D message
        """
        ros_msg = ROSDetection3D()

        # Set header
        ros_msg.header = ROSHeader()
        ros_msg.header.frame_id = self.header.frame_id
        ros_msg.header.stamp.sec = self.header.stamp.sec
        ros_msg.header.stamp.nanosec = self.header.stamp.nsec

        # Set bounding box
        ros_msg.bbox = ROSBoundingBox3D()
        ros_msg.bbox.center = ROSPose()
        ros_msg.bbox.center.position = ROSPoint()
        ros_msg.bbox.center.position.x = float(self.bbox.center.position.x)
        ros_msg.bbox.center.position.y = float(self.bbox.center.position.y)
        ros_msg.bbox.center.position.z = float(self.bbox.center.position.z)
        ros_msg.bbox.center.orientation = ROSQuaternion()
        ros_msg.bbox.center.orientation.x = float(self.bbox.center.orientation.x)
        ros_msg.bbox.center.orientation.y = float(self.bbox.center.orientation.y)
        ros_msg.bbox.center.orientation.z = float(self.bbox.center.orientation.z)
        ros_msg.bbox.center.orientation.w = float(self.bbox.center.orientation.w)
        ros_msg.bbox.size = ROSVector3()
        ros_msg.bbox.size.x = float(self.bbox.size.x)
        ros_msg.bbox.size.y = float(self.bbox.size.y)
        ros_msg.bbox.size.z = float(self.bbox.size.z)

        # Set results (object hypotheses)
        for result in self.results:
            ros_result = ROSObjectHypothesisWithPose()
            ros_result.hypothesis = ROSObjectHypothesis()
            ros_result.hypothesis.class_id = str(result.hypothesis.class_id)
            ros_result.hypothesis.score = float(result.hypothesis.score)
            ros_msg.results.append(ros_result)

        # Set ID if available
        if hasattr(self, "id"):
            ros_msg.id = str(self.id)

        return ros_msg
