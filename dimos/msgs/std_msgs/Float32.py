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

"""Float32 message type."""

from typing import ClassVar

from dimos_lcm.std_msgs import Float32 as LCMFloat32

try:
    from std_msgs.msg import Float32 as ROSFloat32  # type: ignore[attr-defined]
except ImportError:
    ROSFloat32 = None  # type: ignore[assignment, misc]


class Float32(LCMFloat32):  # type: ignore[misc]
    """ROS-compatible Float32 message."""

    msg_name: ClassVar[str] = "std_msgs.Float32"

    def __init__(self, data: float = 0.0) -> None:
        """Initialize Float32 with data value."""
        self.data = data

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSFloat32) -> "Float32":
        """Create a Float32 from a ROS std_msgs/Float32 message.

        Args:
            ros_msg: ROS Float32 message

        Returns:
            Float32 instance
        """
        return cls(data=ros_msg.data)

    def to_ros_msg(self) -> ROSFloat32:
        """Convert to a ROS std_msgs/Float32 message.

        Returns:
            ROS Float32 message
        """
        if ROSFloat32 is None:
            raise ImportError("ROS std_msgs not available")
        ros_msg = ROSFloat32()  # type: ignore[no-untyped-call]
        ros_msg.data = float(self.data)
        return ros_msg
