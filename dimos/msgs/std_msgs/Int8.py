#!/usr/bin/env python3
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

# Copyright 2025 Dimensional Inc.

"""Int32 message type."""

from typing import ClassVar

from dimos_lcm.std_msgs import Int8 as LCMInt8  # type: ignore[import-untyped]

try:
    from std_msgs.msg import Int8 as ROSInt8  # type: ignore[attr-defined, import-untyped]
except ImportError:
    ROSInt8 = None  # type: ignore[assignment, misc]


class Int8(LCMInt8):  # type: ignore[misc]
    """ROS-compatible Int32 message."""

    msg_name: ClassVar[str] = "std_msgs.Int8"

    def __init__(self, data: int = 0) -> None:
        """Initialize Int8 with data value."""
        self.data = data

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSInt8) -> "Int8":
        """Create a Bool from a ROS std_msgs/Bool message.

        Args:
            ros_msg: ROS Int8 message

        Returns:
            Int8 instance
        """
        return cls(data=ros_msg.data)

    def to_ros_msg(self) -> ROSInt8:
        """Convert to a ROS std_msgs/Bool message.

        Returns:
            ROS Int8 message
        """
        if ROSInt8 is None:
            raise ImportError("ROS std_msgs not available")
        ros_msg = ROSInt8()  # type: ignore[no-untyped-call]
        ros_msg.data = self.data
        return ros_msg
