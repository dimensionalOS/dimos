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

"""Bool message type."""

from dimos_lcm.std_msgs import Bool as LCMBool  # type: ignore[import-untyped]

try:
    from std_msgs.msg import Bool as ROSBool  # type: ignore[attr-defined, import-untyped]
except ImportError:
    ROSBool = None  # type: ignore[assignment, misc]


class Bool(LCMBool):  # type: ignore[misc]
    """ROS-compatible Bool message."""

    msg_name = "std_msgs.Bool"

    def __init__(self, data: bool = False) -> None:
        """Initialize Bool with data value."""
        self.data = data

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSBool) -> "Bool":
        """Create a Bool from a ROS std_msgs/Bool message.

        Args:
            ros_msg: ROS Bool message

        Returns:
            Bool instance
        """
        return cls(data=ros_msg.data)

    def to_ros_msg(self) -> ROSBool:
        """Convert to a ROS std_msgs/Bool message.

        Returns:
            ROS Bool message
        """
        if ROSBool is None:
            raise ImportError("ROS std_msgs not available")
        ros_msg = ROSBool()  # type: ignore[no-untyped-call]
        ros_msg.data = bool(self.data)
        return ros_msg
