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

import time
from typing import TypeAlias

from dimos_lcm.sensor_msgs import JointState as LCMJointState

try:
    from sensor_msgs.msg import JointState as ROSJointState
except ImportError:
    ROSJointState = None

from plum import dispatch

from dimos.types.timestamped import Timestamped

# Types that can be converted to/from JointState
JointStateConvertable: TypeAlias = dict[str, list[str] | list[float]] | LCMJointState


def sec_nsec(ts):
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class JointState(Timestamped):
    msg_name = "sensor_msgs.JointState"
    ts: float
    frame_id: str
    name: list[str]
    position: list[float]
    velocity: list[float]
    effort: list[float]

    @dispatch
    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "",
        name: list[str] | None = None,
        position: list[float] | None = None,
        velocity: list[float] | None = None,
        effort: list[float] | None = None,
    ) -> None:
        """Initialize a JointState message.

        Args:
            ts: Timestamp in seconds
            frame_id: Frame ID for the message
            name: List of joint names
            position: List of joint positions (rad or m)
            velocity: List of joint velocities (rad/s or m/s)
            effort: List of joint efforts (Nm or N)
        """
        self.ts = ts if ts != 0 else time.time()
        self.frame_id = frame_id
        self.name = name if name is not None else []
        self.position = position if position is not None else []
        self.velocity = velocity if velocity is not None else []
        self.effort = effort if effort is not None else []

    @dispatch
    def __init__(self, joint_dict: dict[str, list[str] | list[float]]) -> None:
        """Initialize from a dictionary."""
        self.ts = joint_dict.get("ts", time.time())
        self.frame_id = joint_dict.get("frame_id", "")
        self.name = list(joint_dict.get("name", []))
        self.position = list(joint_dict.get("position", []))
        self.velocity = list(joint_dict.get("velocity", []))
        self.effort = list(joint_dict.get("effort", []))

    @dispatch
    def __init__(self, joint: JointState) -> None:
        """Initialize from another JointState (copy constructor)."""
        self.ts = joint.ts
        self.frame_id = joint.frame_id
        self.name = list(joint.name)
        self.position = list(joint.position)
        self.velocity = list(joint.velocity)
        self.effort = list(joint.effort)

    @dispatch
    def __init__(self, lcm_joint: LCMJointState) -> None:
        """Initialize from an LCM JointState message."""
        self.ts = lcm_joint.header.stamp.sec + (lcm_joint.header.stamp.nsec / 1_000_000_000)
        self.frame_id = lcm_joint.header.frame_id
        self.name = list(lcm_joint.name) if lcm_joint.name else []
        self.position = list(lcm_joint.position) if lcm_joint.position else []
        self.velocity = list(lcm_joint.velocity) if lcm_joint.velocity else []
        self.effort = list(lcm_joint.effort) if lcm_joint.effort else []

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMJointState()
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)
        lcm_msg.header.frame_id = self.frame_id
        lcm_msg.name_length = len(self.name)
        lcm_msg.name = self.name
        lcm_msg.position_length = len(self.position)
        lcm_msg.position = self.position
        lcm_msg.velocity_length = len(self.velocity)
        lcm_msg.velocity = self.velocity
        lcm_msg.effort_length = len(self.effort)
        lcm_msg.effort = self.effort
        return lcm_msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes) -> JointState:
        lcm_msg = LCMJointState.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            name=list(lcm_msg.name) if lcm_msg.name else [],
            position=list(lcm_msg.position) if lcm_msg.position else [],
            velocity=list(lcm_msg.velocity) if lcm_msg.velocity else [],
            effort=list(lcm_msg.effort) if lcm_msg.effort else [],
        )

    def __str__(self) -> str:
        return f"JointState({len(self.name)} joints, frame_id='{self.frame_id}')"

    def __repr__(self) -> str:
        return (
            f"JointState(ts={self.ts}, frame_id='{self.frame_id}', "
            f"name={self.name}, position={self.position}, "
            f"velocity={self.velocity}, effort={self.effort})"
        )

    def __eq__(self, other) -> bool:
        """Check if two JointState messages are equal."""
        if not isinstance(other, JointState):
            return False
        return (
            self.name == other.name
            and self.position == other.position
            and self.velocity == other.velocity
            and self.effort == other.effort
            and self.frame_id == other.frame_id
        )

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSJointState) -> JointState:
        """Create a JointState from a ROS sensor_msgs/JointState message.

        Args:
            ros_msg: ROS JointState message

        Returns:
            JointState instance
        """
        # Convert timestamp from ROS header
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        return cls(
            ts=ts,
            frame_id=ros_msg.header.frame_id,
            name=list(ros_msg.name),
            position=list(ros_msg.position),
            velocity=list(ros_msg.velocity),
            effort=list(ros_msg.effort),
        )

    def to_ros_msg(self) -> ROSJointState:
        """Convert to a ROS sensor_msgs/JointState message.

        Returns:
            ROS JointState message
        """
        ros_msg = ROSJointState()

        # Set header
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1_000_000_000)

        # Set joint data
        ros_msg.name = self.name
        ros_msg.position = self.position
        ros_msg.velocity = self.velocity
        ros_msg.effort = self.effort

        return ros_msg
