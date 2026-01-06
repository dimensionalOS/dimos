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

from dimos_lcm.sensor_msgs import Joy as LCMJoy  # type: ignore[import-untyped]

try:
    from sensor_msgs.msg import Joy as ROSJoy  # type: ignore[attr-defined, import-untyped]
except ImportError:
    ROSJoy = None  # type: ignore[assignment, misc]

from plum import dispatch

from dimos.types.timestamped import Timestamped

# Types that can be converted to/from Joy
JoyConvertable: TypeAlias = (
    tuple[list[float], list[int]] | dict[str, list[float] | list[int]] | LCMJoy
)


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class Joy(Timestamped):
    msg_name = "sensor_msgs.Joy"
    ts: float
    frame_id: str
    axes: list[float]
    buttons: list[int]

    @dispatch
    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "",
        axes: list[float] | None = None,
        buttons: list[int] | None = None,
    ) -> None:
        """Initialize a Joy message.

        Args:
            ts: Timestamp in seconds
            frame_id: Frame ID for the message
            axes: List of axis values (typically -1.0 to 1.0)
            buttons: List of button states (0 or 1)
        """
        self.ts = ts if ts != 0 else time.time()
        self.frame_id = frame_id
        self.axes = axes if axes is not None else []
        self.buttons = buttons if buttons is not None else []

    @dispatch  # type: ignore[no-redef]
    def __init__(self, joy_tuple: tuple[list[float], list[int]]) -> None:
        """Initialize from a tuple of (axes, buttons)."""
        self.ts = time.time()
        self.frame_id = ""
        self.axes = list(joy_tuple[0])
        self.buttons = list(joy_tuple[1])

    @dispatch  # type: ignore[no-redef]
    def __init__(self, joy_dict: dict[str, list[float] | list[int]]) -> None:
        """Initialize from a dictionary with 'axes' and 'buttons' keys."""
        self.ts = joy_dict.get("ts", time.time())
        self.frame_id = joy_dict.get("frame_id", "")
        self.axes = list(joy_dict.get("axes", []))
        self.buttons = list(joy_dict.get("buttons", []))

    @dispatch  # type: ignore[no-redef]
    def __init__(self, joy: Joy) -> None:
        """Initialize from another Joy (copy constructor)."""
        self.ts = joy.ts
        self.frame_id = joy.frame_id
        self.axes = list(joy.axes)
        self.buttons = list(joy.buttons)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, lcm_joy: LCMJoy) -> None:
        """Initialize from an LCM Joy message."""
        self.ts = lcm_joy.header.stamp.sec + (lcm_joy.header.stamp.nsec / 1_000_000_000)
        self.frame_id = lcm_joy.header.frame_id
        self.axes = list(lcm_joy.axes)
        self.buttons = list(lcm_joy.buttons)

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMJoy()
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        lcm_msg.axes_length = len(self.axes)
        lcm_msg.axes = self.axes
        lcm_msg.buttons_length = len(self.buttons)
        lcm_msg.buttons = self.buttons
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> Joy:
        lcm_msg = LCMJoy.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            axes=list(lcm_msg.axes) if lcm_msg.axes else [],
            buttons=list(lcm_msg.buttons) if lcm_msg.buttons else [],
        )

    def __str__(self) -> str:
        return (
            f"Joy(axes={len(self.axes)} values, buttons={len(self.buttons)} values, "
            f"frame_id='{self.frame_id}')"
        )

    def __repr__(self) -> str:
        return (
            f"Joy(ts={self.ts}, frame_id='{self.frame_id}', "
            f"axes={self.axes}, buttons={self.buttons})"
        )

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two Joy messages are equal."""
        if not isinstance(other, Joy):
            return False
        return (
            self.axes == other.axes
            and self.buttons == other.buttons
            and self.frame_id == other.frame_id
        )

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSJoy) -> Joy:
        """Create a Joy from a ROS sensor_msgs/Joy message.

        Args:
            ros_msg: ROS Joy message

        Returns:
            Joy instance
        """
        # Convert timestamp from ROS header
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        return cls(
            ts=ts,
            frame_id=ros_msg.header.frame_id,
            axes=list(ros_msg.axes),
            buttons=list(ros_msg.buttons),
        )

    def to_ros_msg(self) -> ROSJoy:
        """Convert to a ROS sensor_msgs/Joy message.

        Returns:
            ROS Joy message
        """
        ros_msg = ROSJoy()  # type: ignore[no-untyped-call]

        # Set header
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1_000_000_000)

        # Set axes and buttons
        ros_msg.axes = self.axes
        ros_msg.buttons = self.buttons

        return ros_msg
