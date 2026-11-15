# Copyright 2026 Dimensional Inc.
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

"""ROS ``visualization_msgs/InteractiveMarkerUpdate``: named poses to set and names to erase."""

from __future__ import annotations

from typing import BinaryIO

from dimos_lcm.visualization_msgs import (
    InteractiveMarkerPose as LCMInteractiveMarkerPose,
    InteractiveMarkerUpdate as LCMInteractiveMarkerUpdate,
)

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.std_msgs.Header import Header


def named_pose(name: str, pose: PoseStamped) -> LCMInteractiveMarkerPose:
    """An ``InteractiveMarkerPose`` carrying *pose* (with its frame and stamp) under *name*."""
    return LCMInteractiveMarkerPose(
        header=Header(pose.ts, pose.frame_id), pose=Pose(pose.position, pose.orientation), name=name
    )


def pose_of(named: LCMInteractiveMarkerPose) -> PoseStamped:
    """The stamped pose an ``InteractiveMarkerPose`` carries."""
    stamp = named.header.stamp
    return PoseStamped(
        ts=stamp.sec + stamp.nsec / 1e9,
        frame_id=named.header.frame_id,
        position=[named.pose.position.x, named.pose.position.y, named.pose.position.z],
        orientation=[
            named.pose.orientation.x,
            named.pose.orientation.y,
            named.pose.orientation.z,
            named.pose.orientation.w,
        ],
    )


class InteractiveMarkerUpdate(LCMInteractiveMarkerUpdate):  # type: ignore[misc]
    """``poses`` are set (added or moved) by name and ``erases`` are removed. ``markers`` stays empty here."""

    msg_name = "visualization_msgs.InteractiveMarkerUpdate"

    def __init__(
        self,
        poses: list[LCMInteractiveMarkerPose] | None = None,
        erases: list[str] | None = None,
        server_id: str = "",
        seq_num: int = 0,
    ) -> None:
        super().__init__(
            server_id=server_id,
            seq_num=seq_num,
            type=LCMInteractiveMarkerUpdate.UPDATE,
            markers=[],
            poses=poses or [],
            erases=erases or [],
        )

    # The LCM wire counts, always derived; the base decoder assigns them.
    @property
    def markers_length(self) -> int:
        return len(self.markers)

    @markers_length.setter
    def markers_length(self, _: int) -> None:
        pass

    @property
    def poses_length(self) -> int:
        return len(self.poses)

    @poses_length.setter
    def poses_length(self, _: int) -> None:
        pass

    @property
    def erases_length(self) -> int:
        return len(self.erases)

    @erases_length.setter
    def erases_length(self, _: int) -> None:
        pass

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> InteractiveMarkerUpdate:
        msg = LCMInteractiveMarkerUpdate.lcm_decode(data)
        return cls(list(msg.poses), list(msg.erases), msg.server_id, msg.seq_num)

    def __repr__(self) -> str:
        names = [p.name for p in self.poses]
        return f"InteractiveMarkerUpdate(seq={self.seq_num}, poses={names}, erases={self.erases})"
