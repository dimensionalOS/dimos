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

"""Typed virtual-hardware boundary; no scene or other-robot state crosses it."""

from dataclasses import dataclass
from typing import Literal

import numpy as np
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from microduck_world.roster import ROBOT_IDS
from numpy.typing import NDArray

VISITOR_IDS = ROBOT_IDS


@dataclass(frozen=True)
class RobotCommand:
    generation: str
    kind: Literal["twist", "policy", "respawn", "drop_ball"]
    value: tuple[float, float, float] | str


@dataclass(frozen=True)
class RobotState:
    generation: str
    odom: PoseStamped
    joints: JointState
    policy: str


@dataclass(frozen=True)
class RobotVision:
    generation: str
    image: Image
    depth: Image
    camera_info: CameraInfo
    camera_pose: PoseStamped
    tf: TFMessage
    points: NDArray[np.float32]


@dataclass(frozen=True)
class Observation:
    """A synchronized RGB-D observation in this robot's own map frame."""

    image: Image
    depth: Image
    camera_info: CameraInfo
    camera_pose: PoseStamped


def allowed_generation(actual: str, expected: str) -> bool:
    return bool(expected) and actual == expected
