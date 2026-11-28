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


"""A simulated G1 body: its 29 joints driven by joint commands, its joint states and pelvis IMU read off the world, and mounts for a lidar and a camera."""

import math
from pathlib import Path

from pydantic import Field

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.core.hardware_module import Mounts
from dimos.hardware.sensors.lidar.pointlio.module import PointLioConfig
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.config import G1_JOINT_NAMES
from dimos.robot.unitree.g1.g1_tf_publisher import (
    _TORSO_D435_XYZ,
    _TORSO_MID360_XYZ,
    D435_PITCH,
    MID360_PITCH,
    G1TfPublisherConfig,
)
from dimos.sim2.body import SimBody, SimBodyConfig, with_sites

# Its root link, the pelvis; named here so building a blueprint needs no USD read.
G1_BODY = "g1_29dof_rev_1_0/Geometry/pelvis"
G1_TORSO = f"{G1_BODY}/waist_yaw_link/waist_roll_link/torso_link"
# Its head camera's mount (x forward), where the real G1 has its D435.
G1_CAMERA = f"{G1_TORSO}/camera"
# The pelvis IMU, a site on its root link.
G1_IMU = f"{G1_BODY}/imu_in_pelvis"

# The frame a real G1 reports its odometry in: Point-LIO's, which it namespaces itself.
G1_ODOM_FRAME: str = PointLioConfig.model_fields["frame_id"].default

# The frames of the odometry tf a real G1 publishes (its pelvis is its base_link), for the OdomTf beside a simulated one.
G1_ODOM_TF: dict[str, dict[str, str]] = {
    "tf_frames_local": {"odom": G1_ODOM_FRAME, "body": "base_link"}
}


def g1_body_usd() -> UsdModel:
    """The G1, with its Mid-360 and D435 mounts (each x forward) and its pelvis IMU marked as Isaac Sim sites."""
    framed = UsdModel.from_file(Path(__file__).parent / "assets" / "g1.usda").with_frames(
        {
            # The real Mid-360 hangs upside down under the head.
            "lidar": (G1_TORSO, _TORSO_MID360_XYZ, (math.pi, MID360_PITCH, 0.0)),
            "camera": (G1_TORSO, _TORSO_D435_XYZ, (0.0, D435_PITCH, 0.0)),
        }
    )
    return with_sites(
        framed,
        {
            f"{G1_TORSO}/lidar": ("lidar mount", "X"),
            G1_CAMERA: ("head camera mount", "X"),
            G1_IMU: ("pelvis IMU", "X"),
        },
    )


class G1BodySimConfig(SimBodyConfig):
    usd: UsdModel = Field(default_factory=g1_body_usd)
    joint_names: tuple[str, ...] = tuple(G1_JOINT_NAMES)
    # GR00T's gains, as the real G1 runs them.
    kp: tuple[float, ...] | None = tuple(G1_GROOT_KP)
    kd: tuple[float, ...] | None = tuple(G1_GROOT_KD)
    imu_site: str | None = G1_IMU
    # The real G1's tf goes out at its mount publisher's rate.
    tf_rate_hz: float = G1TfPublisherConfig.model_fields["publish_hz"].default
    tf_frames_local: dict[str, str] = {"base_link": G1_BODY}
    odom_frame: str = G1_ODOM_FRAME


class G1BodySim(SimBody):
    """A G1 body in a simulated world, held at its USD's pose until a joint command comes; no policy keeps it up."""

    config: G1BodySimConfig
    mounts = Mounts(f"{G1_TORSO}/lidar", G1_CAMERA)
