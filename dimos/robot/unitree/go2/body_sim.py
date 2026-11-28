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


"""A simulated Go2 body: its 12 joints driven by joint commands, its joint states and IMU read off the world, and mounts for a lidar and a camera."""

from pathlib import Path

from pydantic import Field

from dimos.core.hardware_module import Mounts
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    MID360_PITCH_DOWN,
    MID360_XYZ,
    OPTICAL_RPY,
)
from dimos.sim2.body import SimBody, SimBodyConfig, with_sites

# Its root link: the prim the real Go2's base_link is.
GO2_BODY = "go2/Geometry/base_link"
# Its front camera's housing, which the lidar bracket sits on as on the real rig.
GO2_FRONT_CAMERA = f"{GO2_BODY}/front_camera"
# Its IMU, a site on its root link.
GO2_IMU = f"{GO2_BODY}/imu"

GO2_JOINT_NAMES = tuple(
    f"go2/{leg}_{part}" for leg in ("FL", "FR", "RL", "RR") for part in ("hip", "thigh", "calf")
)


def go2_body_usd() -> UsdModel:
    """The articulated Go2, with its lidar and camera mounts (the camera's an optical frame) and its IMU marked as Isaac Sim sites."""
    framed = UsdModel.from_file(Path(__file__).parent / "assets" / "go2.usda").with_frames(
        {
            "front_camera": (GO2_BODY, CAMERA_XYZ, (0.0, 0.0, 0.0)),
            "camera": (GO2_FRONT_CAMERA, (0.0, 0.0, 0.0), OPTICAL_RPY),
            "lidar": (GO2_FRONT_CAMERA, MID360_XYZ, (0.0, MID360_PITCH_DOWN, 0.0)),
        }
    )
    return with_sites(
        framed,
        {
            f"{GO2_FRONT_CAMERA}/camera": ("front camera mount, an optical frame", "Z"),
            f"{GO2_FRONT_CAMERA}/lidar": ("lidar mount", "X"),
            GO2_IMU: ("IMU", "X"),
        },
    )


class Go2BodySimConfig(SimBodyConfig):
    usd: UsdModel = Field(default_factory=go2_body_usd)
    joint_names: tuple[str, ...] = GO2_JOINT_NAMES
    imu_site: str | None = GO2_IMU
    tf_frames_local: dict[str, str] = {"base_link": GO2_BODY}


class Go2BodySim(SimBody):
    """A Go2 body in a simulated world, held at its USD's standing pose until a joint command comes."""

    config: Go2BodySimConfig
    mounts = Mounts(f"{GO2_FRONT_CAMERA}/lidar", f"{GO2_FRONT_CAMERA}/camera")
