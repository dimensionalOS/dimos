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

"""The G1 desk asset and the sensor mounts it exposes."""

from dimos.core.hardware_module import Mounts
from dimos.robot.assets.model import UsdModel
from dimos.sim2.hardware import SimHardwareConfig, SimHardwareModule
from dimos.utils.data import LfsPath

PELVIS = "world/g1_29dof_rev_1_0/Geometry/pelvis"
TORSO = PELVIS + "/waist_yaw_link/waist_roll_link/torso_link"


class G1DeskConfig(SimHardwareConfig):
    usd: UsdModel = UsdModel.from_file(LfsPath("sim2_g1_usd/world.usda"))


class G1Desk(SimHardwareModule):
    config: G1DeskConfig
    mounts = Mounts(PELVIS + "/imu_mount", TORSO + "/camera_mount", TORSO + "/lidar_mount")
