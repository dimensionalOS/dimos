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

"""A Go2 as DimSim's kinematic base: moved as a box on cmd_vel, with a Go2's USD, mounts and odometry frame."""

from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.go2.sim2 import GO2_ODOM_FRAME, GO2_USD, Go2Sim
from dimos.sim2.dimsim.kinematic_base import SimKinematicBase, SimKinematicBaseConfig


class Go2KinematicBaseConfig(SimKinematicBaseConfig):
    usd: UsdModel = GO2_USD
    odom_frame: str = GO2_ODOM_FRAME


class Go2KinematicBase(SimKinematicBase):
    config: Go2KinematicBaseConfig
    mounts = Go2Sim.mounts
