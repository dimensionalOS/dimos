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

"""Coordinator setup that stays importable by the isolated policy runtime."""

from pathlib import Path

from dimos.control.path_following_coordinator import PathFollowingCoordinator
from dimos.core.core import rpc
from dimos.hardware.spec import JointLimits
from dimos.robot.galaxea.r1pro.home_spec import HomeSceneSpec


class R1ProHomeCoordinator(PathFollowingCoordinator):
    """Resolve the generated scene before connecting its shared-memory adapter."""

    _home_sim: HomeSceneSpec

    @rpc
    def build(self) -> None:
        session = self._home_sim.prepare_home_scene()
        hardware = next(item for item in self.config.hardware if item.hardware_id == "r1pro")
        hardware.address = Path(session["scene"])
        hardware.limits = JointLimits(
            position_lower=[item[0] for item in session["limits"]],
            position_upper=[item[1] for item in session["limits"]],
            velocity_max=[2.0] * 18 + [0.25, 0.25],
        )
        super().build()
