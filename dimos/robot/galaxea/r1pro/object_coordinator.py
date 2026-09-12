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

"""Resolve a generated object scene before opening its shared-memory adapter."""

from pathlib import Path

from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.hardware.spec import JointLimits
from dimos.robot.galaxea.r1pro.object_packing_run import ObjectPackingSimSpec


class R1ProObjectCoordinator(ControlCoordinator):
    _object_sim: ObjectPackingSimSpec

    @rpc
    def build(self) -> None:
        session = self._object_sim.prepare_object_session()
        hardware = next(h for h in self.config.hardware if h.hardware_id == "r1pro")
        hardware.address = Path(session["scene"])
        hardware.limits = JointLimits(
            position_lower=[r[0] for r in session["limits"]],
            position_upper=[r[1] for r in session["limits"]],
            velocity_max=[2.0] * 18 + [0.25, 0.25, 0.4, 0.4, 0.4],
        )
        super().build()
