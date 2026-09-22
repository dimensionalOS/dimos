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

"""DimSim launch and scene setup for live agent evals."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, cast

from dimos.e2e_tests.dim_sim_client import DimSimClient
from dimos.evals.environments.sim import Sim, SimConfig

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class DimSimEnvironmentConfig(SimConfig):
    scene: str = "apartment"
    setup: Callable[[DimSimClient], None] | None = None


class DimSimEnvironment(Sim):
    config: DimSimEnvironmentConfig

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = "dimsim"
        proc.global_args = ["--dimsim-scene", self.config.scene]

    def setup_scene(self) -> None:
        if self.config.setup is not None:
            client = DimSimClient()
            self._resources.callback(client.stop)
            client.start()
            self.config.setup(client)

    def latest_pose(self, recording: Store) -> PoseStamped:
        if "odom" not in recording.streams:
            raise LookupError("No DimSim odometry recorded")
        return cast("PoseStamped", recording.streams.odom.last().data)
