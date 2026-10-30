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

"""One simulated world, built from every module that sends Simulation a description.

class Go2Sim(Module):
    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {"simulated": SimulatedRobot("go2", ...)}
"""

from pathlib import Path
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.sim2.spec import Simulated, SimulatedRobot, SimulatedSensor, validate


class SimulationConfig(ModuleConfig):
    scene: Path | None = Field(default_factory=lambda m: m["g"].scene_package)


class Simulation(SharedConfigModule):
    """Engine-agnostic: an engine subclasses this and builds its world from self.simulated."""

    config: SimulationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Each sender's "simulated" value, keyed by its instance name.
        self.simulated: dict[str, Simulated] = {}

    def config_receive(self, config_senders: ConfigSenders) -> None:
        simulated: dict[str, Simulated] = {}
        for name, config in config_senders:
            description = config.get("simulated")
            if set(config) != {"simulated"} or not isinstance(description, Simulated):
                raise TypeError(
                    f"{name}'s @shared_config(Simulation) must return"
                    f" {{'simulated': SimulatedRobot | SimulatedSensor}}, not {config!r}"
                )
            simulated[name] = description
        self.simulated = simulated

    @rpc
    def build(self) -> None:
        super().build()
        validate(self.simulated.values())

    @rpc
    def describe(self) -> dict[str, Any]:
        """The robots and sensors in this world."""
        values = self.simulated.values()
        return {
            "robots": sorted(d.robot_id for d in values if isinstance(d, SimulatedRobot)),
            "sensors": sorted(
                f"{d.robot_id or 'world'}/{d.name}"
                for d in values
                if isinstance(d, SimulatedSensor)
            ),
        }
