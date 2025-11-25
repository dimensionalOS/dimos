# Copyright 2025 Dimensional Inc.
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


from isaacsim import SimulationApp  # type: ignore[import-not-found]

from ..base.simulator_base import SimulatorBase


class IsaacSimulator(SimulatorBase):
    """Isaac Sim simulator implementation."""

    def __init__(
        self,
        headless: bool = True,
        open_usd: str | None = None,
        entities: list[dict[str, str | dict]] | None = None,  # type: ignore[type-arg]  # Add but ignore
    ) -> None:
        """Initialize the Isaac Sim simulation."""
        super().__init__(headless, open_usd)
        self.app = SimulationApp({"headless": headless, "open_usd": open_usd})

    def get_stage(self):  # type: ignore[no-untyped-def]
        """Get the current USD stage."""
        import omni.usd  # type: ignore[import-not-found]

        self.stage = omni.usd.get_context().get_stage()
        return self.stage

    def close(self) -> None:
        """Close the simulation."""
        if hasattr(self, "app"):
            self.app.close()
