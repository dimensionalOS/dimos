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

"""Runtime CAN port selection for the Seeed B601-DM coordinator."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM_HARDWARE_ID, b601_dm_hardware
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SeeedB601DMCoordinatorConfig(ControlCoordinatorConfig):
    """B601-DM deployment: in-memory hardware unless a CAN port is given."""

    can_port: str | None = None
    gravity_comp: bool = True


class SeeedB601DMCoordinator(ControlCoordinator):
    """Select mock or physical B601-DM hardware during coordinator setup."""

    config: SeeedB601DMCoordinatorConfig

    def _setup_from_config(self) -> None:
        # The generic addresses map would reach only the mock adapter here.
        if B601_DM_HARDWARE_ID in self.config.addresses:
            raise ValueError("select the B601-DM port with --can-port, not addresses")
        can_port = self.config.can_port
        if can_port is not None and self.config.g.simulation:
            logger.warning("Simulation runs mock B601-DM hardware; ignoring --can-port")
            can_port = None
        self.config.hardware = [
            b601_dm_hardware(can_port=can_port, gravity_comp=self.config.gravity_comp)
        ]
        super()._setup_from_config()
