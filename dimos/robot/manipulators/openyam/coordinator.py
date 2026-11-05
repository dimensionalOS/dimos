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

"""OpenYAM deployment configuration for standard and Quest coordinators."""

from dataclasses import replace

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.robot.manipulators.openyam.config import OPENYAM_HARDWARE_ID


class OpenYamCoordinatorConfig(ControlCoordinatorConfig):
    can_port: str | None = None


def _configure_can(config: OpenYamCoordinatorConfig) -> None:
    if config.can_port is None:
        return
    for component in config.hardware:
        if (
            component.hardware_id != OPENYAM_HARDWARE_ID
            or component.adapter_type != "openyam_damiao"
        ):
            continue
        runtime = component.adapter_kwargs["runtime_config"]
        if not isinstance(runtime, DamiaoRuntimeConfig):
            runtime = DamiaoRuntimeConfig(**runtime)
        component.adapter_kwargs = {
            **component.adapter_kwargs,
            "runtime_config": replace(
                runtime, bus_devices={**runtime.bus_devices, "openyam": config.can_port}
            ),
        }


class OpenYamCoordinator(ControlCoordinator):
    config: OpenYamCoordinatorConfig

    def _setup_from_config(self) -> None:
        _configure_can(self.config)
        super()._setup_from_config()


class OpenYamTeleopCoordinator(TeleopControlCoordinator):
    config: OpenYamCoordinatorConfig

    def _setup_from_config(self) -> None:
        _configure_can(self.config)
        super()._setup_from_config()
