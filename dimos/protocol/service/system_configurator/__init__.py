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

"""System configurator package — re-exports for backward compatibility."""

import platform

from dimos.protocol.service.system_configurator.base import (
    SystemConfigurator,
    configure_system,
    sudo_run,
)
from dimos.protocol.service.system_configurator.clock_sync import ClockSyncConfigurator
from dimos.protocol.service.system_configurator.lcm import (
    IDEAL_RMEM_SIZE,
    BufferConfiguratorLinux,
    BufferConfiguratorMacOS,
    MaxFileConfiguratorMacOS,
    MulticastConfiguratorLinux,
    MulticastConfiguratorMacOS,
)


# TODO: This is a configurator API issue and inserted here temporarily
#
# We need to use different configurators based on the underlying OS
#
# We should have separation of concerns, nothing but configurators themselves care about the OS in this context
#
# So configurators with multi-os behavior should be responsible for the right per-OS behaviour, and
# not external systems
#
# We might want to have some sort of recursive configurators
#
def lcm_configurators() -> list[SystemConfigurator]:
    """Return the platform-appropriate LCM system configurators."""
    system = platform.system()
    if system == "Linux":
        return [
            MulticastConfiguratorLinux(loopback_interface="lo"),
            BufferConfiguratorLinux(),
        ]
    elif system == "Darwin":
        return [
            MulticastConfiguratorMacOS(loopback_interface="lo0"),
            BufferConfiguratorMacOS(),
            MaxFileConfiguratorMacOS(),  # TODO: this is not LCM related and shouldn't be here at all
        ]
    return []


__all__ = [
    "IDEAL_RMEM_SIZE",
    "BufferConfiguratorLinux",
    "BufferConfiguratorMacOS",
    "ClockSyncConfigurator",
    "MaxFileConfiguratorMacOS",
    "MulticastConfiguratorLinux",
    "MulticastConfiguratorMacOS",
    "SystemConfigurator",
    "configure_system",
    "lcm_configurators",
    "sudo_run",
]
