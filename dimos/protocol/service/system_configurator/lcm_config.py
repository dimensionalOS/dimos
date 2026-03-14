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

"""Platform-appropriate LCM system configurators."""

import platform

from dimos.protocol.service.system_configurator.base import SystemConfigurator
from dimos.protocol.service.system_configurator.lcm import (
    BufferConfiguratorLinux,
    BufferConfiguratorMacOS,
    MaxFileConfiguratorMacOS,
    MulticastConfiguratorLinux,
    MulticastConfiguratorMacOS,
)
from dimos.protocol.service.system_configurator.libpython import LibPythonConfiguratorMacOS


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
            MaxFileConfiguratorMacOS(),
            LibPythonConfiguratorMacOS(),
        ]
    return []
