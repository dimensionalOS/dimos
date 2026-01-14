#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Teleop Connectors Package.

Connectors are injectable classes that transform delta poses into robot commands.
They are injected into VRTeleopModule (or other teleop modules) to handle per-input control logic.

Example usage:
    from dimos.teleop.connectors import ArmConnector, ArmConnectorConfig

    connector = ArmConnector(ArmConnectorConfig(
        driver_module_name="MyArmDriver",
        dummy_driver=True,
    ))

    vr_module = VRTeleopModule(config=VRTeleopConfig(
        connectors=[connector, connector],  # One per input
    ))
"""

from dimos.teleop.connectors.arm_connector import ArmConnector, ArmConnectorConfig
from dimos.teleop.connectors.base_connector import BaseTeleopConnector, ConnectorConfig
from dimos.teleop.connectors.quadruped_connector import (
    QuadrupedConnector,
    QuadrupedConnectorConfig,
)

__all__ = [
    "BaseTeleopConnector",
    "ConnectorConfig",
    "ArmConnector",
    "ArmConnectorConfig",
    "QuadrupedConnector",
    "QuadrupedConnectorConfig",
]
