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

"""What ``DamiaoConnection.status`` returns, importable without the motor SDK.

An RPC client decodes a status by importing this module, so it imports
neither the Damiao SDK nor pinocchio.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from dimos.control.connection.connection_module import ConnectionStatus
from dimos.robot.manipulators.calibration.hooks import AppliedCalibration


@dataclass(frozen=True, slots=True)
class DamiaoConnectionStatus(ConnectionStatus):
    """``ConnectionStatus`` plus the calibration each arm runs.

    Attributes:
        calibration: By arm id, the file that arm runs, or ``None`` for its
            URDF masses and bus gains.
    """

    calibration: dict[str, AppliedCalibration | None] = field(default_factory=dict)
