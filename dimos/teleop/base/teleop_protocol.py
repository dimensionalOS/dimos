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

"""
Teleoperation Protocol.

Defines the interface that all teleoperation modules must implement.
No implementation - just method signatures.
"""

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class TeleopProtocol(Protocol):
    """Protocol defining the teleoperation interface.

    All teleop modules (Quest, keyboard, joystick, etc.) should implement these methods.
    No state or implementation here - just the contract.
    """

    def start(self) -> None:
        """Start the teleoperation module."""
        ...

    def stop(self) -> None:
        """Stop the teleoperation module."""
        ...

    def engage(self) -> bool:
        """Engage teleoperation. Returns True on success."""
        ...

    def disengage(self) -> None:
        """Disengage teleoperation."""
        ...

    def get_status(self) -> dict[str, Any]:
        """Get current teleoperation status."""
        ...
