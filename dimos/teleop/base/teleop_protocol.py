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

"""Teleoperation specifications: Protocol.

Defines the interface that all teleoperation modules must implement.
No implementation - just method signatures.
"""

from typing import Any, Protocol, runtime_checkable

# ============================================================================
# TELEOP PROTOCOL
# ============================================================================


@runtime_checkable
class TeleopProtocol(Protocol):
    """Protocol defining the teleoperation interface.

    All teleop modules (Quest, keyboard, joystick, etc.) should implement these methods.
    No state or implementation here - just the contract.
    """

    # --- Lifecycle ---

    def start(self) -> None:
        """Start the teleoperation module."""
        ...

    def stop(self) -> None:
        """Stop the teleoperation module."""
        ...

    # --- Engage / Disengage ---

    def engage(self, hand: Any = None) -> bool:
        """Engage teleoperation. Hand type is device-specific (e.g., Hand enum for Quest)."""
        ...

    def disengage(self, hand: Any = None) -> None:
        """Disengage teleoperation. Hand type is device-specific."""
        ...


__all__ = ["TeleopProtocol"]
