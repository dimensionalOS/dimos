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

"""TwistBase adapter protocol for velocity-commanded platforms.

Lightweight protocol for mobile bases, quadrupeds, drones, RC cars,
and any other platform that accepts Twist (velocity) commands.

Virtual joint ordering is defined by the HardwareComponent.joints list.
For a holonomic base: [vx, vy, wz] maps to joints ["base_vx", "base_vy", "base_wz"].
"""

from typing import Protocol, runtime_checkable


@runtime_checkable
class TwistBaseAdapter(Protocol):
    """Protocol for velocity-commanded platform IO.

    Implement this per vendor SDK. All methods use SI units:
    - Linear velocity: m/s
    - Angular velocity: rad/s
    - Position: meters
    - Angle: radians
    """

    # --- Connection ---

    def connect(self) -> bool:
        """Connect to hardware. Returns True on success."""
        ...

    def disconnect(self) -> None:
        """Disconnect from hardware."""
        ...

    def is_connected(self) -> bool:
        """Check if connected."""
        ...

    # --- Info ---

    def get_dof(self) -> int:
        """Get number of velocity DOFs (e.g., 3 for holonomic, 2 for differential)."""
        ...

    # --- State Reading ---

    def read_velocities(self) -> list[float]:
        """Read current velocities in virtual joint order (m/s or rad/s)."""
        ...

    def read_odometry(self) -> list[float] | None:
        """Read position estimate in virtual joint order.

        For a holonomic base this would be [x, y, theta].
        Returns None if the platform doesn't provide odometry.
        """
        ...

    # --- Control ---

    def write_velocities(self, velocities: list[float]) -> bool:
        """Command velocities in virtual joint order. Returns success."""
        ...

    def write_stop(self) -> bool:
        """Stop all motion immediately (zero velocities)."""
        ...

    # --- Enable/Disable ---

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable the platform. Returns success."""
        ...

    def read_enabled(self) -> bool:
        """Check if platform is enabled."""
        ...


__all__ = [
    "TwistBaseAdapter",
]
