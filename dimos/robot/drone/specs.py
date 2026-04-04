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

"""Protocol specs for drone spatial memory wiring."""

from typing import Any, Protocol

from dimos.spec.utils import Spec


class SpatialMemoryQuerySpec(Spec, Protocol):
    """Spatial memory text search (CLIP)."""

    def query_by_text(self, text: str, limit: int = 5) -> list[dict[str, Any]]:
        """Return ranked matches with metadata (pos_x, pos_y, pos_z)."""
        ...

    def get_stats(self) -> dict[str, int]:
        """Return frame_count and stored_frame_count."""
        ...


class DroneGoToPositionSpec(Spec, Protocol):
    """Drone local NED position target."""

    def go_to_position(
        self,
        x: float,
        y: float,
        z: float,
        vx_ff: float = 0.0,
        vy_ff: float = 0.0,
        vz_ff: float = 0.0,
    ) -> bool:
        ...


__all__ = ["SpatialMemoryQuerySpec", "DroneGoToPositionSpec"]
