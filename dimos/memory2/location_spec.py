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

"""Module-facing interface for durable named locations.

Replaces :class:`~dimos.perception.spatial_memory_spec.SpatialMemorySpec`, which
cannot express this design:

- ``RobotLocation.frame_id`` is *the associated video frame id*, not a tf frame —
  the name collides with the concept we need and means something else.
- There is no ``map_id``, no anchored/run-local distinction, no lineage.
- ``RobotLocation.rotation`` is documented and consumed as roll/pitch/yaw, so it
  cannot losslessly hold an orientation. The legacy path stored quaternion
  ``(x, y, z)`` there and read it back through ``Quaternion.from_euler``, which
  compresses every saved heading into a ±57° cone.

An adapter over the old Spec would therefore have to either drop the new
semantics or lie about them, so the Spec changes instead. The two run in
parallel during migration.
"""

from __future__ import annotations

from typing import Protocol

from dimos.memory2.locations import SavedLocation
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.spec.utils import Spec

__all__ = ["LocationMemorySpec", "PoseStamped", "SavedLocation"]


class LocationMemorySpec(Spec, Protocol):
    """Save, find, and resolve durable named locations.

    ``resolve_location`` raises a :class:`~dimos.memory2.locations.LocationError`
    subclass instead of returning a best-effort pose — callers are expected to
    turn each one into a specific message rather than issuing a goal.
    """

    def save_location(self, name: str) -> SavedLocation: ...

    def resolve_location(self, name: str, target_frame: str = "world") -> PoseStamped: ...

    def find_locations(self, query: str, limit: int = 5) -> list[SavedLocation]: ...

    def list_locations(self) -> list[SavedLocation]: ...

    def delete_location(self, name: str) -> bool: ...
