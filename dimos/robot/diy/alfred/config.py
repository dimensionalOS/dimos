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

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

DEFAULT_ADDRESS = "172.6.2.20:11323"
ALFRED_URDF = Path(__file__).resolve().parent / "alfred.urdf"


@dataclass(frozen=True)
class AlfredConfig:
    """Physical metadata used by Alfred navigation and sensor blueprints."""

    name: str
    body_height: float
    height_clearance: float
    width_clearance: float
    mid360_ip: str
    d455_serial: str
    internal_odom_offsets: dict[str, Any] = field(default_factory=dict)


ALFRED = AlfredConfig(
    name="alfred",
    # NOT the planner's headroom - alfred-nav passes ALFRED_HEIGHT_M (1.86 m, the
    # top of the collision scene) for that. The claim this comment used to make,
    # that nothing above 0.5 m is a collision risk, is contradicted by the URDF:
    # the mast alone reaches 1.57 m. Kept because the MLS planner's body_height
    # is also used as the free-span heuristic elsewhere.
    body_height=0.5,
    # Unused by any Alfred blueprint - only the G1's read these two fields. They
    # look authoritative here and are not; ALFRED_FOOTPRINT_RADIUS_M in
    # alfred_model is the measured number the planner is given.
    height_clearance=2.0,  # meters
    width_clearance=1.0,
    # The Mid-360 is on the Jetson's wired 192.168.1.100/24 link.
    mid360_ip="192.168.1.189",
    # The mast D455. The rear D435i stays plugged in, so the device is pinned by serial.
    d455_serial="260922302422",
    internal_odom_offsets={
        # Mid-360 lidar: a bit forward, and a bit to the right of base center, above ground.
        "mid360_link": Pose(0.20, -0.20, 0.30, *Quaternion.from_euler(Vector3(0, 0, 0))),
    },
)
