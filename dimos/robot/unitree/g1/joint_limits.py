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

"""Physical G1 joint limits from the bundled robot description."""

from functools import cache
from pathlib import Path
import xml.etree.ElementTree as ET


@cache
def _g1_velocity_limit_items() -> tuple[tuple[str, float], ...]:
    root = ET.parse(Path(__file__).with_name("g1.urdf")).getroot()
    limits: dict[str, float] = {}
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        limit = joint.find("limit")
        velocity = None if limit is None else limit.get("velocity")
        if not name.endswith("_joint") or velocity is None:
            continue
        limits[name.removesuffix("_joint")] = float(velocity)
    return tuple(limits.items())


def g1_velocity_limits(hardware_id: str = "g1") -> dict[str, float]:
    """Return URDF velocity limits in coordinator joint-name form."""
    return {f"{hardware_id}/{name}": velocity for name, velocity in _g1_velocity_limit_items()}
