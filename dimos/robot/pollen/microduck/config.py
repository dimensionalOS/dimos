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

"""Microduck physical description, as the navigation stack needs it.

Mirrors dimos/robot/unitree/g1/config.py: one frozen descriptor the blueprints
read, rather than loose constants copied into each one.

These are CLEARANCES, not measurements - each carries margin over the duck's
real size, which is why they are named `*_clearance`. Measured from the
composed MJCF (the AABB of the robot root's body subtree, rest pose):

    width   14 cm      footprint circle  20 cm
    height  26 cm

They cannot be derived from that model here: a blueprint is a static
declaration evaluated long before the MuJoCo model is loaded, and neither
CostMapper nor ReplanningAStarPlanner can take a footprint at start-up. So
they are constants - but constants in one place, with the margin visible.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MicroduckConfig:
    """Physical metadata used by Microduck navigation blueprints."""

    name: str
    #: Planner footprint width. 14 cm of duck plus ~6 cm of margin; the
    #: costmap adds its own safe radius on top of this.
    width_clearance: float
    #: Ceiling the duck fits under. 26 cm of duck plus ~2 cm.
    height_clearance: float
    #: Diameter the duck sweeps turning in place. Its footprint circle is
    #: ~20 cm; the extra 10 cm keeps a turn off the walls.
    rotation_diameter: float


MICRODUCK = MicroduckConfig(
    name="microduck",
    width_clearance=0.2,
    height_clearance=0.28,
    rotation_diameter=0.3,
)
