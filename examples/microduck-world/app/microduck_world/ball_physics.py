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

"""App-owned Pollen flat-floor physics, independent of shared DimOS constants."""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import mujoco

BALL_RADIUS = 0.05
BALL_MASS = 0.03
BALL_SPAWN_HEIGHT = BALL_RADIUS + 0.001
BALL_FRICTION = (0.4, 0.01, 0.003)
BALL_SOLREF = (0.03, 0.4)
FLOOR_NAMES = ("floor", "football_floor", "club_floor", "tunnel_floor", "corridor_floor")
FLOOR_FRICTION = (1.0, 0.005, 0.0001)
FLOOR_SOLREF = (0.02, 1.0)
CONTACT_SOLIMP = (0.9, 0.95, 0.001, 0.5, 2.0)


def configure_contacts(spec: "mujoco.MjSpec", ball_names: tuple[str, ...]) -> None:
    """Configure every physical ball and floor before mass/inertia are compiled.

    Pollen adds a sphere and plane outside the robot's named defaults. Keep
    their equal default priority and solmix so MuJoCo combines both surfaces.
    Decorative floors and ball paint remain outside this contact configuration.
    """
    spec.option.timestep = 0.005
    spec.option.gravity = [0, 0, -9.81]
    for name in (*FLOOR_NAMES, *(name + "_geom" for name in ball_names)):
        geom = spec.geom(name)
        ball = name not in FLOOR_NAMES
        geom.friction = BALL_FRICTION if ball else FLOOR_FRICTION
        geom.solref = BALL_SOLREF if ball else FLOOR_SOLREF
        geom.condim = 6 if ball else 3
        geom.solimp = CONTACT_SOLIMP
        geom.priority = 0
        geom.solmix = 1
        geom.margin = 0
        geom.gap = 0
        if ball:
            geom.size = [BALL_RADIUS, 0, 0]
            geom.mass = BALL_MASS
