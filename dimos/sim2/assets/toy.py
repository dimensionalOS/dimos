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

"""The robot the sim2 tests drive: a box on a free joint with a hinged head and a motor on it.

The head sits clear of the box (0.3, 0, 0.2 from its centre, which is 0.2 x 0.1 x 0.1) rather
than touching its corner, so a sensor at the head's centre is inside the sphere and nothing
else. A ray from a point lying exactly on two surfaces is a coin toss, and a test that wants to
know what a sensor sees from inside its own housing should not be deciding it.

Converted once from an MJCF and committed beside this file, the same way the G1 and the room are,
because the tests describe robots the way a blueprint does and a description is USD. It has to come
through the real converter rather than be written by hand or converted from a URDF: a servo needs an
actuator, and only mujoco-usd-converter writes one (as an MjcActuator prim). All primitives, so it
names no mesh files at all.

    MuJoCo: bodies 3, joints 2 (free + neck), actuators 1, geoms 2 (a 0.2x0.1x0.1 box, a
            0.05 sphere)
    dimos:  root base, joint neck, frames base / head / neck / toy
"""

from pathlib import Path

from dimos.robot.assets.model import UsdModel

TOY = Path(__file__).parent / "toy.usda"


def toy_robot() -> UsdModel:
    """A fresh model each time, so a test adding frames to it cannot change another's."""
    return UsdModel.from_file(TOY)
