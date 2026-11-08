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


"""The office_555 room, converted once from its MJCF and committed beside this file."""

from pathlib import Path

from dimos.robot.assets.model import UsdModel

SCENE = Path(__file__).parent / "scene.usda"
OFFICE_555 = UsdModel.from_file(SCENE)

# 5.74 m of clear floor, the widest in the room: measured by taking every geom below 2 m with
# its bounding radius and finding the point furthest from all of them. Where a robot can stand
# without being inside the furniture.
CLEAR_FLOOR = (-0.85, -7.66)
