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

"""habitat_nav for the planner arm: the goal coordinates are the navigable point beside the object.

An object's centre is inside the object; the MLS planner snaps a goal within 1.5 m of its
surface, which fails for large or tucked-in objects. The case picker's end point (``end_nav_xy``)
is on the navmesh, so the planner ceiling is measured on every case. Grading is unchanged (the
object's box). Cases without ``end_nav_xy`` fall back to the centre.
"""

from dimos.evals.suites.habitat_nav import SCENES, cases_for
from dimos.evals.types import Suite

SUITE: Suite = [
    case for f in sorted(SCENES.glob("*.json")) for case in cases_for(f, goal_key="end_nav_xy")
]
