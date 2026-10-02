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

"""TypeSafe policy in the HM3D example house (00861-GLAQ4DNUx5U), two goals.

The house ships with the Habitat install, so this needs no dataset download::

    dimos evals run dimos.evals.suites.typesafe_habitat_hm3d \\
        --agent dimos.evals.agents.typesafe_policy \\
        --set scene_json=dimos/evals/suites/scenes/habitat/00861-GLAQ4DNUx5U.json

Habitat runs on zenoh: export ``DIMOS_TRANSPORT=zenoh`` (the environment does
so for the simulator it launches; the policy runs in this process).
"""

from dimos.evals.suites.lib.typesafe_habitat import SCENES, habitat_suite
from dimos.evals.types import Suite

SCENE = SCENES / "00861-GLAQ4DNUx5U.json"

SUITE: Suite = habitat_suite(
    "typesafe_habitat_hm3d",
    SCENE,
    scene_id="00861-GLAQ4DNUx5U",
    # The benchmark's spawn for this house (PR 4216), on the ground floor.
    spawn_xyz=(-0.209, -0.059, -1.593),
    spawn_yaw_deg=0.0,
    # Words that name exactly one object of the house (it has four toilets).
    cases=(
        ("couch", "go to the couch"),
        ("dresser", "navigate to the dresser"),
    ),
)
