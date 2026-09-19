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

"""TypeSafe policy in HSSD scene 102344193, two goals.

Needs the HSSD dataset under ``target/habitat/data/hssd-hab`` (in Docker, the
``compose.habitat-data.yaml`` mount); the scene file is PR 4211's ground truth::

    dimos evals run dimos.evals.suites.typesafe_habitat_hssd \\
        --agent dimos.evals.agents.typesafe_policy \\
        --set scene_json=dimos/evals/suites/scenes/habitat/102344193.json

Habitat runs on zenoh: export ``DIMOS_TRANSPORT=zenoh``.
"""

from dimos.evals.suites.lib.typesafe_habitat import HABITAT_DATA, SCENES, habitat_suite
from dimos.evals.types import Suite

SCENE = SCENES / "102344193.json"
DATASET = HABITAT_DATA / "hssd-hab" / "hssd-hab.scene_dataset_config.json"

SUITE: Suite = habitat_suite(
    "typesafe_habitat_hssd",
    SCENE,
    scene_id="102344193",
    # The benchmark's spawn for this scene (PR 4216).
    spawn_xyz=(5.404, 3.072, 0.124),
    spawn_yaw_deg=0.0,
    cases=(
        ("couch", "go to the couch"),
        ("toilet", "navigate to the toilet"),
    ),
    scene_dataset_config=str(DATASET),
)
