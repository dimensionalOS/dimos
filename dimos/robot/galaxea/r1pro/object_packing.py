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

"""Observation contract for a reusable selected-object ACT pick and place."""

from dimos.imitation.profile import PolicyIOProfile, VectorSource
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_IO
from dimos.robot.galaxea.r1pro.object_packing_scene import MAX_OBJECTS, SHAPES

OBJECT_GOAL_FEATURES = (
    *(f"selected_from_tcp_{axis}" for axis in "xyz"),
    *(f"destination_from_tcp_{axis}" for axis in "xyz"),
    *(f"selected_rotation_{i}" for i in range(9)),
    *(f"half_size_{axis}" for axis in "xyz"),
    *(f"shape_{shape}" for shape in SHAPES),
    *(f"home_from_tcp_{axis}" for axis in "xyz"),
    *(
        f"neighbor_{i}_{feature}"
        for i in range(MAX_OBJECTS - 1)
        for feature in ("present", "x", "y", "z", "extent_x", "extent_y", "extent_z")
    ),
)
OBJECT_PACKING_TASK = (
    "Grasp the selected rigid object and release it upright in the assigned free tray space."
)
OBJECT_PACKING_IO = PolicyIOProfile(
    name="r1pro-sim-object-packing-v1",
    robot_type="r1pro_sim_object_packing",
    observations={
        **R1PRO_PICK_PLACE_IO.observations,
        "observation.environment_state": VectorSource(
            stream="object_goal", features=OBJECT_GOAL_FEATURES
        ),
    },
    action=R1PRO_PICK_PLACE_IO.action,
    sync=R1PRO_PICK_PLACE_IO.sync,
    quality=R1PRO_PICK_PLACE_IO.quality,
)
