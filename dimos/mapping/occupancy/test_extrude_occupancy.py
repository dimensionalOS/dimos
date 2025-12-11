# Copyright 2025 Dimensional Inc.
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

from dimos.mapping.occupancy.extrude_occupancy import generate_mujoco_scene
from dimos.utils.data import get_data


def test_generate_mujoco_scene(occupancy) -> None:
    with open(get_data("expected_occupancy_scene.xml")) as f:
        expected = f.read()

    actual = generate_mujoco_scene(occupancy)

    assert actual == expected
