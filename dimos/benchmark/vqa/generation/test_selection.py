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

# Copyright 2026 Dimensional Inc.

from dimos.benchmark.vqa.contracts import GroundedObject
from dimos.benchmark.vqa.generation.primitives.selection import select_nearest_object


def test_select_nearest_object_optionally_restricts_to_image_side() -> None:
    objects = [
        GroundedObject("left", "chair", 3, 2.0, "left"),
        GroundedObject("right", "chair", 3, 1.0, "right"),
    ]

    assert select_nearest_object(objects).id == "right"
    assert select_nearest_object(objects, "left").id == "left"
    assert select_nearest_object(objects, "center") is None
