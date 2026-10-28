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

"""Recording reuses registered schemas without inspecting payload fields."""

from dimos_cdrgen.registry import schema
import pytest

from dimos.experimental.memory.cdr_schema import cdr_name, recording_schema
from dimos.messages.geometry_msgs.msg import PoseStamped


def test_registry_owner_and_complete_schema_are_reused():
    assert cdr_name(PoseStamped) == "geometry_msgs/msg/PoseStamped"
    assert recording_schema(PoseStamped) == {
        "name": "geometry_msgs/msg/PoseStamped",
        "data": schema("geometry_msgs/msg/PoseStamped"),
    }
    with pytest.raises(TypeError, match="registry-owned"):
        recording_schema(bytes)
