# Copyright 2025-2026 Dimensional Inc.
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

import pytest

from dimos.simulation.dimsim.scene_client import SceneClient


def test_semantic_object_bounds_rejects_missing_browser_object(mocker) -> None:
    client = SceneClient()
    mocker.patch.object(client, "exec", return_value=None)

    with pytest.raises(LookupError, match="'queen size bed'"):
        client.get_semantic_object_bounds("queen size bed")


def test_semantic_object_bounds_rejects_empty_query() -> None:
    client = SceneClient()

    with pytest.raises(ValueError, match="must not be empty"):
        client.get_semantic_object_bounds("  ")
