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

from dimos.constants import STATE_DIR
from dimos.perception.spatial_perception import (
    SpatialConfig,
    _default_spatial_memory_dir,
)


def test_spatial_memory_defaults_to_state_dir(monkeypatch) -> None:
    monkeypatch.delenv("DIMOS_SPATIAL_MEMORY_DIR", raising=False)

    assert _default_spatial_memory_dir() == STATE_DIR / "spatial_memory"
    assert SpatialConfig().db_path == str(STATE_DIR / "spatial_memory" / "chromadb_data")


def test_spatial_memory_dir_honors_env(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("DIMOS_SPATIAL_MEMORY_DIR", str(tmp_path))

    assert _default_spatial_memory_dir() == tmp_path
