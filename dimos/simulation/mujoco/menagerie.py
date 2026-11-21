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

"""Resolve the pinned MuJoCo Menagerie used by the Unitree simulator."""

import importlib.util
from pathlib import Path

from dimos.robot.assets.git_cache import GitAssetCache

SIM_INSTALL_HINT = "Simulation dependencies are not installed. Run `uv sync --extra sim --inexact` to install them."
_MENAGERIE_COMMIT = "1b86ece576591213e2b666ebf59508454200ca97"


def menagerie_path() -> Path:
    """Reuse an existing Playground checkout or resolve the models in the DimOS cache."""
    spec = importlib.util.find_spec("mujoco_playground")
    if spec and spec.submodule_search_locations:
        for location in spec.submodule_search_locations:
            legacy = Path(location) / "external_deps" / "mujoco_menagerie"
            head = legacy / ".git" / "HEAD"
            if head.is_file() and head.read_text().strip() == _MENAGERIE_COMMIT:
                return legacy

    return GitAssetCache().resolve(
        "https://github.com/google-deepmind/mujoco_menagerie.git", _MENAGERIE_COMMIT
    )
