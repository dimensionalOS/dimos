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

"""Regression tests for published smart-nav native module refs."""

from pathlib import Path

from dimos.navigation.smart_nav.modules.arise_slam.arise_slam import AriseSLAMConfig
from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlannerConfig
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlannerConfig
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollowerConfig
from dimos.navigation.smart_nav.modules.tare_planner.tare_planner import TarePlannerConfig
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import (
    TerrainAnalysisConfig,
)

LATEST_MODULE_REFS = {
    "arise-slam": "github:dimensionalOS/dimos-module-arise-slam/v0.2.0",
    "far-planner": "github:dimensionalOS/dimos-module-far-planner/v0.5.0",
    "local-planner": "github:dimensionalOS/dimos-module-local-planner/v0.6.0",
    "path-follower": "github:dimensionalOS/dimos-module-path-follower/v0.2.0",
    "tare-planner": "github:dimensionalOS/dimos-module-tare-planner/v0.1.0",
    "terrain-analysis": "github:dimensionalOS/dimos-module-terrain-analysis/v0.1.1",
}


def test_python_native_module_defaults_use_latest_refs() -> None:
    configs = {
        "arise-slam": AriseSLAMConfig(),
        "far-planner": FarPlannerConfig(),
        "local-planner": LocalPlannerConfig(),
        "path-follower": PathFollowerConfig(),
        "tare-planner": TarePlannerConfig(),
        "terrain-analysis": TerrainAnalysisConfig(),
    }

    for module, expected_ref in LATEST_MODULE_REFS.items():
        assert configs[module].build_command is not None
        assert expected_ref in configs[module].build_command


def test_m20_nix_wrappers_use_latest_refs() -> None:
    root = Path(__file__).resolve().parents[2]
    wrapper_root = root / "robot" / "deeprobotics" / "m20" / "nix_wrappers"
    wrappers = {
        "arise-slam": wrapper_root / "arise_slam" / "flake.nix",
        "far-planner": wrapper_root / "far_planner" / "flake.nix",
        "local-planner": wrapper_root / "local_planner" / "flake.nix",
        "path-follower": wrapper_root / "path_follower" / "flake.nix",
        "tare-planner": wrapper_root / "tare_planner" / "flake.nix",
        "terrain-analysis": wrapper_root / "terrain_analysis" / "flake.nix",
    }

    for module, flake_path in wrappers.items():
        assert LATEST_MODULE_REFS[module] in flake_path.read_text()
