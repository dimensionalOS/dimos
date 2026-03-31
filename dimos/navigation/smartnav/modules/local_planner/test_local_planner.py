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

"""Tests for LocalPlanner NativeModule wrapper."""

from pathlib import Path

import pytest

from dimos.navigation.smartnav.modules.local_planner.local_planner import (
    LocalPlanner,
    LocalPlannerConfig,
)


class TestLocalPlannerConfig:
    """Test LocalPlanner configuration."""

    def test_default_config(self):
        config = LocalPlannerConfig()
        assert config.max_speed == 2.0
        assert config.autonomy_speed == 1.0
        assert config.vehicle_config == "omniDir"
        assert config.obstacle_height_threshold == 0.15

    def test_cli_args_generation(self):
        config = LocalPlannerConfig(
            max_speed=1.5,
            paths_dir="/custom/paths",
        )
        args = config.to_cli_args()
        assert "--max_speed" in args
        assert "1.5" in args
        assert "--paths_dir" in args
        assert "/custom/paths" in args


class TestLocalPlannerModule:
    """Test LocalPlanner module declaration."""

    def test_ports_declared(self):
        from typing import get_origin, get_type_hints

        from dimos.core.stream import In, Out

        hints = get_type_hints(LocalPlanner)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}

        assert "registered_scan" in in_ports
        assert "odometry" in in_ports
        assert "joy_cmd" in in_ports
        assert "way_point" in in_ports
        assert "path" in out_ports


@pytest.mark.skipif(
    not Path(__file__).resolve().parent.joinpath("result", "bin").exists(),
    reason="Native binary not built (run nix build first)",
)
class TestPathResolution:
    """Verify native module paths resolve to real filesystem locations."""

    def _make(self):
        m = LocalPlanner()
        m._resolve_paths()
        return m

    def test_cwd_resolves_to_existing_directory(self):
        m = self._make()
        try:
            assert Path(m.config.cwd).exists(), f"cwd does not exist: {m.config.cwd}"
            assert Path(m.config.cwd).is_dir()
        finally:
            m.stop()

    def test_executable_exists(self):
        m = self._make()
        try:
            exe = Path(m.config.executable)
            assert exe.exists(), f"Binary not found: {exe}. Run nix build first."
        finally:
            m.stop()

    def test_cwd_resolves_to_smartnav_root(self):
        """cwd should resolve to the smartnav root (where CMakeLists.txt lives)."""
        m = self._make()
        try:
            cwd = Path(m.config.cwd).resolve()
            assert (cwd / "CMakeLists.txt").exists(), f"cwd {cwd} is not the smartnav root"
            assert (cwd / "flake.nix").exists()
        finally:
            m.stop()

    def test_data_files_exist(self):
        """Local planner needs path data files (pulled from LFS)."""
        from dimos.utils.data import get_data

        paths_dir = get_data("smartnav_paths")
        assert paths_dir.exists(), f"paths_dir not found: {paths_dir}"
        assert (paths_dir / "startPaths.ply").exists()
        assert (paths_dir / "pathList.ply").exists()
        assert (paths_dir / "paths.ply").exists()
