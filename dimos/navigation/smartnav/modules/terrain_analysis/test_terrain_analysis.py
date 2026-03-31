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

"""Tests for TerrainAnalysis NativeModule wrapper."""

from pathlib import Path

import pytest

from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import (
    TerrainAnalysis,
    TerrainAnalysisConfig,
)


class TestTerrainAnalysisConfig:
    """Test TerrainAnalysis configuration."""

    def test_default_config(self):
        """Default config should have sensible values."""
        config = TerrainAnalysisConfig()
        assert config.obstacle_height_threshold == 0.15
        assert config.voxel_size == 0.05
        assert config.sensor_range == 20.0

    def test_cli_args_generation(self):
        """Config should generate CLI args for the native binary."""
        config = TerrainAnalysisConfig(
            obstacle_height_threshold=0.2,
            voxel_size=0.1,
        )
        args = config.to_cli_args()
        assert "--obstacle_height_threshold" in args
        assert "0.2" in args
        assert "--voxel_size" in args
        assert "0.1" in args


class TestTerrainAnalysisModule:
    """Test TerrainAnalysis module declaration."""

    def test_ports_declared(self):
        """Module should declare the expected In/Out ports."""
        from typing import get_origin, get_type_hints

        from dimos.core.stream import In, Out

        hints = get_type_hints(TerrainAnalysis)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}

        assert "registered_scan" in in_ports
        assert "odometry" in in_ports
        assert "terrain_map" in out_ports


@pytest.mark.skipif(
    not Path(__file__).resolve().parent.joinpath("result", "bin").exists(),
    reason="Native binary not built (run nix build first)",
)
class TestPathResolution:
    """Verify native module paths resolve to real filesystem locations."""

    def _make(self):
        m = TerrainAnalysis()
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
