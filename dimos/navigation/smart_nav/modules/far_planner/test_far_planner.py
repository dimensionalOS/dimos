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

"""Tests for FarPlanner NativeModule wrapper."""

from pathlib import Path

import pytest

from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlanner, FarPlannerConfig


class TestFarPlannerConfig:
    """Test FarPlanner configuration."""

    def test_default_config(self):
        config = FarPlannerConfig()
        assert config.update_rate == 5.0
        assert config.robot_dimension == 0.5
        assert config.sensor_range == 30.0
        assert config.voxel_dim == 0.1
        assert config.terrain_range == 7.5
        assert config.is_static_env is True

    def test_cli_args_generation(self):
        config = FarPlannerConfig(
            sensor_range=20.0,
            robot_dimension=0.8,
            is_static_env=True,
        )
        args = config.to_cli_args()
        assert "--sensor_range" in args
        assert "20.0" in args
        assert "--robot_dim" in args  # cli_name_override maps robot_dimension -> robot_dim
        assert "0.8" in args
        assert "--is_static_env" in args
        assert "true" in args

    def test_all_config_fields_generate_cli_args(self):
        """Every non-NativeModuleConfig field should produce a CLI arg."""
        config = FarPlannerConfig()
        args = config.to_cli_args()
        for expected in [
            "--update_rate",
            "--voxel_dim",
            "--terrain_range",
            "--floor_height",
            "--converge_dist",
            "--angle_noise",
        ]:
            assert expected in args, f"Missing CLI arg: {expected}"


class TestFarPlannerModule:
    """Test FarPlanner module declaration."""

    def test_ports_declared(self):
        from typing import get_origin, get_type_hints

        from dimos.core.stream import In, Out

        hints = get_type_hints(FarPlanner)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}

        assert "terrain_map_ext" in in_ports
        assert "terrain_map" in in_ports
        assert "registered_scan" in in_ports
        assert "odometry" in in_ports
        assert "goal" in in_ports
        assert "way_point" in out_ports
        assert "goal_path" in out_ports


@pytest.mark.skipif(
    not Path(__file__).resolve().parent.joinpath("repo", "result", "bin").exists(),
    reason="Native binary not built (run nix build first)",
)
class TestPathResolution:
    """Verify native module paths resolve to real filesystem locations."""

    def _make(self):
        return FarPlanner()

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
