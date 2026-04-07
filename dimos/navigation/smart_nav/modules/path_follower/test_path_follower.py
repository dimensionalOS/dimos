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

"""Tests for PathFollower NativeModule wrapper."""

from pathlib import Path

import pytest

from dimos.navigation.smart_nav.modules.path_follower.path_follower import (
    PathFollower,
    PathFollowerConfig,
)


class TestPathFollowerConfig:
    """Test PathFollower configuration."""

    def test_default_config(self):
        config = PathFollowerConfig()
        assert config.look_ahead_distance == 0.5
        assert config.max_speed == 2.0
        assert config.max_yaw_rate == 80.0
        assert config.goal_tolerance == 0.3

    def test_cli_args_generation(self):
        config = PathFollowerConfig(
            look_ahead_distance=1.0,
            max_speed=1.0,
        )
        args = config.to_cli_args()
        # Field names are remapped to the C++ binary's camelCase names.
        assert "--lookAheadDis" in args
        assert "--maxSpeed" in args


class TestPathFollowerModule:
    """Test PathFollower module declaration."""

    def test_ports_declared(self):
        from typing import get_origin, get_type_hints

        from dimos.core.stream import In, Out

        hints = get_type_hints(PathFollower)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}

        assert "path" in in_ports
        assert "odometry" in in_ports
        assert "cmd_vel" in out_ports


@pytest.mark.skipif(
    not Path(__file__).resolve().parent.joinpath("result", "bin").exists(),
    reason="Native binary not built (run nix build first)",
)
class TestPathResolution:
    """Verify native module paths resolve to real filesystem locations."""

    def _make(self):
        m = PathFollower()
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
