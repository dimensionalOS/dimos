"""Tests for PCTPlanner NativeModule wrapper."""

from pathlib import Path

import pytest

from dimos.navigation.smart_nav.modules.pct_planner.pct_planner import (
    PCTPlanner,
    PCTPlannerConfig,
)


class TestPCTPlannerConfig:
    """Test PCTPlanner configuration."""

    def test_default_config(self):
        config = PCTPlannerConfig()
        assert config.resolution == 0.075
        assert config.slice_dh == 0.4
        assert config.slope_max == 0.45
        assert config.step_max == 0.5
        assert config.lookahead_distance == 1.25
        assert config.cost_barrier == 100.0
        assert config.kernel_size == 11

    def test_cli_args_generation(self):
        config = PCTPlannerConfig(
            resolution=0.1,
            slice_dh=0.5,
            lookahead_distance=2.0,
        )
        args = config.to_cli_args()
        assert "--resolution" in args
        assert "0.1" in args
        assert "--slice_dh" in args
        assert "0.5" in args
        assert "--lookahead_distance" in args
        assert "2.0" in args

    def test_all_config_fields_generate_cli_args(self):
        config = PCTPlannerConfig()
        args = config.to_cli_args()
        for expected in [
            "--update_rate",
            "--resolution",
            "--slice_dh",
            "--slope_max",
            "--step_max",
            "--cost_barrier",
            "--kernel_size",
            "--safe_margin",
            "--inflation",
            "--lookahead_distance",
        ]:
            assert expected in args, f"Missing CLI arg: {expected}"


class TestPCTPlannerModule:
    """Test PCTPlanner module declaration."""

    def test_ports_declared(self):
        from typing import get_origin, get_type_hints

        from dimos.core.stream import In, Out

        hints = get_type_hints(PCTPlanner)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}

        assert "explored_areas" in in_ports
        assert "odometry" in in_ports
        assert "goal" in in_ports
        assert "way_point" in out_ports
        assert "goal_path" in out_ports
        assert "tomogram" in out_ports


@pytest.mark.skipif(
    not Path(__file__).resolve().parent.joinpath("result", "bin").exists(),
    reason="Native binary not built (run nix build first)",
)
class TestPathResolution:
    """Verify native module paths resolve to real filesystem locations."""

    def _make(self):
        m = PCTPlanner()
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
