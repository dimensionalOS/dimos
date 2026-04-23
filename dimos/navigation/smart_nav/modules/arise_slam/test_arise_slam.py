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

"""Tests for AriseSlam NativeModule wrapper."""

from pathlib import Path
from typing import get_origin, get_type_hints

from dimos.core.stream import In, Out
from dimos.navigation.smart_nav.modules.arise_slam.arise_slam import (
    AriseSlam,
    AriseSlamConfig,
)


class TestAriseSlamConfig:
    """Test AriseSlam configuration."""

    def test_default_config(self):
        config = AriseSlamConfig()
        # Feature extraction defaults (upstream livox_mid360.yaml)
        assert config.scan_line == 4
        assert config.sensor == "livox"
        assert config.min_range == 0.1
        assert config.max_range == 130.0
        assert config.provide_point_time == 1
        assert config.skip_frame == 1
        assert config.use_imu_roll_pitch is False
        # Laser mapping defaults
        assert config.mapping_line_resolution == 0.1
        assert config.mapping_plane_resolution == 0.2
        assert config.max_iterations == 5
        assert config.max_surface_features == 2000
        assert config.velocity_failure_threshold == 30.0
        assert config.auto_voxel_size is True
        assert config.pos_degeneracy_threshold == 1.0
        assert config.ori_degeneracy_threshold == 1.0
        assert config.shift_undistortion is True
        # IMU preintegration defaults
        assert config.acc_n == 0.3994
        assert config.gyr_n == 0.001564
        assert config.acc_w == 0.006436
        assert config.gyr_w == 0.0000356
        assert config.g_norm == 9.80511
        assert config.lidar_correction_noise == 0.01
        assert config.smooth_factor == 0.9
        # IMU bias offsets
        assert config.imu_acc_x_offset == 0.04
        assert config.imu_acc_y_offset == 0.0
        assert config.imu_acc_z_offset == 0.0
        assert config.imu_acc_x_limit == 1.0
        assert config.imu_acc_y_limit == 1.0
        assert config.imu_acc_z_limit == 1.0
        # Blind zone defaults
        assert config.blind_front == 0.2
        assert config.blind_back == -0.2
        assert config.blind_left == 0.3
        assert config.blind_right == -0.3
        assert config.blind_disk_radius == 0.5

    def test_cli_args_generation(self):
        config = AriseSlamConfig(
            scan_line=16,
            sensor="velodyne",
            max_iterations=10,
            auto_voxel_size=False,
        )
        args = config.to_cli_args()
        assert "--scan_line" in args
        assert "16" in args
        assert "--sensor" in args
        assert "velodyne" in args
        assert "--max_iterations" in args
        assert "10" in args
        assert "--auto_voxel_size" in args
        assert "false" in args

    def test_every_config_field_has_cli_arg(self):
        """Every declared config field must appear as a --flag on the CLI."""
        config = AriseSlamConfig()
        args = config.to_cli_args()
        parent_fields = set(AriseSlamConfig.__mro__[1].model_fields.keys())
        for name in AriseSlamConfig.model_fields:
            if name in parent_fields:
                continue
            val = getattr(config, name)
            if val is None:
                continue  # Optional fields with None default are excluded
            assert f"--{name}" in args, f"Missing CLI arg for config field: {name}"

    def test_build_config(self):
        """Build configuration should point to the cpp subdirectory."""
        config = AriseSlamConfig()
        assert config.cwd is not None
        assert config.executable == "result/bin/arise_slam"
        assert config.build_command == "nix build .#default --no-write-lock-file"


class TestAriseSlamModule:
    """Test AriseSlam module declaration."""

    def test_ports_declared(self):
        hints = get_type_hints(AriseSlam)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}

        assert "lidar" in in_ports
        assert "imu" in in_ports
        assert "fallback_odometry" in in_ports
        assert "odometry" in out_ports
        assert "registered_scan" in out_ports
        assert "surround_map" in out_ports
        assert "global_map" in out_ports
        assert "laser_odometry" in out_ports
        assert "incremental_odometry" in out_ports
        assert "slam_stats" in out_ports

    def test_no_extra_out_ports(self):
        """AriseSlam should have exactly 7 output ports."""
        hints = get_type_hints(AriseSlam)
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}
        assert len(out_ports) == 7, f"Expected 7 out ports, got {out_ports}"

    def test_no_extra_in_ports(self):
        """AriseSlam should have exactly 3 input ports."""
        hints = get_type_hints(AriseSlam)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        assert len(in_ports) == 3, f"Expected 3 in ports, got {in_ports}"

    def test_executable_path_is_relative_to_module(self):
        """Config cwd/executable resolves against the module's own directory."""
        config = AriseSlamConfig()
        assert config.cwd is not None
        cwd = Path(config.cwd)
        assert cwd.is_dir(), f"Module cwd must exist: {cwd}"
        assert cwd.name == "cpp"
        assert not Path(config.executable).is_absolute()
        assert config.executable.endswith("arise_slam")
