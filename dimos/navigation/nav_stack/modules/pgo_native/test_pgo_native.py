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

"""Unit tests for PGONative module config, ports, and CLI arg generation."""

from __future__ import annotations

from pathlib import Path

from dimos.navigation.nav_stack.modules.pgo_native.pgo_native import (
    PGONative,
    PGONativeConfig,
)


class TestPGONativeConfig:
    def test_defaults_match_original(self) -> None:
        config = PGONativeConfig()
        # Keyframe detection
        assert config.key_pose_delta_deg == 10.0
        assert config.key_pose_delta_trans == 0.5
        # Loop closure (match pgo_unity_sim.yaml)
        assert config.loop_search_radius == 1.0
        assert config.loop_time_thresh == 60.0
        assert config.loop_score_thresh == 0.15
        assert config.loop_submap_half_range == 5
        assert config.submap_resolution == 0.1
        assert config.min_loop_detect_duration == 5.0
        # Global map (match original hardcoded values)
        assert config.global_map_voxel_size == 0.1
        assert config.global_map_publish_rate == 1.0
        # Frame names
        assert config.world_frame == "map"
        assert config.local_frame == "odom"
        # Input mode
        assert config.unregister_input is True

    def test_cli_args_contain_all_config_fields(self) -> None:
        config = PGONativeConfig()
        args = config.to_cli_args()
        args_str = " ".join(args)
        expected_fields = [
            "key_pose_delta_deg",
            "key_pose_delta_trans",
            "loop_search_radius",
            "loop_time_thresh",
            "loop_score_thresh",
            "loop_submap_half_range",
            "submap_resolution",
            "min_loop_detect_duration",
            "global_map_voxel_size",
            "global_map_publish_rate",
            "world_frame",
            "local_frame",
            "unregister_input",
        ]
        for field in expected_fields:
            assert f"--{field}" in args_str, f"Missing CLI arg: --{field}"

    def test_cli_args_values(self) -> None:
        config = PGONativeConfig(
            loop_search_radius=2.0,
            loop_score_thresh=0.1,
        )
        args = config.to_cli_args()
        pairs = dict(zip(args[::2], args[1::2], strict=False))
        assert pairs["--loop_search_radius"] == "2.0"
        assert pairs["--loop_score_thresh"] == "0.1"

    def test_executable_path(self) -> None:
        config = PGONativeConfig()
        assert config.executable == "result/bin/pgo_native"
        assert config.cwd is not None
        cwd_path = Path(config.cwd)
        assert cwd_path.name == "cpp"

    def test_build_command(self) -> None:
        config = PGONativeConfig()
        assert config.build_command == "nix build .#default --no-write-lock-file"


class TestPGONativePorts:
    def test_input_ports(self) -> None:
        annotations = PGONative.__annotations__
        assert "registered_scan" in annotations
        assert "odometry" in annotations

    def test_output_ports(self) -> None:
        annotations = PGONative.__annotations__
        assert "corrected_odometry" in annotations
        assert "global_map" in annotations
        assert "pgo_tf" in annotations
