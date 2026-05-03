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

import importlib
import sys

import pytest

from dimos.navigation.cmd_vel_mux import CmdVelMux
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.fastlio2.fastlio2 import FastLio2
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.navigation.smart_nav.modules.simple_planner.simple_planner import SimplePlanner
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import (
    TerrainAnalysis,
)
from dimos.navigation.smart_nav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.robot.deeprobotics.m20.drdds_bridge.module import (
    AiryImuBridge,
    DrddsLidarBridge,
)

_MODULE = "dimos.robot.deeprobotics.m20.blueprints.nav.m20_smartnav_native"


@pytest.fixture(autouse=True)
def clean_imported_m20_native_blueprint():
    sys.modules.pop(_MODULE, None)
    yield
    sys.modules.pop(_MODULE, None)


def _load_m20_native(
    monkeypatch,
    *,
    max_yaw_rate: str | None = None,
    max_command_duration: str | None = None,
    omni_dir_goal_threshold: str | None = None,
    omni_dir_diff_threshold: str | None = None,
    yaw_rate_gain: str | None = None,
    stop_yaw_rate_gain: str | None = None,
    dir_diff_threshold: str | None = None,
    stop_distance_threshold: str | None = None,
    slow_down_distance_threshold: str | None = None,
    goal_reached_threshold: str | None = None,
    goal_behind_range: str | None = None,
    freeze_ang: str | None = None,
    robot_exclusion_radius: str | None = None,
    terrain_no_decay_distance: str | None = None,
    terrain_voxel_size: str | None = None,
    nav_use_pgo: str | None = None,
    nav_use_pgo_corrected_odometry: str | None = None,
    lidar_source: str | None = None,
    slam_cores: str | None = None,
    fastlio_cores: str | None = None,
    drdds_lidar_cores: str | None = None,
    airy_imu_cores: str | None = None,
):
    monkeypatch.setenv("M20_NAV_ENABLED", "1")
    if max_yaw_rate is None:
        monkeypatch.delenv("M20_NAV_MAX_YAW_RATE", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_MAX_YAW_RATE", max_yaw_rate)
    if max_command_duration is None:
        monkeypatch.delenv("M20_NAV_MAX_COMMAND_DURATION", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_MAX_COMMAND_DURATION", max_command_duration)
    if omni_dir_goal_threshold is None:
        monkeypatch.delenv("M20_NAV_OMNI_DIR_GOAL_THRESHOLD", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_OMNI_DIR_GOAL_THRESHOLD", omni_dir_goal_threshold)
    if omni_dir_diff_threshold is None:
        monkeypatch.delenv("M20_NAV_OMNI_DIR_DIFF_THRESHOLD", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_OMNI_DIR_DIFF_THRESHOLD", omni_dir_diff_threshold)
    if yaw_rate_gain is None:
        monkeypatch.delenv("M20_NAV_YAW_RATE_GAIN", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_YAW_RATE_GAIN", yaw_rate_gain)
    if stop_yaw_rate_gain is None:
        monkeypatch.delenv("M20_NAV_STOP_YAW_RATE_GAIN", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_STOP_YAW_RATE_GAIN", stop_yaw_rate_gain)
    if dir_diff_threshold is None:
        monkeypatch.delenv("M20_NAV_DIR_DIFF_THRESHOLD", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_DIR_DIFF_THRESHOLD", dir_diff_threshold)
    if stop_distance_threshold is None:
        monkeypatch.delenv("M20_NAV_STOP_DISTANCE_THRESHOLD", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_STOP_DISTANCE_THRESHOLD", stop_distance_threshold)
    if slow_down_distance_threshold is None:
        monkeypatch.delenv("M20_NAV_SLOW_DOWN_DISTANCE_THRESHOLD", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_SLOW_DOWN_DISTANCE_THRESHOLD", slow_down_distance_threshold)
    if goal_reached_threshold is None:
        monkeypatch.delenv("M20_NAV_GOAL_REACHED_THRESHOLD", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_GOAL_REACHED_THRESHOLD", goal_reached_threshold)
    if goal_behind_range is None:
        monkeypatch.delenv("M20_NAV_GOAL_BEHIND_RANGE", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_GOAL_BEHIND_RANGE", goal_behind_range)
    if freeze_ang is None:
        monkeypatch.delenv("M20_NAV_FREEZE_ANG", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_FREEZE_ANG", freeze_ang)
    if robot_exclusion_radius is None:
        monkeypatch.delenv("M20_NAV_ROBOT_EXCLUSION_RADIUS", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_ROBOT_EXCLUSION_RADIUS", robot_exclusion_radius)
    if terrain_no_decay_distance is None:
        monkeypatch.delenv("M20_TERRAIN_NO_DECAY_DISTANCE", raising=False)
    else:
        monkeypatch.setenv("M20_TERRAIN_NO_DECAY_DISTANCE", terrain_no_decay_distance)
    if terrain_voxel_size is None:
        monkeypatch.delenv("M20_TERRAIN_VOXEL_SIZE", raising=False)
    else:
        monkeypatch.setenv("M20_TERRAIN_VOXEL_SIZE", terrain_voxel_size)
    if nav_use_pgo is None:
        monkeypatch.delenv("M20_NAV_USE_PGO", raising=False)
    else:
        monkeypatch.setenv("M20_NAV_USE_PGO", nav_use_pgo)
    if nav_use_pgo_corrected_odometry is None:
        monkeypatch.delenv("M20_NAV_USE_PGO_CORRECTED_ODOMETRY", raising=False)
    else:
        monkeypatch.setenv(
            "M20_NAV_USE_PGO_CORRECTED_ODOMETRY",
            nav_use_pgo_corrected_odometry,
        )
    if lidar_source is None:
        monkeypatch.delenv("M20_LIDAR_SOURCE", raising=False)
    else:
        monkeypatch.setenv("M20_LIDAR_SOURCE", lidar_source)
    if slam_cores is None:
        monkeypatch.delenv("M20_SLAM_CORES", raising=False)
    else:
        monkeypatch.setenv("M20_SLAM_CORES", slam_cores)
    if fastlio_cores is None:
        monkeypatch.delenv("M20_FASTLIO_CORES", raising=False)
    else:
        monkeypatch.setenv("M20_FASTLIO_CORES", fastlio_cores)
    if drdds_lidar_cores is None:
        monkeypatch.delenv("M20_DRDDS_LIDAR_CORES", raising=False)
    else:
        monkeypatch.setenv("M20_DRDDS_LIDAR_CORES", drdds_lidar_cores)
    if airy_imu_cores is None:
        monkeypatch.delenv("M20_AIRY_IMU_CORES", raising=False)
    else:
        monkeypatch.setenv("M20_AIRY_IMU_CORES", airy_imu_cores)

    return importlib.import_module(_MODULE)


def _cmd_vel_mux_kwargs(module) -> dict:
    muxes = [
        atom.kwargs for atom in module.m20_smartnav_native.blueprints if atom.module is CmdVelMux
    ]
    assert len(muxes) == 1
    return muxes[0]


def _local_planner_kwargs(module) -> dict:
    local_planners = [
        atom.kwargs for atom in module.m20_smartnav_native.blueprints if atom.module is LocalPlanner
    ]
    assert len(local_planners) == 1
    return local_planners[0]


def _path_follower_kwargs(module) -> dict:
    path_followers = [
        atom.kwargs for atom in module.m20_smartnav_native.blueprints if atom.module is PathFollower
    ]
    assert len(path_followers) == 1
    return path_followers[0]


def _simple_planner_kwargs(module) -> dict:
    simple_planners = [
        atom.kwargs
        for atom in module.m20_smartnav_native.blueprints
        if atom.module is SimplePlanner
    ]
    assert len(simple_planners) == 1
    return simple_planners[0]


def _terrain_analysis_kwargs(module) -> dict:
    terrain_analyses = [
        atom.kwargs
        for atom in module.m20_smartnav_native.blueprints
        if atom.module is TerrainAnalysis
    ]
    assert len(terrain_analyses) == 1
    return terrain_analyses[0]


def _terrain_map_ext_kwargs(module) -> dict:
    terrain_map_exts = [
        atom.kwargs
        for atom in module.m20_smartnav_native.blueprints
        if atom.module is TerrainMapExt
    ]
    assert len(terrain_map_exts) == 1
    return terrain_map_exts[0]


def _native_kwargs(module, native_cls) -> dict:
    matches = [
        atom.kwargs for atom in module.m20_smartnav_native.blueprints if atom.module is native_cls
    ]
    assert len(matches) == 1
    return matches[0]


def _module_count(module, module_cls) -> int:
    return sum(1 for atom in module.m20_smartnav_native.blueprints if atom.module is module_cls)


def test_m20_blueprint_caps_path_follower_yaw_rate_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _path_follower_kwargs(module)["max_yaw_rate"] == 40.0


def test_m20_blueprint_can_disable_pgo_nav_path(monkeypatch):
    module = _load_m20_native(monkeypatch, nav_use_pgo="0")
    remappings = module.m20_smartnav_native.remapping_map

    assert _module_count(module, PGO) == 0
    assert (SimplePlanner, "odometry") not in remappings
    assert (ClickToGoal, "odometry") not in remappings
    assert (TerrainAnalysis, "odometry") not in remappings


def test_m20_blueprint_can_keep_pgo_for_viz_without_corrected_nav(monkeypatch):
    module = _load_m20_native(
        monkeypatch,
        nav_use_pgo="1",
        nav_use_pgo_corrected_odometry="0",
    )
    remappings = module.m20_smartnav_native.remapping_map

    assert _module_count(module, PGO) == 1
    assert remappings[(PGO, "global_map")] == "global_map_pgo"
    assert (SimplePlanner, "odometry") not in remappings
    assert (ClickToGoal, "odometry") not in remappings
    assert (TerrainAnalysis, "odometry") not in remappings


def test_m20_blueprint_accepts_path_follower_yaw_rate_env_override(monkeypatch):
    module = _load_m20_native(monkeypatch, max_yaw_rate="20.0")

    assert _path_follower_kwargs(module)["max_yaw_rate"] == 20.0


def test_m20_blueprint_uses_long_nav_command_window_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _cmd_vel_mux_kwargs(module)["max_nav_command_duration_sec"] == 180.0


def test_m20_blueprint_accepts_nav_command_window_env_override(monkeypatch):
    module = _load_m20_native(monkeypatch, max_command_duration="75.0")

    assert _cmd_vel_mux_kwargs(module)["max_nav_command_duration_sec"] == 75.0


def test_m20_blueprint_uses_close_goal_omni_mode_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _path_follower_kwargs(module)["omni_dir_goal_threshold"] == 2.0
    assert _path_follower_kwargs(module)["omni_dir_diff_threshold"] == 3.2


def test_m20_blueprint_accepts_omni_goal_threshold_env_override(monkeypatch):
    module = _load_m20_native(
        monkeypatch,
        omni_dir_goal_threshold="0.5",
        omni_dir_diff_threshold="1.0",
    )

    assert _path_follower_kwargs(module)["omni_dir_goal_threshold"] == 0.5
    assert _path_follower_kwargs(module)["omni_dir_diff_threshold"] == 1.0


def test_m20_blueprint_uses_m20_path_follower_gains_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    path_follower = _path_follower_kwargs(module)
    assert path_follower["yaw_rate_gain"] == 1.5
    assert path_follower["stop_yaw_rate_gain"] == 1.5
    assert path_follower["direction_difference_threshold"] == 0.4
    assert path_follower["stop_distance_threshold"] == 0.3
    assert path_follower["slow_down_distance_threshold"] == 0.875


def test_m20_blueprint_keeps_path_follower_stop_distance_at_goal_threshold(monkeypatch):
    module = _load_m20_native(monkeypatch, goal_reached_threshold="0.35")

    path_follower = _path_follower_kwargs(module)
    assert path_follower["stop_distance_threshold"] == 0.35


def test_m20_blueprint_accepts_path_follower_gain_env_overrides(monkeypatch):
    module = _load_m20_native(
        monkeypatch,
        yaw_rate_gain="2.0",
        stop_yaw_rate_gain="2.5",
        dir_diff_threshold="0.25",
        stop_distance_threshold="0.35",
        slow_down_distance_threshold="0.7",
    )

    path_follower = _path_follower_kwargs(module)
    assert path_follower["yaw_rate_gain"] == 2.0
    assert path_follower["stop_yaw_rate_gain"] == 2.5
    assert path_follower["direction_difference_threshold"] == 0.25
    assert path_follower["stop_distance_threshold"] == 0.35
    assert path_follower["slow_down_distance_threshold"] == 0.7


def test_m20_blueprint_passes_local_planner_arrival_thresholds(monkeypatch):
    module = _load_m20_native(monkeypatch)

    local_planner = _local_planner_kwargs(module)
    simple_planner = _simple_planner_kwargs(module)
    assert local_planner["goal_reached_threshold"] == 0.3
    assert local_planner["goal_behind_range"] == 0.3
    assert local_planner["freeze_ang"] == 180.0
    assert local_planner["two_way_drive"] is False
    assert simple_planner["goal_reached_threshold"] == 0.3


def test_m20_blueprint_accepts_local_planner_arrival_env_overrides(monkeypatch):
    module = _load_m20_native(
        monkeypatch,
        goal_reached_threshold="0.25",
        goal_behind_range="0.2",
        freeze_ang="135.0",
    )

    local_planner = _local_planner_kwargs(module)
    simple_planner = _simple_planner_kwargs(module)
    assert local_planner["goal_reached_threshold"] == 0.25
    assert local_planner["goal_behind_range"] == 0.2
    assert local_planner["freeze_ang"] == 135.0
    assert simple_planner["goal_reached_threshold"] == 0.25


def test_m20_blueprint_configures_robot_exclusion_radius(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _simple_planner_kwargs(module)["robot_exclusion_radius"] == 0.0
    assert _terrain_map_ext_kwargs(module)["robot_exclusion_radius"] == 0.0


def test_m20_blueprint_accepts_robot_exclusion_radius_env_override(monkeypatch):
    module = _load_m20_native(monkeypatch, robot_exclusion_radius="0.8")

    assert _simple_planner_kwargs(module)["robot_exclusion_radius"] == 0.8
    assert _terrain_map_ext_kwargs(module)["robot_exclusion_radius"] == 0.8


def test_m20_blueprint_decays_near_robot_terrain_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _terrain_analysis_kwargs(module)["no_decay_distance"] == 0.0


def test_m20_blueprint_uses_one_meter_terrain_voxels_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    terrain = _terrain_analysis_kwargs(module)
    assert terrain["terrain_voxel_size"] == 1.0
    assert terrain["terrain_voxel_half_width"] == 10


def test_m20_blueprint_accepts_terrain_voxel_size_env_override(monkeypatch):
    module = _load_m20_native(monkeypatch, terrain_voxel_size="0.5")

    terrain = _terrain_analysis_kwargs(module)
    assert terrain["terrain_voxel_size"] == 0.5
    assert terrain["terrain_voxel_half_width"] == 20


def test_m20_blueprint_accepts_terrain_no_decay_distance_env_override(monkeypatch):
    module = _load_m20_native(monkeypatch, terrain_no_decay_distance="0.4")

    assert _terrain_analysis_kwargs(module)["no_decay_distance"] == 0.4


def test_m20_blueprint_uses_front_lidar_source_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _native_kwargs(module, DrddsLidarBridge)["lidar_source"] == "front"


def test_m20_blueprint_accepts_rear_lidar_source_env_override(monkeypatch):
    module = _load_m20_native(monkeypatch, lidar_source="rear")

    assert _native_kwargs(module, DrddsLidarBridge)["lidar_source"] == "rear"


def test_m20_blueprint_pairs_rear_lidar_with_rear_airy_imu(monkeypatch):
    monkeypatch.setenv("M20_SLAM_BACKEND", "fastlio2")
    monkeypatch.setenv("M20_FASTLIO2_IMU", "airy")
    module = _load_m20_native(monkeypatch, lidar_source="rear")

    assert _native_kwargs(module, AiryImuBridge)["which"] == "rear"
    assert module.m20_smartnav_native.remapping_map[(FastLio2, "imu")] == "airy_imu_rear"


def test_m20_blueprint_rejects_invalid_lidar_source(monkeypatch):
    with pytest.raises(ValueError, match="M20_LIDAR_SOURCE"):
        _load_m20_native(monkeypatch, lidar_source="side")


def test_m20_blueprint_uses_slam_cpu_affinity_for_native_path(monkeypatch):
    monkeypatch.setenv("M20_SLAM_BACKEND", "fastlio2")
    monkeypatch.setenv("M20_FASTLIO2_IMU", "airy")
    module = _load_m20_native(monkeypatch, slam_cores="4,5")

    assert _native_kwargs(module, FastLio2)["cpu_affinity"] == frozenset({4, 5})
    assert _native_kwargs(module, DrddsLidarBridge)["cpu_affinity"] == frozenset({4, 5})
    assert _native_kwargs(module, AiryImuBridge)["cpu_affinity"] == frozenset({4, 5})


def test_m20_blueprint_accepts_split_native_cpu_affinity(monkeypatch):
    monkeypatch.setenv("M20_SLAM_BACKEND", "fastlio2")
    monkeypatch.setenv("M20_FASTLIO2_IMU", "airy")
    module = _load_m20_native(
        monkeypatch,
        slam_cores="4,5,6,7",
        fastlio_cores="4",
        drdds_lidar_cores="4",
        airy_imu_cores="6",
    )

    assert _native_kwargs(module, FastLio2)["cpu_affinity"] == frozenset({4})
    assert _native_kwargs(module, DrddsLidarBridge)["cpu_affinity"] == frozenset({4})
    assert _native_kwargs(module, AiryImuBridge)["cpu_affinity"] == frozenset({6})


def test_m20_click_goals_always_flow_through_simple_planner(monkeypatch):
    monkeypatch.setenv("M20_DIRECT_CLICK_WAYPOINT", "1")

    module = _load_m20_native(monkeypatch)
    remappings = module.m20_smartnav_native.remapping_map

    assert remappings[(ClickToGoal, "way_point")] == "_click_way_point_unused"
    assert (SimplePlanner, "goal") not in remappings
    assert (SimplePlanner, "clicked_point") not in remappings
