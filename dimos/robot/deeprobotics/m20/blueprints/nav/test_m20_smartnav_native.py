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
from dimos.navigation.smart_nav.modules.fastlio2.fastlio2 import FastLio2
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
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
    goal_reached_threshold: str | None = None,
    goal_behind_range: str | None = None,
    freeze_ang: str | None = None,
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
        atom.kwargs
        for atom in module.m20_smartnav_native.blueprints
        if atom.module is CmdVelMux
    ]
    assert len(muxes) == 1
    return muxes[0]


def _local_planner_kwargs(module) -> dict:
    local_planners = [
        atom.kwargs
        for atom in module.m20_smartnav_native.blueprints
        if atom.module is LocalPlanner
    ]
    assert len(local_planners) == 1
    return local_planners[0]


def _path_follower_kwargs(module) -> dict:
    path_followers = [
        atom.kwargs
        for atom in module.m20_smartnav_native.blueprints
        if atom.module is PathFollower
    ]
    assert len(path_followers) == 1
    return path_followers[0]


def _native_kwargs(module, native_cls) -> dict:
    matches = [
        atom.kwargs
        for atom in module.m20_smartnav_native.blueprints
        if atom.module is native_cls
    ]
    assert len(matches) == 1
    return matches[0]


def test_m20_blueprint_caps_path_follower_yaw_rate_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _path_follower_kwargs(module)["max_yaw_rate"] == 40.0


def test_m20_blueprint_accepts_path_follower_yaw_rate_env_override(monkeypatch):
    module = _load_m20_native(monkeypatch, max_yaw_rate="20.0")

    assert _path_follower_kwargs(module)["max_yaw_rate"] == 20.0


def test_m20_blueprint_uses_long_nav_command_window_by_default(monkeypatch):
    module = _load_m20_native(monkeypatch)

    assert _cmd_vel_mux_kwargs(module)["max_nav_command_duration_sec"] == 30.0


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


def test_m20_blueprint_passes_local_planner_arrival_thresholds(monkeypatch):
    module = _load_m20_native(monkeypatch)

    local_planner = _local_planner_kwargs(module)
    assert local_planner["goal_reached_threshold"] == 0.3
    assert local_planner["goal_behind_range"] == 0.3
    assert local_planner["freeze_ang"] == 180.0
    assert local_planner["two_way_drive"] is False


def test_m20_blueprint_accepts_local_planner_arrival_env_overrides(monkeypatch):
    module = _load_m20_native(
        monkeypatch,
        goal_reached_threshold="0.25",
        goal_behind_range="0.2",
        freeze_ang="135.0",
    )

    local_planner = _local_planner_kwargs(module)
    assert local_planner["goal_reached_threshold"] == 0.25
    assert local_planner["goal_behind_range"] == 0.2
    assert local_planner["freeze_ang"] == 135.0


def test_m20_blueprint_uses_slam_cpu_affinity_for_native_path(monkeypatch):
    monkeypatch.setenv("M20_SLAM_BACKEND", "fastlio2")
    monkeypatch.setenv("M20_FASTLIO2_IMU", "airy")
    module = _load_m20_native(monkeypatch, slam_cores="4,5")

    assert _native_kwargs(module, FastLio2)["cpu_affinity"] == frozenset({4, 5})
    assert _native_kwargs(module, DrddsLidarBridge)["cpu_affinity"] == frozenset(
        {4, 5}
    )
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
