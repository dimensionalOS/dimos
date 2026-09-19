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

"""The two recorder blueprints: sensors and Point-LIO, nothing that plans."""

from __future__ import annotations

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.depth_cloud.module import StereoCloud
from dimos.hardware.sensors.lidar.pointlio.module import PointLioRust
from dimos.robot.galaxea.r1pro.blueprints.basic import r1pro_recorder as recorder_module
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_recorder import (
    CALIBRATION_COLOR_HZ,
    r1pro_calibration_recorder,
    r1pro_recorder,
)
from dimos.robot.galaxea.r1pro.connection import R1ProConnection
from dimos.robot.galaxea.r1pro.lio import (
    LIDAR_FRAME,
    ODOM_FRAME,
    R1ProLioMountTf,
    R1ProMid360,
)


def _atoms(blueprint: Blueprint) -> dict:
    return {atom.module: atom for atom in blueprint.active_blueprints}


@pytest.mark.parametrize("blueprint", [r1pro_recorder, r1pro_calibration_recorder])
def test_the_robot_hangs_off_pointlio_and_its_streams_stay_apart(blueprint: Blueprint) -> None:
    atoms = _atoms(blueprint)
    # The wheel odometry is off, so base_link has one parent: the mount tf's.
    assert atoms[R1ProConnection].kwargs["publish_odom"] is False
    assert R1ProLioMountTf in atoms
    assert atoms[R1ProMid360].kwargs["frame_id"] == LIDAR_FRAME
    assert atoms[PointLioRust].kwargs["sensor_frame_id"] == LIDAR_FRAME
    assert atoms[PointLioRust].kwargs["frame_id"] == ODOM_FRAME
    # Above the estimator's own rate, so the caps never decide anything.
    assert atoms[PointLioRust].kwargs["pointcloud_freq"] >= 100.0
    assert atoms[PointLioRust].kwargs["odom_freq"] >= 100.0

    remaps = blueprint.remapping_map
    key = blueprint._instance_key
    # Renamed off the vendor driver's `lidar` and the wheels' `odometry`, so
    # the recording never interleaves two producers on one stream.
    assert remaps[(key(R1ProMid360), "lidar")] == "lidar_raw"
    assert remaps[(key(PointLioRust), "lidar")] == "pointlio_lidar"
    assert remaps[(key(PointLioRust), "odometry")] == "pointlio_odometry"


@pytest.mark.parametrize("blueprint", [r1pro_recorder, r1pro_calibration_recorder])
def test_nothing_derived_is_recorded(blueprint: Blueprint) -> None:
    # A stereo cloud is a pure function of the two eyes and the matcher's
    # parameters; a recorded one would be an answer about to change.
    assert StereoCloud not in _atoms(blueprint)
    planners = [m for m in _atoms(blueprint) if "Planner" in m.__name__ or "Costmap" in m.__name__]
    assert planners == []


def test_full_rate_recorder_puts_no_cap_on_the_cameras() -> None:
    atoms = _atoms(r1pro_recorder)
    assert atoms[R1ProConnection].kwargs["color_publish_hz"] == 0.0
    # Wrists left at the connection's default: this is the everything recorder.
    assert "enable_wrist_color" not in atoms[R1ProConnection].kwargs


def test_calibration_recorder_takes_both_eyes_at_30_and_drops_the_wrists() -> None:
    atoms = _atoms(r1pro_calibration_recorder)
    assert CALIBRATION_COLOR_HZ == 30.0
    assert atoms[R1ProConnection].kwargs["color_publish_hz"] == 30.0
    assert atoms[R1ProConnection].kwargs["enable_wrist_color"] is False


def test_the_documented_command_names_exactly_the_streams_the_fit_needs() -> None:
    # The docstring is what an operator copies; keep it honest against the
    # port names the blueprint actually produces.
    doc = recorder_module.__doc__ or ""
    assert "dimos run r1pro-calibration-recorder" in doc
    assert "--record sqlite --record-engine rust" in doc
    for stream in (
        "head_left_color",
        "head_right_color",
        "head_left_info",
        "head_right_info",
        "pointlio_lidar",
        "pointlio_odometry",
        "tf",
    ):
        assert stream in doc, stream
    remapped_to = set(r1pro_calibration_recorder.remapping_map.values())
    assert {"pointlio_lidar", "pointlio_odometry"} <= remapped_to
    for port in ("head_left_color", "head_right_color", "head_left_info", "head_right_info"):
        assert port in R1ProConnection.__annotations__, port
