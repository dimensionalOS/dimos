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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2.blueprints.layers.layer_6_robot_body.robot_body_state import (
    _Go2RobotBodyState,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


def test_robot_body_snapshot_reports_missing_streams() -> None:
    body_state = _Go2RobotBodyState()
    try:
        snapshot = body_state.get_robot_body_snapshot()

        assert snapshot["available"] is True
        assert snapshot["version"] == "v1"
        assert snapshot["sensors"]["odom"]["available"] is False
        assert snapshot["connection"]["available"] is False
        assert snapshot["local_policy"]["available"] is True
        assert snapshot["safety"]["body_pose_available"] is False
    finally:
        _stop_modules(body_state)


def test_robot_body_snapshot_tracks_observed_odom() -> None:
    body_state = _Go2RobotBodyState()
    try:
        body_state._on_odom(PoseStamped(position=[1.5, -2.0, 0.0], frame_id="map"))

        snapshot = body_state.get_robot_body_snapshot()

        assert snapshot["sensors"]["odom"]["available"] is True
        assert snapshot["sensors"]["odom"]["count"] == 1
        assert snapshot["sensors"]["odom"]["latest"]["position"]["x"] == 1.5
        assert snapshot["sensors"]["odom"]["latest"]["position"]["y"] == -2.0
        assert snapshot["safety"]["body_pose_available"] is True
    finally:
        _stop_modules(body_state)


def test_sensor_state_counts_image_and_lidar_events() -> None:
    body_state = _Go2RobotBodyState()
    try:
        body_state._on_color_image(object())  # type: ignore[arg-type]
        body_state._on_lidar(object())  # type: ignore[arg-type]

        sensors = body_state.get_sensor_state()

        assert sensors["color_image"]["available"] is True
        assert sensors["color_image"]["count"] == 1
        assert sensors["lidar"]["available"] is True
        assert sensors["lidar"]["count"] == 1
    finally:
        _stop_modules(body_state)
