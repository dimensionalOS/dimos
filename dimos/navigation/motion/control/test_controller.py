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

from dataclasses import replace
import math

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.controller import ControllerConfig
from dimos.navigation.motion.control.laws.seed import PursuitController
from dimos.navigation.motion.embodiment.go2 import GO2


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
    )


def _straight_path(n: int = 40, step: float = 0.1) -> Path:
    return Path(frame_id="world", poses=[_pose(i * step, 0.0) for i in range(n)])


def _fan_path() -> Path:
    """Rotate in place at the origin: yaw steps with zero displacement."""
    yaws = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5]
    poses = [_pose(0.0, 0.0, y) for y in yaws] + [_pose(0.1, 0.0, 1.5), _pose(1.0, 0.1, 1.5)]
    return Path(frame_id="world", poses=poses)


def test_on_path_drives_forward() -> None:
    tw = PursuitController().update(_pose(0.0, 0.0), _straight_path(), 0.0)
    assert tw.linear.x > 0.3
    assert abs(tw.linear.y) < 1e-6
    assert abs(tw.angular.z) < 1e-6


def test_lateral_offset_commands_crab_back() -> None:
    tw = PursuitController().update(_pose(1.0, 0.3), _straight_path(), 0.0)
    assert tw.linear.y < -0.1  # path is at y=0, robot at +0.3: crab right
    assert tw.linear.x > 0.0


def test_speed_clamped() -> None:
    tw = PursuitController(replace(GO2, max_speed=0.5)).update(
        _pose(-2.0, -2.0), _straight_path(), 0.0
    )
    assert math.hypot(tw.linear.x, tw.linear.y) <= 0.5 + 1e-9


def test_heading_error_yaws_toward_path() -> None:
    tw = PursuitController().update(_pose(0.0, 0.0, math.pi / 2), _straight_path(), 0.0)
    assert tw.angular.z < -0.5  # facing +y, path yaw 0: rotate cw


def test_fan_rotates_in_place() -> None:
    tw = PursuitController().update(_pose(0.0, 0.0, 0.0), _fan_path(), 0.0)
    assert tw.angular.z > 0.3
    assert math.hypot(tw.linear.x, tw.linear.y) < 0.15  # holds position


def test_fan_done_resumes_translation() -> None:
    tw = PursuitController().update(_pose(0.0, 0.0, 1.45), _fan_path(), 0.0)
    assert tw.linear.x > 0.1 or tw.linear.y != 0.0


def test_empty_path_stops() -> None:
    tw = PursuitController().update(_pose(0.0, 0.0), Path(frame_id="world", poses=[]), 0.0)
    assert tw.linear.x == 0.0 and tw.angular.z == 0.0


def test_yaw_rate_clamped() -> None:
    tw = PursuitController(replace(GO2, max_yaw_rate=1.4)).update(
        _pose(0.0, 0.0, math.pi - 0.1), _straight_path(), 0.0
    )
    assert abs(tw.angular.z) <= 1.4 + 1e-9


def test_config_rejects_unknown_fields() -> None:
    with pytest.raises(Exception):
        ControllerConfig(nonexistent=1.0)  # extra=forbid
    with pytest.raises(Exception):
        # the controller never looked one up; the field claimed it did
        ControllerConfig(frame_id="base_link")


def test_governor_creeps_in_tight_room() -> None:
    import numpy as np

    path = _straight_path()
    tight = np.full(len(path), 0.06)  # barely above the precision floor
    tw = PursuitController().update(_pose(0.0, 0.0), path, 0.0, clearance=tight)
    assert math.hypot(tw.linear.x, tw.linear.y) <= GO2.min_speed + 0.02


def test_governor_full_speed_in_open_room() -> None:
    import numpy as np

    path = _straight_path()
    wide = np.full(len(path), 1.0)
    tw_open = PursuitController().update(_pose(-1.0, 0.0), path, 0.0, clearance=wide)
    tw_blind = PursuitController().update(_pose(-1.0, 0.0), path, 0.0)
    assert math.hypot(tw_open.linear.x, tw_open.linear.y) == pytest.approx(
        math.hypot(tw_blind.linear.x, tw_blind.linear.y)
    )


def test_governor_reads_room_ahead_not_behind() -> None:
    import numpy as np

    path = _straight_path()  # 4 m of path
    clear = np.full(len(path), 1.0)
    clear[:5] = 0.06  # tight patch already behind the lookahead window
    tw = PursuitController().update(_pose(1.5, 0.0), path, 0.0, clearance=clear)
    assert math.hypot(tw.linear.x, tw.linear.y) > GO2.min_speed + 0.1
