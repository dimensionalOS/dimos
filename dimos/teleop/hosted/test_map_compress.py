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

"""Unit tests for MapCompressModule's costmap/odom compression."""

from __future__ import annotations

import base64
from collections.abc import Iterator
import json
import math
from typing import Any
from unittest.mock import MagicMock

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.time import time_from_seconds
from dimos.teleop.hosted.map_compress import MapCompressModule


@pytest.fixture
def module(mocker) -> Iterator[MapCompressModule]:
    module = MapCompressModule()
    mocker.patch.object(module.map_out, "publish")
    try:
        yield module
    finally:
        module.stop()


def _published_json(mock: MagicMock, msg_type: str) -> dict[str, Any] | None:
    """Return the last JSON payload of the given type published on a mock."""
    for call in reversed(mock.publish.call_args_list):
        (data,) = call.args
        try:
            msg = json.loads(data)
        except (ValueError, TypeError):
            continue
        if msg.get("type") == msg_type:
            return msg
    return None


def _occupancy(grid: Any, resolution: float = 0.1) -> OccupancyGrid:
    cells = np.asarray(grid, dtype=np.int8)
    message = OccupancyGrid(
        info=MapMetaData(
            width=cells.shape[1],
            height=cells.shape[0],
            resolution=resolution,
            map_load_time=Time(sec=0, nanosec=0),
            origin=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.asarray(cells.ravel(), dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    return cdr_decode(cdr_encode(message), OccupancyGrid)


def test_costmap_encodes_and_publishes_map(module: MapCompressModule) -> None:
    grid = _occupancy([[-1, 0, 100], [0, 0, -1]])
    module._on_costmap(grid)

    msg = _published_json(module.map_out, "map")
    assert msg is not None, "no map message published"
    assert msg["fmt"] == "png" and msg["png_b64"]
    assert msg["w"] == 3 and msg["h"] == 2
    assert msg["res"] == pytest.approx(0.1)
    assert len(msg["origin"]) == 2


def test_costmap_png_round_trips_palette(module: MapCompressModule) -> None:
    module._on_costmap(_occupancy([[-1, 0, 100]]))
    msg = _published_json(module.map_out, "map")
    assert msg is not None
    raw = base64.b64decode(msg["png_b64"])
    # BGRA (color + alpha) — the rerun palette baked in by the robot.
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)
    assert img.shape[2] == 4  # has alpha
    row = [tuple(int(v) for v in px) for px in img[0]]
    # unknown → transparent; free → dark cyan; occupied(100) → white-hot lethal.
    assert row[0] == (0, 0, 0, 0)  # unknown transparent
    assert row[1] == (68, 58, 30, 255)  # free #1e3a44 in BGRA
    assert row[2] == (255, 255, 255, 255)  # 100 = lethal #ffffff


def test_costmap_rate_gated(module: MapCompressModule) -> None:
    module._on_costmap(_occupancy([[0, 0]]))
    first = len(module.map_out.publish.call_args_list)
    module._on_costmap(_occupancy([[0, 0]]))  # immediately again → gated out
    assert len(module.map_out.publish.call_args_list) == first


def test_block_max_preserves_obstacle_when_coarsening(module: MapCompressModule) -> None:
    # 0.02 m/cell → coarsen by 5× to reach 0.1. A lone obstacle must survive.
    cells = np.zeros((10, 10), dtype=np.int8)
    cells[3, 3] = 100
    module._on_costmap(_occupancy(cells, resolution=0.02))
    msg = _published_json(module.map_out, "map")
    assert msg is not None
    assert msg["res"] == pytest.approx(0.1)  # coarsened 5×
    raw = base64.b64decode(msg["png_b64"])
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)
    # Lethal (100) survives as an opaque white pixel (BGRA #ffffff).
    lethal = np.all(img == (255, 255, 255, 255), axis=-1)
    assert lethal.any(), "obstacle erased by coarsening"


def test_odom_publishes_planar_pose(module: MapCompressModule) -> None:
    q = quaternion_from_euler(0.0, 0.0, math.pi / 2)  # yaw = 90°
    pose = PoseStamped(
        header=Header(stamp=time_from_seconds(123), frame_id=""),
        pose=Pose(position=Point(x=1.5, y=-2, z=0.3), orientation=q),
    )
    module._on_odom(pose)

    msg = _published_json(module.map_out, "odom")
    assert msg is not None
    assert msg["x"] == pytest.approx(1.5) and msg["y"] == pytest.approx(-2.0)
    assert msg["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)
    assert msg["ts"] == pytest.approx(123.0)


def test_empty_costmap_publishes_nothing(module: MapCompressModule) -> None:
    module._on_costmap(
        OccupancyGrid(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            info=MapMetaData(
                map_load_time=Time(sec=0, nanosec=0),
                resolution=0.0,
                width=0,
                height=0,
                origin=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            data=np.array([], dtype=np.int8),
        )
    )  # no-arg = empty 1D grid; must be skipped
    assert _published_json(module.map_out, "map") is None


def test_odom_degenerate_quaternion_does_not_raise(module: MapCompressModule) -> None:
    # A zero quaternion makes to_euler() (scipy) raise; _on_odom runs inside an
    # RxPY subscriber, so it must drop the frame, not kill the odom stream.
    pose = PoseStamped(
        pose=Pose(
            orientation=Quaternion(w=0, x=0.0, y=0.0, z=0.0), position=Point(x=0.0, y=0.0, z=0.0)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    module._on_odom(pose)  # must not raise
    assert _published_json(module.map_out, "odom") is None


def test_oversized_map_dropped(module: MapCompressModule) -> None:
    rng = np.random.default_rng(7)
    noise = rng.choice([-1, 0, 50, 100], size=(500, 500))
    module._on_costmap(_occupancy(noise))
    module.map_out.publish.assert_not_called()
    assert module._last_map_pub > 0  # throttle window consumed


def test_malformed_grid_does_not_break_next_frame(module):
    module._on_costmap(
        OccupancyGrid(
            info=MapMetaData(
                width=2,
                height=2,
                resolution=0.1,
                map_load_time=Time(sec=0, nanosec=0),
                origin=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            data=np.array([0], dtype=np.int8),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
    )
    module.map_out.publish.assert_not_called()
    module._on_costmap(_occupancy([[0, 100]]))
    assert _published_json(module.map_out, "map")["w"] == 2
