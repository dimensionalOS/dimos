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

"""recall.py against a tiny synthetic recording, so it can be checked without the robot.

    uv run pytest dimos/experimental/frank/tools/test_recall.py

The robot walks east for ten seconds, then spins a full turn in place over ten more, with a
camera frame every half second.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))

import recall

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.Image import Image

T0 = 1_800_000_000.0


def _pose(ts: float, x: float, y: float, yaw_deg: float) -> PoseStamped:
    half = math.radians(yaw_deg) / 2
    return PoseStamped(
        ts,
        "world",
        position=[x, y, 0.0],
        orientation=Quaternion(0.0, 0.0, math.sin(half), math.cos(half)),
    )


@pytest.fixture
def store(tmp_path: Path) -> SqliteStore:
    st = SqliteStore(path=tmp_path / "memory.db")
    odom = st.stream("odom", PoseStamped)
    img = st.stream("color_image", Image)
    for i in range(400):  # 20 s at 20 Hz
        ts = T0 + i * 0.05
        if ts < T0 + 10:
            x, y, yaw = (ts - T0) * 0.3, 0.0, 0.0
        else:
            x, y, yaw = 3.0, 0.0, (ts - T0 - 10) * 36.0
        odom.append(_pose(ts, x, y, yaw), ts=ts)
        if i % 10 == 0:
            frame = np.full((72, 128, 3), int(yaw) % 255, dtype=np.uint8)
            img.append(Image.from_opencv(frame, ts=ts, frame_id="camera_optical"), ts=ts)
    return st


def test_track_finds_pose_at_time(store: SqliteStore) -> None:
    track = recall.Track(store, T0)
    x, y, yaw = track.at(T0 + 5.0)
    assert x == pytest.approx(1.5, abs=0.05) and yaw == pytest.approx(0.0, abs=1)
    assert track.at(T0 + 15.0)[2] == pytest.approx(180.0, abs=2)
    assert track.at(T0 - 5.0) is None


def test_within_gives_the_visit(store: SqliteStore) -> None:
    spans = recall.Track(store, T0).within(3.0, 0.0, 0.5)
    assert len(spans) == 1
    assert spans[0][0] == pytest.approx(T0 + 8.4, abs=0.1)  # x >= 2.5 from 8.33 s


def test_recent_shows_a_tile_per_heading(store: SqliteStore, tmp_path: Path) -> None:
    recall.NOW = T0 + 20
    out = str(tmp_path / "recall.jpg")
    text = recall.recent(store, 10.0, out, n=8)
    assert Path(out).exists()
    lines = text.splitlines()[1:]
    assert 6 <= len(lines) <= 8
    assert "facing north" in text and "facing south" in text


def test_near_shows_what_was_seen_there(store: SqliteStore, tmp_path: Path) -> None:
    recall.NOW = T0 + 20
    out = str(tmp_path / "near.jpg")
    text = recall.near(store, 1.0, 0.0, 0.5, 5.0, out)
    assert text.startswith(out)
    assert "x=1.0" in text or "x=0.8" in text or "x=1.2" in text
    assert "have not been" in recall.near(store, 10.0, 10.0, 0.5, 5.0, out)


def test_compass_words() -> None:
    assert recall.compass(0) == "east"
    assert recall.compass(90) == "north"
    assert recall.compass(-90) == "south"
    assert recall.compass(200) == "west"
