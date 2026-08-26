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

from pathlib import Path

import numpy as np
import pytest

from dimos.memory.store.mcap import McapStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image


def test_write_then_read_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "rec.mcap"
    w = McapStore(path=str(path), mode="w")
    w.start()
    imgs = w.stream("color_image", Image)
    odom = w.stream("odom", PoseStamped)
    nums = w.stream("nums", int)
    img = Image(data=np.zeros((4, 4, 3), dtype=np.uint8))
    imgs.append(img, ts=1.0, pose=(1, 2, 3, 0, 0, 0, 1), tags={"reception_ts": 1.5})
    imgs.append(img, ts=2.0)
    odom.append(PoseStamped(ts=3.0), ts=3.0)
    nums.append(7, ts=4.0)
    w.stop()

    r = McapStore(path=str(path))
    r.start()
    assert r.list_streams() == ["color_image", "nums", "odom"]
    got = list(r.stream("color_image"))
    assert [o.ts for o in got] == [1.0, 2.0]
    assert got[0].pose_tuple == (1, 2, 3, 0, 0, 0, 1)
    assert got[0].tags == {"reception_ts": 1.5}
    assert got[1].pose_tuple is None
    assert isinstance(got[0].data, Image)
    assert got[0].data.data.shape == (4, 4, 3)
    assert isinstance(next(iter(r.stream("odom"))).data, PoseStamped)
    assert next(iter(r.stream("nums"))).data == 7
    assert r.stream("color_image").count() == 2
    r.stop()


def test_write_only_cannot_query(tmp_path: Path) -> None:
    w = McapStore(path=str(tmp_path / "rec.mcap"), mode="w")
    w.start()
    s = w.stream("nums", int)
    s.append(1)
    with pytest.raises(NotImplementedError):
        list(s)
    w.stop()
