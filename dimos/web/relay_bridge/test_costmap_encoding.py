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

"""costmap.zlib.v1 golden vectors: the encoder pinned byte-exact.

The mirror of the cockpit's decoders.test.ts vectors block: pytest re-encodes
each vector, vitest inflates it, so drift on either side fails a suite.
"""

import base64
import json
from typing import Any
import zlib

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.web.relay_bridge.builtin_codecs import encode_costmap
from dimos.web.relay_bridge.gen_costmap_fixtures import grid_msg
from dimos.web.relay_bridge.locate import find_web_dir

with open(find_web_dir() / "shared" / "fixtures" / "costmap_frames.json") as f:
    VECTORS: list[dict[str, Any]] = json.load(f)["vectors"]


@pytest.mark.parametrize("vec", VECTORS, ids=lambda v: v["name"])
def test_encoder_reproduces_golden_vector(vec: dict[str, Any]) -> None:
    """Compressed-byte equality pins CPython's bundled zlib at level 6. If a
    future CPython legitimately shifts those bytes, regenerate the fixture
    (gen_costmap_fixtures) - the decoded-grid assertions on both sides are
    the actual wire contract."""
    meta = vec["meta"]
    cells = np.frombuffer(base64.b64decode(vec["grid_b64"]), dtype=np.uint8)
    rows = np.where(cells == 255, -1, cells).astype(np.int8).reshape(meta["h"], meta["w"])
    ox, oy, yaw = meta["origin"]
    msg = grid_msg(rows.tolist(), meta["res"], ox, oy, yaw)
    encoded = encode_costmap(msg)
    assert encoded is not None
    payload, out_meta = encoded.payload, encoded.meta
    assert payload == base64.b64decode(vec["payload_b64"])
    assert zlib.decompress(payload) == cells.tobytes()
    assert (out_meta["w"], out_meta["h"], out_meta["res"]) == (meta["w"], meta["h"], meta["res"])
    assert out_meta["origin"][:2] == meta["origin"][:2]
    assert out_meta["origin"][2] == pytest.approx(yaw, abs=1e-12)


def test_encoder_handles_wire_decoded_grid() -> None:
    """The bridge encodes grids that arrived over LCM; the decoded origin must
    still expose .yaw (regression: the raw wire pose did not, and every
    global_costmap frame failed to encode)."""
    msg = grid_msg([[0, 100], [-1, 50]], 0.05, -1.25, 2.5, 0.5)
    decoded = OccupancyGrid.lcm_decode(msg.lcm_encode())
    encoded = encode_costmap(decoded)
    assert encoded is not None
    payload, meta = encoded.payload, encoded.meta
    assert meta["origin"] == pytest.approx([-1.25, 2.5, 0.5])
    assert zlib.decompress(payload) == bytes([0, 100, 255, 50])


def test_empty_grid_encodes_to_none() -> None:
    assert encode_costmap(OccupancyGrid()) is None


def test_oversized_grid_is_downsampled_within_budget() -> None:
    rows = np.zeros((4096, 4096), dtype=np.int8)
    rows[0, 1] = 100  # lone obstacle: the block max must keep it
    rows[2:4, 2:4] = -1  # a fully unknown block stays unknown
    msg = OccupancyGrid(grid=rows, resolution=0.05, origin=Pose(1.0, 2.0, 0.0), ts=1.0)
    encoded = encode_costmap(msg)
    assert encoded is not None
    payload, meta = encoded.payload, encoded.meta
    assert (meta["w"], meta["h"]) == (2048, 2048)
    assert meta["res"] == pytest.approx(0.1)
    assert meta["origin"][:2] == [1.0, 2.0]
    cells = np.frombuffer(zlib.decompress(payload), dtype=np.uint8).reshape(2048, 2048)
    assert cells[0, 0] == 100
    assert cells[1, 1] == 255
    assert cells[2, 2] == 0


def test_grid_at_exactly_max_side_is_not_downsampled() -> None:
    rows = np.full((4, 2048), 7, dtype=np.int8)
    msg = OccupancyGrid(grid=rows, resolution=0.05, origin=Pose(0.0, 0.0, 0.0), ts=1.0)
    encoded = encode_costmap(msg)
    assert encoded is not None
    payload, meta = encoded.payload, encoded.meta
    assert (meta["w"], meta["h"], meta["res"]) == (2048, 4, 0.05)
    assert zlib.decompress(payload) == rows.astype(np.uint8).tobytes()


def test_non_square_oversized_grid_uses_ceil_factor() -> None:
    rows = np.zeros((100, 4100), dtype=np.int8)
    msg = OccupancyGrid(grid=rows, resolution=0.05, origin=Pose(0.0, 0.0, 0.0), ts=1.0)
    encoded = encode_costmap(msg)
    assert encoded is not None
    meta = encoded.meta
    assert (meta["w"], meta["h"]) == (1366, 33)
    assert meta["res"] == pytest.approx(0.15)
