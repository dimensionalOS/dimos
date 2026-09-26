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

"""voxels.zlib.v1 golden vectors: the encoder pinned byte-exact, plus its
quantization, budget and parameter rules.

The mirror of the SDK's decoders.test.ts vectors block: pytest re-encodes
each vector, vitest inflates it, so drift on either side fails a suite.
"""

import base64
from collections.abc import Mapping
import json
from typing import Any
import zlib

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.web.codecs import resolve_encoder
from dimos.web.relay_bridge import builtin_codecs
from dimos.web.relay_bridge.builtin_codecs import _VOXEL_RECORD, encode_voxels
from dimos.web.relay_bridge.gen_voxel_fixtures import cloud_msg, voxel_indices
from dimos.web.relay_bridge.locate import find_web_dir

with open(find_web_dir() / "shared" / "fixtures" / "voxel_frames.json") as f:
    VECTORS: list[dict[str, Any]] = json.load(f)["vectors"]


def unpack(payload: bytes, meta: Mapping[str, Any]) -> list[list[int]]:
    """Reference decoder: sorted voxel indices from the chunk records."""
    records = np.frombuffer(zlib.decompress(payload), dtype=_VOXEL_RECORD)
    assert len(records) == meta["chunks"]
    bits = np.unpackbits(records["bits"], axis=1, bitorder="little")
    chunk, bit = np.nonzero(bits)
    assert len(bit) == meta["n"]
    local = np.stack([bit & 15, (bit >> 4) & 15, bit >> 8], axis=1)
    origin = np.stack([records["cx"], records["cy"], records["cz"]], axis=1)[chunk].astype(np.int64)
    rows: list[list[int]] = np.unique(origin * 16 + local, axis=0).tolist()
    return rows


@pytest.mark.parametrize("vec", VECTORS, ids=lambda v: v["name"])
def test_encoder_reproduces_golden_vector(vec: dict[str, Any]) -> None:
    """Compressed-byte equality pins CPython's bundled zlib at level 6. If a
    future CPython legitimately shifts those bytes, regenerate the fixture
    (gen_voxel_fixtures) - the unpacked-voxel assertions on both sides are the
    actual wire contract."""
    encoded = encode_voxels(cloud_msg(vec["points"]), vec["params"])
    assert encoded.payload == base64.b64decode(vec["payload_b64"])
    assert encoded.meta == vec["meta"]
    assert unpack(encoded.payload, encoded.meta) == vec["voxels"]
    assert vec["voxels"] == voxel_indices(vec["points"], vec["params"]["res"])


def test_encoder_handles_wire_decoded_cloud() -> None:
    """The bridge encodes clouds that arrived over LCM."""
    msg = cloud_msg([[0.025, 0.025, 0.025], [1.025, -0.475, 0.125]])
    encoded = encode_voxels(PointCloud2.lcm_decode(msg.lcm_encode()), {"res": 0.05})
    assert encoded.meta == {"res": 0.05, "n": 2, "chunks": 2}
    assert unpack(encoded.payload, encoded.meta) == [[0, 0, 0], [20, -10, 2]]


EMPTY_FRAME = (zlib.compress(b"", 6), {"res": 0.05, "n": 0, "chunks": 0})


def test_empty_cloud_encodes_to_an_empty_frame() -> None:
    # An empty cloud is a frame, not a skip: a filtered cloud can empty out
    # and a producer can clear its map, and the panel must clear with it.
    encoded = encode_voxels(PointCloud2(), {})
    assert (encoded.payload, encoded.meta) == EMPTY_FRAME
    assert unpack(encoded.payload, encoded.meta) == []
    nan = float("nan")
    encoded = encode_voxels(cloud_msg([[nan, 0.0, 0.0]]), {})
    assert (encoded.payload, encoded.meta) == EMPTY_FRAME


def test_nonempty_empty_nonempty_sequence() -> None:
    points = [[0.025, 0.025, 0.025], [1.025, -0.475, 0.125]]
    first = encode_voxels(cloud_msg(points), {})
    cleared = encode_voxels(cloud_msg([]), {})
    again = encode_voxels(cloud_msg(points), {})
    assert first.meta["n"] == 2
    assert (cleared.payload, cleared.meta) == EMPTY_FRAME
    assert (again.payload, again.meta) == (first.payload, first.meta)


def test_non_finite_and_far_points_are_dropped() -> None:
    inf = float("inf")
    points = [[0.025, 0.025, 0.025], [inf, 0.0, 0.0], [0.0, float("nan"), 0.0], [1e9, 0.0, 0.0]]
    encoded = encode_voxels(cloud_msg(points), {})
    assert encoded.meta == {"res": 0.05, "n": 1, "chunks": 1}  # res defaults to the mapper's


def test_over_budget_cloud_is_coarsened(monkeypatch: pytest.MonkeyPatch) -> None:
    # A 2x2x2 block of 5 cm voxels is one 10 cm voxel once the budget bites.
    block = [[x, y, z] for x in (0.025, 0.075) for y in (0.025, 0.075) for z in (0.025, 0.075)]
    monkeypatch.setattr(builtin_codecs, "_VOXEL_MAX_VOXELS", 4)
    encoded = encode_voxels(cloud_msg(block), {"res": 0.05})
    assert encoded.meta == {"res": 0.1, "n": 1, "chunks": 1}
    assert unpack(encoded.payload, encoded.meta) == [[0, 0, 0]]


def test_over_chunk_budget_cloud_is_coarsened(monkeypatch: pytest.MonkeyPatch) -> None:
    # Two voxels in two chunks (indices 0 and 16) share a chunk at 10 cm.
    monkeypatch.setattr(builtin_codecs, "_VOXEL_MAX_CHUNKS", 1)
    encoded = encode_voxels(cloud_msg([[0.025, 0.0, 0.0], [0.825, 0.0, 0.0]]), {"res": 0.05})
    assert encoded.meta == {"res": 0.1, "n": 2, "chunks": 1}
    assert unpack(encoded.payload, encoded.meta) == [[0, 0, 0], [8, 0, 0]]


def test_params_check_rejects_bad_res() -> None:
    codec = resolve_encoder("voxels.zlib.v1", PointCloud2)
    assert codec.encode is encode_voxels
    assert codec.check_params is not None
    codec.check_params({})
    codec.check_params({"res": 0.1})
    for bad in (0, -0.05, "0.05", True, float("nan")):
        with pytest.raises(ValueError):
            codec.check_params({"res": bad})
