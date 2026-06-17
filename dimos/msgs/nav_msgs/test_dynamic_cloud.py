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

"""Tests for DynamicCloud.

The KNOWN_BYTES fixture below is the same fixture the Rust mirror's unit
test asserts against — keep both sides in sync.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.msgs.nav_msgs.DynamicCloud import DynamicCloud


def _make_fixture():
    """A small fixed-content cloud used for cross-language byte-equality."""
    voxels = np.array([[1, -2, 3], [4, 5, -6]], dtype=np.int32)
    quantity = np.array([7, 8], dtype=np.uint32)
    event_indices = np.array([0, 1, 0], dtype=np.uint32)
    event_timestamps = np.array([1_000_000_000, 2_000_000_000, 1_500_000_000], dtype=np.uint64)
    return DynamicCloud(
        voxels=voxels,
        quantity=quantity,
        event_indices=event_indices,
        event_timestamps=event_timestamps,
        voxel_size=0.25,
        frame_id="map",
        ts=1.5,  # 1_500_000_000 ns
    )


# Hand-computed expected encoding of _make_fixture(); the Rust unit test
# (dimos/mapping/ray_tracing/rust/src/dynamic_cloud.rs::tests) reproduces
# the exact same bytes. Any drift on either side fails both tests.
KNOWN_BYTES = bytes.fromhex(
    "002f685900000000"  # ts_ns = 1_500_000_000 LE (0x5968_2F00)
    "0000803e"  # voxel_size = 0.25 f32 LE
    "0300"  # frame_id_len = 3
    "6d6170"  # frame_id "map"
    "02000000"  # num_points = 2
    "01000000feffffff03000000"  # voxels: (1,-2,3)
    "0400000005000000faffffff"  # voxels: (4,5,-6)
    "0700000008000000"  # quantity: 7, 8
    "03000000"  # num_events = 3
    "000000000100000000000000"  # event_indices: 0, 1, 0
    "00ca9a3b00000000"  # event_timestamps[0] = 1_000_000_000 LE (0x3B9A_CA00)
    "0094357700000000"  # event_timestamps[1] = 2_000_000_000 LE (0x7735_9400)
    "002f685900000000"  # event_timestamps[2] = 1_500_000_000 LE (0x5968_2F00)
)


def test_roundtrip():
    cloud = _make_fixture()
    encoded = cloud.lcm_encode()
    decoded = DynamicCloud.lcm_decode(encoded)

    assert decoded.frame_id == cloud.frame_id
    assert decoded.voxel_size == cloud.voxel_size
    assert decoded.ts == cloud.ts
    np.testing.assert_array_equal(decoded.voxels, cloud.voxels)
    np.testing.assert_array_equal(decoded.quantity, cloud.quantity)
    np.testing.assert_array_equal(decoded.event_indices, cloud.event_indices)
    np.testing.assert_array_equal(decoded.event_timestamps, cloud.event_timestamps)


def test_known_bytes():
    """Pinned wire format; mirrors the Rust unit test fixture exactly."""
    encoded = _make_fixture().lcm_encode()
    assert encoded == KNOWN_BYTES, f"encoded:\n{encoded.hex()}\nexpected:\n{KNOWN_BYTES.hex()}"


def test_decode_known_bytes():
    decoded = DynamicCloud.lcm_decode(KNOWN_BYTES)
    expected = _make_fixture()
    assert decoded.frame_id == expected.frame_id
    assert decoded.voxel_size == expected.voxel_size
    np.testing.assert_array_equal(decoded.voxels, expected.voxels)
    np.testing.assert_array_equal(decoded.quantity, expected.quantity)
    np.testing.assert_array_equal(decoded.event_indices, expected.event_indices)
    np.testing.assert_array_equal(decoded.event_timestamps, expected.event_timestamps)


def test_empty_cloud():
    # 0.125 is exactly representable in f32; 0.1 would round-trip with f32 drift.
    cloud = DynamicCloud(voxel_size=0.125, frame_id="world", ts=0.0)
    encoded = cloud.lcm_encode()
    decoded = DynamicCloud.lcm_decode(encoded)
    assert len(decoded) == 0
    assert decoded.event_indices.shape[0] == 0
    assert decoded.frame_id == "world"
    assert decoded.voxel_size == 0.125


def test_world_positions():
    cloud = DynamicCloud(
        voxels=np.array([[2, 0, -1]], dtype=np.int32),
        quantity=np.array([1], dtype=np.uint32),
        voxel_size=0.5,
    )
    world = cloud.world_positions()
    np.testing.assert_array_almost_equal(world, [[1.0, 0.0, -0.5]])


def test_per_point_latest_timestamp():
    # event_indices: [0, 1, 0] with timestamps [1, 2, 5]
    #   point 0 has events at t=1 and t=5 → latest is 5
    #   point 1 has one event at t=2 → latest is 2
    #   point 2 has no events → 0
    cloud = DynamicCloud(
        voxels=np.zeros((3, 3), dtype=np.int32),
        quantity=np.zeros(3, dtype=np.uint32),
        event_indices=np.array([0, 1, 0], dtype=np.uint32),
        event_timestamps=np.array([1, 2, 5], dtype=np.uint64),
    )
    latest = cloud.per_point_latest_timestamp()
    np.testing.assert_array_equal(latest, [5, 2, 0])


def test_voxels_quantity_length_mismatch_raises():
    with pytest.raises(ValueError, match="voxels/quantity length mismatch"):
        DynamicCloud(
            voxels=np.zeros((3, 3), dtype=np.int32),
            quantity=np.zeros(2, dtype=np.uint32),
        )


def test_event_arrays_length_mismatch_raises():
    with pytest.raises(ValueError, match="event_indices/event_timestamps length mismatch"):
        DynamicCloud(
            voxels=np.zeros((2, 3), dtype=np.int32),
            quantity=np.zeros(2, dtype=np.uint32),
            event_indices=np.array([0], dtype=np.uint32),
            event_timestamps=np.array([1, 2], dtype=np.uint64),
        )


def test_event_index_out_of_range_raises():
    with pytest.raises(ValueError, match="event index 5 out of range"):
        DynamicCloud(
            voxels=np.zeros((2, 3), dtype=np.int32),
            quantity=np.zeros(2, dtype=np.uint32),
            event_indices=np.array([5], dtype=np.uint32),
            event_timestamps=np.array([1], dtype=np.uint64),
        )
