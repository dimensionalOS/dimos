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

The KNOWN_BYTES fixture below pins the wire format. When the Rust mirror at
``dimos/mapping/ray_tracing/rust/src/dynamic_cloud.rs`` is added, its unit
test must reproduce the same bytes — keep both sides in sync.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.msgs.nav_msgs.DynamicCloud import DynamicCloud


def _make_fixture():
    """A small fixed-content cloud used for cross-language byte-equality."""
    voxels = np.array([[1, -2, 3], [4, 5, -6]], dtype=np.int32)
    quantity = np.array([7, 8], dtype=np.uint32)
    event_indices = np.array([0, 1], dtype=np.uint32)
    event_timestamps = np.array([100, 200], dtype=np.uint64)
    return DynamicCloud(
        voxels=voxels,
        quantity=quantity,
        event_indices=event_indices,
        event_timestamps=event_timestamps,
        voxel_size=0.25,
        frame_id="map",
        ts=1.5,  # 1_500_000_000 ns
    )


# Hand-computed expected encoding of _make_fixture(). When a Rust mirror
# lands its unit test must reproduce these exact bytes.
KNOWN_BYTES = bytes.fromhex(
    "002f685900000000"  # 1_500_000_000 LE (0x5968_2F00)
    "0000803e"  # voxel_size 0.25 LE
    "0300"  # frame_id_len = 3
    "6d6170"  # frame_id "map"
    "02000000"  # num_points u32 = 2
    "01000000feffffff03000000"  # voxels: (1,-2,3)
    "0400000005000000faffffff"  # voxels: (4,5,-6)
    "0700000008000000"  # quantity: 7, 8
    "02000000"  # num_events u32 = 2
    "0000000001000000"  # event_indices: 0, 1
    "6400000000000000"  # event_timestamps: 100 (u64 LE)
    "c800000000000000"  # event_timestamps: 200 (u64 LE)
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
    assert decoded.frame_id == "world"
    assert decoded.voxel_size == 0.125
    assert decoded.event_indices.shape == (0,)
    assert decoded.event_timestamps.shape == (0,)


def test_no_events_roundtrip():
    """Cloud with points but no events should encode/decode cleanly."""
    cloud = DynamicCloud(
        voxels=np.array([[1, 2, 3]], dtype=np.int32),
        quantity=np.array([5], dtype=np.uint32),
        voxel_size=0.5,
        frame_id="map",
        ts=0.5,
    )
    encoded = cloud.lcm_encode()
    decoded = DynamicCloud.lcm_decode(encoded)
    np.testing.assert_array_equal(decoded.voxels, cloud.voxels)
    np.testing.assert_array_equal(decoded.quantity, cloud.quantity)
    assert decoded.event_indices.shape == (0,)
    assert decoded.event_timestamps.shape == (0,)


def test_world_positions():
    cloud = DynamicCloud(
        voxels=np.array([[2, 0, -1]], dtype=np.int32),
        quantity=np.array([1], dtype=np.uint32),
        voxel_size=0.5,
    )
    world = cloud.world_positions()
    np.testing.assert_array_almost_equal(world, [[1.0, 0.0, -0.5]])


def test_per_point_latest_timestamp():
    """``per_point_latest_timestamp`` picks the max event ts per voxel index."""
    cloud = DynamicCloud(
        voxels=np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=np.int32),
        quantity=np.array([1, 1, 1], dtype=np.uint32),
        # voxel 0 has events at t=10 and t=30; voxel 2 has one at t=20; voxel 1 has none
        event_indices=np.array([0, 0, 2], dtype=np.uint32),
        event_timestamps=np.array([10, 30, 20], dtype=np.uint64),
    )
    np.testing.assert_array_equal(cloud.per_point_latest_timestamp(), [30, 0, 20])


def test_shape_mismatch_raises():
    with pytest.raises(ValueError, match="length mismatch"):
        DynamicCloud(
            voxels=np.zeros((3, 3), dtype=np.int32),
            quantity=np.zeros(2, dtype=np.uint32),
        )


def test_event_index_out_of_range_raises():
    with pytest.raises(ValueError, match="out of range"):
        DynamicCloud(
            voxels=np.array([[0, 0, 0]], dtype=np.int32),
            quantity=np.array([1], dtype=np.uint32),
            event_indices=np.array([5], dtype=np.uint32),
            event_timestamps=np.array([100], dtype=np.uint64),
        )
