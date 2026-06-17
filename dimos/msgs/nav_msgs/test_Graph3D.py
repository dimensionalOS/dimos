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

"""Unit tests for the Graph3D message type.

These pin the wire layout (per ``Graph3D.ksy``) so the hand-written
Python encoder/decoder and the matching C++ encoder in
``Graph3D.hpp`` (same directory) don't drift.
"""

from __future__ import annotations

import struct

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Graph3D import Graph3D


def _make_graph() -> Graph3D:
    return Graph3D(
        ts=1234.5,
        nodes=[
            Graph3D.Node3D(
                pose=PoseStamped(
                    ts=10.5,
                    frame_id="map",
                    position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                id=100,
                metadata_id=1,
            ),
            Graph3D.Node3D(
                pose=PoseStamped(
                    ts=11.0,
                    frame_id="odom",
                    position=Vector3(4.0, 5.0, 6.0),
                    orientation=Quaternion(0.1, 0.2, 0.3, 0.9273618),
                ),
                id=200,
                metadata_id=2,
            ),
        ],
        edges=[
            Graph3D.Edge(start_id=100, end_id=200, timestamp=10.7, metadata_id=0),
        ],
    )


def test_round_trip() -> None:
    original = _make_graph()
    decoded = Graph3D.lcm_decode(original.lcm_encode())
    assert decoded.ts == original.ts
    assert len(decoded.nodes) == len(original.nodes)
    assert len(decoded.edges) == len(original.edges)
    for got, want in zip(decoded.nodes, original.nodes, strict=True):
        assert got.id == want.id
        assert got.metadata_id == want.metadata_id
        assert got.pose.ts == want.pose.ts
        assert got.pose.frame_id == want.pose.frame_id
        assert got.pose.position.x == want.pose.position.x
        assert got.pose.position.y == want.pose.position.y
        assert got.pose.position.z == want.pose.position.z
        assert got.pose.orientation.w == want.pose.orientation.w
    for got, want in zip(decoded.edges, original.edges, strict=True):
        assert got.start_id == want.start_id
        assert got.end_id == want.end_id
        assert got.timestamp == want.timestamp
        assert got.metadata_id == want.metadata_id


def test_wire_layout_header() -> None:
    """Header is `[edge_count u8][node_count u8][timestamp f8]` (big-endian)."""
    graph = _make_graph()
    encoded = graph.lcm_encode()
    edge_count, node_count, timestamp = struct.unpack_from(">QQd", encoded, 0)
    assert edge_count == 1
    assert node_count == 2
    assert timestamp == 1234.5


def test_wire_layout_node_starts_with_pose() -> None:
    """A node's first bytes are its pose, NOT id — matches Graph3D.ksy spec."""
    graph = _make_graph()
    encoded = graph.lcm_encode()
    # Header is 24 bytes; node starts at offset 24 with pose.ts (f8).
    (pose_ts,) = struct.unpack_from(">d", encoded, 24)
    assert pose_ts == 10.5, "first node's bytes must be pose.ts, not id"
    # After ts comes a uint32 frame_id_len = 3 (utf-8 "map").
    (frame_id_len,) = struct.unpack_from(">I", encoded, 24 + 8)
    assert frame_id_len == 3
    assert encoded[24 + 12 : 24 + 12 + 3] == b"map"


def test_empty_graph() -> None:
    empty = Graph3D(ts=0.0)
    decoded = Graph3D.lcm_decode(empty.lcm_encode())
    assert decoded.nodes == []
    assert decoded.edges == []


def test_edge_references_unknown_node_id_decodes_fine() -> None:
    """Decoder shouldn't validate id-references — that's a consumer concern."""
    graph = Graph3D(
        ts=1.0,
        nodes=[
            Graph3D.Node3D(pose=PoseStamped(ts=0.0, frame_id="map"), id=1, metadata_id=0),
        ],
        edges=[
            Graph3D.Edge(start_id=1, end_id=999, timestamp=0.5, metadata_id=0),  # 999 doesn't exist
        ],
    )
    decoded = Graph3D.lcm_decode(graph.lcm_encode())
    assert len(decoded.edges) == 1
    assert decoded.edges[0].end_id == 999
