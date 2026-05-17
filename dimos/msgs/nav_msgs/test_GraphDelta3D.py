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

"""Unit tests for GraphDelta3D — pins wire layout vs the C++ encoder."""

from __future__ import annotations

import struct

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D


def _sample() -> GraphDelta3D:
    return GraphDelta3D(
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
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                id=200,
                metadata_id=0,
            ),
        ],
        transforms=[
            GraphDelta3D.Transform(
                translation=Vector3(0.1, 0.2, 0.3),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            GraphDelta3D.Transform(
                translation=Vector3(0.4, 0.5, 0.6),
                rotation=Quaternion(0.1, 0.2, 0.3, 0.9273618),
            ),
        ],
    )


def test_round_trip() -> None:
    original = _sample()
    decoded = GraphDelta3D.lcm_decode(original.lcm_encode())
    assert decoded.ts == original.ts
    assert len(decoded.nodes) == len(original.nodes)
    assert len(decoded.transforms) == len(original.transforms)
    for got, want in zip(decoded.nodes, original.nodes, strict=True):
        assert got.id == want.id
        assert got.metadata_id == want.metadata_id
        assert got.pose.frame_id == want.pose.frame_id
        assert got.pose.position.x == want.pose.position.x
    for got, want in zip(decoded.transforms, original.transforms, strict=True):
        assert got.translation.x == want.translation.x
        assert got.translation.y == want.translation.y
        assert got.translation.z == want.translation.z
        assert got.rotation.x == want.rotation.x
        assert got.rotation.w == want.rotation.w


def test_wire_layout_header() -> None:
    """Header is ``[node_count u8][timestamp f8]`` (big-endian)."""
    encoded = _sample().lcm_encode()
    node_count, timestamp = struct.unpack_from(">Qd", encoded, 0)
    assert node_count == 2
    assert timestamp == 1234.5


def test_empty() -> None:
    empty = GraphDelta3D(ts=0.0)
    decoded = GraphDelta3D.lcm_decode(empty.lcm_encode())
    assert decoded.nodes == []
    assert decoded.transforms == []


def test_misaligned_lengths_rejected() -> None:
    """nodes and transforms must be the same length — aligned arrays."""
    with pytest.raises(ValueError, match="aligned arrays"):
        GraphDelta3D(
            ts=0.0,
            nodes=[
                Graph3D.Node3D(pose=PoseStamped(ts=0.0, frame_id="map"), id=1, metadata_id=0),
            ],
            transforms=[],
        )


def test_node_layout_matches_graph3d() -> None:
    """A GraphDelta3D node's wire bytes should be identical to a Graph3D node."""
    node = Graph3D.Node3D(
        pose=PoseStamped(
            ts=42.0,
            frame_id="map",
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        id=7,
        metadata_id=1,
    )
    delta = GraphDelta3D(
        ts=0.0,
        nodes=[node],
        transforms=[
            GraphDelta3D.Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        ],
    )
    graph = Graph3D(ts=0.0, nodes=[node])

    delta_bytes = delta.lcm_encode()
    graph_bytes = graph.lcm_encode()

    # The Node3D body inside each is 8 (ts) + 4 (frame_id_len) + 3 (frame_id 'map')
    # + 56 (7 doubles for pos/quat) + 8 (id) + 8 (metadata_id) = 87 bytes.
    # In GraphDelta3D: header is 16 bytes (u8 node_count + f8 ts).
    # In Graph3D:      header is 24 bytes (u8 edge_count + u8 node_count + f8 ts).
    NODE_BYTES = 87
    assert delta_bytes[16 : 16 + NODE_BYTES] == graph_bytes[24 : 24 + NODE_BYTES]
