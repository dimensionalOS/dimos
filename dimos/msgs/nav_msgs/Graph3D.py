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

"""Graph3D: pose-graph / visibility-graph message with typed nodes and edges.

Edges reference nodes by ``id`` (not list index), so producers are free
to reorder or re-emit nodes between snapshots. ``metadata_id`` is a
caller-defined enum — ex: for far_planner: 0=normal, 1=odom, 2=goal
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import TYPE_CHECKING, BinaryIO

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


# Default node metadata_id → RGBA. Callers can override via the
# `node_colors` kwarg on to_rerun*; these defaults match the far_planner
# node-type enum (0=normal, 1=odom, 2=goal, 3=frontier, 4=navpoint).
_DEFAULT_NODE_COLORS: dict[int, tuple[int, int, int, int]] = {
    0: (180, 180, 180, 200),
    1: (0, 255, 0, 255),
    2: (255, 0, 0, 255),
    3: (255, 165, 0, 200),
    4: (0, 200, 255, 200),
}
_DEFAULT_NODE_COLOR = (200, 200, 200, 180)

# Edge-type → RGBA, soft default (caller can override via to_rerun args).
_DEFAULT_EDGE_COLORS: dict[int, tuple[int, int, int, int]] = {
    0: (0, 220, 100, 200),  # odom / traversable — green
    1: (255, 180, 0, 220),  # loop_closure / partial — yellow
    2: (255, 50, 50, 150),  # blocked — red
}
_DEFAULT_EDGE_COLOR = (180, 180, 180, 180)


class Graph3D(Timestamped):
    msg_name = "nav_msgs.Graph3D"

    @dataclass
    class Node3D:
        pose: PoseStamped
        id: int = 0
        metadata_id: int = 0

    @dataclass
    class Edge:
        start_id: int
        end_id: int
        timestamp: float = 0.0
        metadata_id: int = 0

    ts: float
    nodes: list[Node3D]
    edges: list[Edge]

    def __init__(
        self,
        ts: float = 0.0,
        nodes: list[Graph3D.Node3D] | None = None,
        edges: list[Graph3D.Edge] | None = None,
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.nodes = nodes if nodes is not None else []
        self.edges = edges if edges is not None else []

    def lcm_encode(self) -> bytes:
        # Field order matches Graph3D.ksy: edge_count, node_count, ts,
        # nodes[] (pose, id, metadata_id), edges[].
        parts: list[bytes] = []
        parts.append(struct.pack(">QQd", len(self.edges), len(self.nodes), self.ts))
        for node in self.nodes:
            frame_id_bytes = node.pose.frame_id.encode("utf-8")
            parts.append(struct.pack(">d", node.pose.ts))
            parts.append(struct.pack(">I", len(frame_id_bytes)))
            parts.append(frame_id_bytes)
            parts.append(
                struct.pack(
                    ">7d",
                    node.pose.position.x,
                    node.pose.position.y,
                    node.pose.position.z,
                    node.pose.orientation.x,
                    node.pose.orientation.y,
                    node.pose.orientation.z,
                    node.pose.orientation.w,
                )
            )
            parts.append(struct.pack(">QQ", node.id, node.metadata_id))
        for edge in self.edges:
            parts.append(
                struct.pack(">QQdQ", edge.start_id, edge.end_id, edge.timestamp, edge.metadata_id)
            )
        return b"".join(parts)

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> Graph3D:
        buf = data if isinstance(data, (bytes, bytearray)) else data.read()
        offset = 0
        edge_count, node_count, graph_ts = struct.unpack_from(">QQd", buf, offset)
        offset += 24

        nodes: list[Graph3D.Node3D] = []
        for _ in range(node_count):
            (pose_ts,) = struct.unpack_from(">d", buf, offset)
            offset += 8
            (frame_id_len,) = struct.unpack_from(">I", buf, offset)
            offset += 4
            frame_id = buf[offset : offset + frame_id_len].decode("utf-8")
            offset += frame_id_len
            px, py, pz, qx, qy, qz, qw = struct.unpack_from(">7d", buf, offset)
            offset += 56
            node_id, metadata_id = struct.unpack_from(">QQ", buf, offset)
            offset += 16
            pose = PoseStamped(
                ts=pose_ts,
                frame_id=frame_id,
                position=Vector3(px, py, pz),
                orientation=Quaternion(qx, qy, qz, qw),
            )
            nodes.append(cls.Node3D(pose=pose, id=node_id, metadata_id=metadata_id))

        edges: list[Graph3D.Edge] = []
        for _ in range(edge_count):
            start_id, end_id, edge_ts, edge_metadata_id = struct.unpack_from(">QQdQ", buf, offset)
            offset += 32
            edges.append(
                cls.Edge(
                    start_id=start_id,
                    end_id=end_id,
                    timestamp=edge_ts,
                    metadata_id=edge_metadata_id,
                )
            )

        return cls(ts=graph_ts, nodes=nodes, edges=edges)

    def to_rerun(
        self,
        z_offset: float = 0.0,
        radii: float = 0.12,
        node_colors: dict[int, tuple[int, int, int, int]] | None = None,
    ) -> Archetype:
        """Default visualization: ``rr.Points3D`` of just the nodes.

        For nodes + edges in separate entity sub-paths, use
        ``to_rerun_multi`` from a ``visual_override`` callback.
        """
        import rerun as rr

        nc = node_colors if node_colors is not None else _DEFAULT_NODE_COLORS
        positions = [
            [n.pose.position.x, n.pose.position.y, n.pose.position.z + z_offset] for n in self.nodes
        ]
        colors = [nc.get(n.metadata_id, _DEFAULT_NODE_COLOR) for n in self.nodes]
        node_radii = [radii * 2.0 if n.metadata_id in (1, 2) else radii for n in self.nodes]
        return rr.Points3D(positions, colors=colors, radii=node_radii)

    def to_rerun_multi(
        self,
        base_path: str,
        z_offset: float = 0.0,
        node_radius: float = 0.12,
        edge_radius: float = 0.04,
        node_colors: dict[int, tuple[int, int, int, int]] | None = None,
        edge_colors: dict[int, tuple[int, int, int, int]] | None = None,
    ) -> list[tuple[str, Archetype]]:
        """Return ``[(base_path/nodes, Points3D), (base_path/edges, LineStrips3D)]``.

        Intended for use from ``visual_override`` callbacks where the
        bridge supports the ``RerunMulti`` list-of-tuples form.
        """
        import rerun as rr

        nc = node_colors if node_colors is not None else _DEFAULT_NODE_COLORS
        ec = edge_colors if edge_colors is not None else _DEFAULT_EDGE_COLORS

        node_positions = [
            [n.pose.position.x, n.pose.position.y, n.pose.position.z + z_offset] for n in self.nodes
        ]
        node_colors_list = [nc.get(n.metadata_id, _DEFAULT_NODE_COLOR) for n in self.nodes]
        node_radii = [
            node_radius * 2.0 if n.metadata_id in (1, 2) else node_radius for n in self.nodes
        ]
        nodes_archetype = rr.Points3D(node_positions, colors=node_colors_list, radii=node_radii)

        id_to_pose: dict[int, PoseStamped] = {n.id: n.pose for n in self.nodes}
        strips: list[list[list[float]]] = []
        edge_colors_list: list[tuple[int, int, int, int]] = []
        for edge in self.edges:
            start = id_to_pose.get(edge.start_id)
            end = id_to_pose.get(edge.end_id)
            if start is None or end is None:
                continue
            strips.append(
                [
                    [start.position.x, start.position.y, start.position.z + z_offset],
                    [end.position.x, end.position.y, end.position.z + z_offset],
                ]
            )
            edge_colors_list.append(ec.get(edge.metadata_id, _DEFAULT_EDGE_COLOR))
        edges_archetype = rr.LineStrips3D(
            strips, colors=edge_colors_list, radii=[edge_radius] * len(strips)
        )

        return [
            (f"{base_path}/nodes", nodes_archetype),
            (f"{base_path}/edges", edges_archetype),
        ]

    def __len__(self) -> int:
        return len(self.nodes)

    def __str__(self) -> str:
        return f"Graph3D(nodes={len(self.nodes)}, edges={len(self.edges)})"
