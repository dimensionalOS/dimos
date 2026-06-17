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

"""GraphDelta3D: per-node SE(3) transforms about to be applied to a list of nodes.

Two aligned arrays: ``nodes[i]`` is the node, ``transforms[i]`` is the
SE(3) delta about to be applied to it. ``post_pose = transforms[i] *
nodes[i].pose`` is the convention (left-multiply).

Use case: PGO publishes this on ``loop_closure_event`` when iSAM2
smooths the pose graph — ``nodes[i]`` is the keyframe pre-smooth,
``transforms[i]`` is the delta iSAM2 just applied to it. Consumers can
re-derive post-poses or filter to large deltas.

Wire format mirrors ``Graph3D`` conventions: big-endian, ``Node3D``
serialization shared, ``Transform`` is just 7 f8s (translation +
quaternion). Custom binary, dispatched by the ``#nav_msgs.GraphDelta3D``
channel-name suffix.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import TYPE_CHECKING, BinaryIO

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


class GraphDelta3D(Timestamped):
    msg_name = "nav_msgs.GraphDelta3D"

    # Reuse Graph3D's nested Node3D for wire-format consistency. A
    # GraphDelta3D[i].node is byte-identical to a Graph3D.nodes[i].
    Node3D = Graph3D.Node3D

    @dataclass
    class Transform:
        """SE(3) transform — translation + rotation quaternion (xyzw)."""

        translation: Vector3
        rotation: Quaternion

    ts: float
    nodes: list[Graph3D.Node3D]
    transforms: list[Transform]

    def __init__(
        self,
        ts: float = 0.0,
        nodes: list[Graph3D.Node3D] | None = None,
        transforms: list[Transform] | None = None,
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.nodes = nodes if nodes is not None else []
        self.transforms = transforms if transforms is not None else []
        if len(self.nodes) != len(self.transforms):
            raise ValueError(
                f"nodes ({len(self.nodes)}) and transforms ({len(self.transforms)}) "
                "must be the same length — they're aligned arrays"
            )

    def lcm_encode(self) -> bytes:
        parts: list[bytes] = []
        parts.append(struct.pack(">Qd", len(self.nodes), self.ts))
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
        for transform in self.transforms:
            parts.append(
                struct.pack(
                    ">7d",
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                )
            )
        return b"".join(parts)

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> GraphDelta3D:
        buf = data if isinstance(data, (bytes, bytearray)) else data.read()
        offset = 0
        node_count, graph_ts = struct.unpack_from(">Qd", buf, offset)
        offset += 16

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
            nodes.append(Graph3D.Node3D(pose=pose, id=node_id, metadata_id=metadata_id))

        transforms: list[GraphDelta3D.Transform] = []
        for _ in range(node_count):
            tx, ty, tz, qx, qy, qz, qw = struct.unpack_from(">7d", buf, offset)
            offset += 56
            transforms.append(
                cls.Transform(
                    translation=Vector3(tx, ty, tz),
                    rotation=Quaternion(qx, qy, qz, qw),
                )
            )

        return cls(ts=graph_ts, nodes=nodes, transforms=transforms)

    def to_rerun(
        self,
        z_offset: float = 0.0,
        arrow_scale: float = 1.0,
    ) -> Archetype:
        """Render each (node, transform) pair as an arrow from node.pose to post_pose.

        The arrow origin is the node's current position; the vector is
        the translation component of the transform (scaled by
        ``arrow_scale``). Rotation deltas aren't visualized by default —
        callers wanting to see those can subclass.
        """
        import rerun as rr

        if not self.nodes:
            return rr.Arrows3D(origins=[], vectors=[])

        origins = []
        vectors = []
        for node, transform in zip(self.nodes, self.transforms, strict=True):
            origins.append(
                [
                    node.pose.position.x,
                    node.pose.position.y,
                    node.pose.position.z + z_offset,
                ]
            )
            vectors.append(
                [
                    transform.translation.x * arrow_scale,
                    transform.translation.y * arrow_scale,
                    transform.translation.z * arrow_scale,
                ]
            )
        return rr.Arrows3D(origins=origins, vectors=vectors)

    def __len__(self) -> int:
        return len(self.nodes)

    def __str__(self) -> str:
        return f"GraphDelta3D(nodes={len(self.nodes)})"
