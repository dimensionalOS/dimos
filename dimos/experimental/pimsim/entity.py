# Copyright 2025-2026 Dimensional Inc.
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

"""Entity primitives for the experimental browser-physics sim.

``EntityDescriptor`` — what an entity *is* (mesh, kind, mass). Stable.
``EntityState``      — where an entity *is* (timestamped pose + twist).

Both flow through ``BabylonSceneViewerModule``. The browser is the
authoritative source for state once an entity is spawned; the module
mirrors the table for reconnect replay and republishes upstream.
"""

from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
import json
import struct
import time
from typing import Any, Literal

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped

EntityKind = Literal["dynamic", "kinematic", "static"]
ShapeHint = Literal["mesh", "box", "sphere", "cylinder"]


@dataclass(frozen=True)
class EntityDescriptor:
    """An entity to add to the sim world.

    Attributes:
        entity_id: Stable identifier.
        kind: ``dynamic`` (physics-driven), ``kinematic`` (program-driven via
            RPC), or ``static`` (fixed in place — useful for runtime-added
            geometry that wasn't in the cooked scene).
        mesh_ref: GLB path or URL. Same artifact format the cooked browser
            scene uses; consumed by both the Babylon viewer (rendering) and
            any downstream collision consumer.
        shape_hint: Physics shape. ``mesh`` falls back to the GLB triangles;
            primitives (``box``/``sphere``/``cylinder``) ignore the mesh
            and use ``extents`` instead.
        extents: Primitive parameters. Box: ``(w, h, d)``; sphere: ``(r,)``;
            cylinder: ``(r, h)``. Ignored for ``shape_hint == "mesh"``.
        mass: kg. Zero forces kinematic behavior regardless of ``kind``.
    """

    entity_id: str
    kind: EntityKind = "kinematic"
    mesh_ref: str = ""
    shape_hint: ShapeHint = "mesh"
    extents: tuple[float, ...] = ()
    mass: float = 0.0

    def to_wire(self) -> dict[str, Any]:
        return {
            "entity_id": self.entity_id,
            "kind": self.kind,
            "mesh_ref": self.mesh_ref,
            "shape_hint": self.shape_hint,
            "extents": list(self.extents),
            "mass": float(self.mass),
        }

    @classmethod
    def from_wire(cls, data: dict[str, Any]) -> EntityDescriptor:
        return cls(
            entity_id=str(data["entity_id"]),
            kind=data.get("kind", "kinematic"),
            mesh_ref=str(data.get("mesh_ref", "")),
            shape_hint=data.get("shape_hint", "mesh"),
            extents=tuple(float(x) for x in data.get("extents", [])),
            mass=float(data.get("mass", 0.0)),
        )


class EntityState(Timestamped):
    """Per-tick state for a single entity, sourced from the authoritative sim."""

    entity_id: str
    frame_id: str
    pose: Pose
    twist: Twist

    def __init__(
        self,
        ts: float,
        entity_id: str,
        pose: Pose,
        twist: Twist | None = None,
        frame_id: str = "world",
    ) -> None:
        super().__init__(ts)
        self.entity_id = entity_id
        self.frame_id = frame_id
        self.pose = pose
        self.twist = twist if twist is not None else Twist()

    def to_wire(self) -> dict[str, Any]:
        return {
            "entity_id": self.entity_id,
            "frame_id": self.frame_id,
            "ts": self.ts,
            "pose": pose_to_wire(self.pose),
            "twist": twist_to_wire(self.twist),
        }

    @classmethod
    def from_wire(cls, data: dict[str, Any]) -> EntityState:
        return cls(
            ts=float(data.get("ts", 0.0)),
            entity_id=str(data["entity_id"]),
            frame_id=str(data.get("frame_id", "world")),
            pose=pose_from_wire(data["pose"]),
            twist=twist_from_wire(data.get("twist", {})),
        )


def pose_to_wire(p: Pose) -> dict[str, float]:
    return {
        "x": float(p.position.x),
        "y": float(p.position.y),
        "z": float(p.position.z),
        "qw": float(p.orientation.w),
        "qx": float(p.orientation.x),
        "qy": float(p.orientation.y),
        "qz": float(p.orientation.z),
    }


def pose_from_wire(d: dict[str, Any]) -> Pose:
    p = Pose()
    p.position = Vector3(
        float(d.get("x", 0.0)),
        float(d.get("y", 0.0)),
        float(d.get("z", 0.0)),
    )
    # Quaternion ctor takes (x, y, z, w).
    p.orientation = Quaternion(
        float(d.get("qx", 0.0)),
        float(d.get("qy", 0.0)),
        float(d.get("qz", 0.0)),
        float(d.get("qw", 1.0)),
    )
    return p


def twist_to_wire(t: Twist) -> dict[str, float]:
    return {
        "lx": float(t.linear.x),
        "ly": float(t.linear.y),
        "lz": float(t.linear.z),
        "ax": float(t.angular.x),
        "ay": float(t.angular.y),
        "az": float(t.angular.z),
    }


def twist_from_wire(d: dict[str, Any]) -> Twist:
    return Twist(
        Vector3(
            float(d.get("lx", 0.0)),
            float(d.get("ly", 0.0)),
            float(d.get("lz", 0.0)),
        ),
        Vector3(
            float(d.get("ax", 0.0)),
            float(d.get("ay", 0.0)),
            float(d.get("az", 0.0)),
        ),
    )


class EntityStateBatch(Timestamped):
    """Aggregated snapshot: every entity with its descriptor + current pose.

    Wire format: JSON over LCM bytes (length-prefixed string payload),
    matching the EntityMarkers pattern so the rust scene_lidar can
    consume it via a custom decode without needing a new lcm_msgs type.
    """

    msg_name = "pimsim.EntityStateBatch"

    def __init__(
        self,
        entries: list[tuple[EntityDescriptor, Pose]] | None = None,
        ts: float | None = None,
    ) -> None:
        super().__init__(ts if ts is not None else time.time())
        self.entries: list[tuple[EntityDescriptor, Pose]] = entries or []

    def _payload(self) -> dict[str, Any]:
        return {
            "ts": self.ts,
            "entities": [
                {
                    "id": d.entity_id,
                    "kind": d.kind,
                    "shape": d.shape_hint,
                    "extents": list(d.extents),
                    "mass": float(d.mass),
                    "pose": pose_to_wire(p),
                }
                for d, p in self.entries
            ],
        }

    def encode(self) -> bytes:
        body = json.dumps(self._payload()).encode()
        buf = BytesIO()
        buf.write(struct.pack(">I", len(body)))
        buf.write(body)
        return buf.getvalue()

    @classmethod
    def decode(cls, data: bytes) -> EntityStateBatch:
        buf = BytesIO(data)
        (length,) = struct.unpack(">I", buf.read(4))
        payload = json.loads(buf.read(length).decode())
        entries: list[tuple[EntityDescriptor, Pose]] = []
        for entry in payload.get("entities", []):
            desc = EntityDescriptor(
                entity_id=str(entry["id"]),
                kind=entry.get("kind", "kinematic"),
                mesh_ref="",  # not transported — lidar doesn't need GLB path
                shape_hint=entry.get("shape", "mesh"),
                extents=tuple(float(x) for x in entry.get("extents", [])),
                mass=float(entry.get("mass", 0.0)),
            )
            entries.append((desc, pose_from_wire(entry["pose"])))
        return cls(entries=entries, ts=float(payload.get("ts", 0.0)))

    # LCM transport hooks — same shape as EntityMarkers so autoconnect
    # wires LCMTransport without needing a generated msg class.
    def lcm_encode(self) -> bytes:
        return self.encode()

    @classmethod
    def lcm_decode(cls, data: bytes, **_: object) -> EntityStateBatch:
        return cls.decode(data)


__all__ = [
    "EntityDescriptor",
    "EntityKind",
    "EntityState",
    "EntityStateBatch",
    "ShapeHint",
    "pose_from_wire",
    "pose_to_wire",
    "twist_from_wire",
    "twist_to_wire",
]
