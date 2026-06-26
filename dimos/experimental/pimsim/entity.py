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

"""Scene-entity contract: what exists in a scene, and where it is now.

``EntityDescriptor`` — what an entity *is* (shape, mass, mesh). Stable
metadata; consumed at world-build time (``add_entities_to_spec`` turns
descriptors into collision bodies).

``EntityStateBatch`` — where every entity *is* (descriptor + pose
snapshot). Streamed by whatever owns physics truth — a simulator today,
a perception stack later — and consumed by anything that mirrors the
scene (the planning world's ``sync_entity_poses``, viewers, recorders).
Producers and consumers are deliberately decoupled: the batch is their
only coupling.

Wire format: length-prefixed JSON over LCM bytes (the
``visualization_msgs.EntityMarkers`` pattern) — no generated lcm_msgs
type needed, hand-decodable from Rust/browser consumers, and tolerant
of schema growth (decoders ignore unknown keys; ``version`` stamps the
payload).
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
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped

EntityKind = Literal["dynamic", "kinematic", "static"]
ShapeHint = Literal["mesh", "box", "sphere", "cylinder"]

_WIRE_VERSION = 1


@dataclass(frozen=True)
class EntityDescriptor:
    """An entity to add to the sim world.

    Attributes:
        entity_id: Stable identifier.
        kind: ``dynamic`` (physics-driven), ``kinematic`` (program-driven via
            RPC), or ``static`` (fixed in place — useful for runtime-added
            geometry that wasn't in the cooked scene).
        mesh_ref: GLB path or URL. Same artifact format the cooked scene
            uses; consumed by renderers and any downstream collision
            consumer.
        shape_hint: Physics shape. ``mesh`` falls back to the GLB triangles;
            primitives (``box``/``sphere``/``cylinder``) ignore the mesh
            and use ``extents`` instead.
        extents: Primitive parameters. Box: ``(w, h, d)``; sphere: ``(r,)``;
            cylinder: ``(r, h)``. Ignored for ``shape_hint == "mesh"``.
        mass: kg. Zero forces kinematic behavior regardless of ``kind``.
        rgba: Optional display color for primitive shapes (renderers fall
            back to their default material when unset).
    """

    entity_id: str
    kind: EntityKind = "kinematic"
    mesh_ref: str = ""
    shape_hint: ShapeHint = "mesh"
    extents: tuple[float, ...] = ()
    mass: float = 0.0
    rgba: tuple[float, float, float, float] | None = None

    def to_wire(self) -> dict[str, Any]:
        wire: dict[str, Any] = {
            "entity_id": self.entity_id,
            "kind": self.kind,
            "mesh_ref": self.mesh_ref,
            "shape_hint": self.shape_hint,
            "extents": list(self.extents),
            "mass": float(self.mass),
        }
        if self.rgba is not None:
            wire["rgba"] = list(self.rgba)
        return wire

    @classmethod
    def from_wire(cls, data: dict[str, Any]) -> EntityDescriptor:
        raw_rgba = data.get("rgba")
        rgba: tuple[float, float, float, float] | None = None
        if isinstance(raw_rgba, list | tuple) and len(raw_rgba) == 4:
            rgba = tuple(float(v) for v in raw_rgba)  # type: ignore[assignment]
        return cls(
            entity_id=str(data["entity_id"]),
            kind=data.get("kind", "kinematic"),
            mesh_ref=str(data.get("mesh_ref", "")),
            shape_hint=data.get("shape_hint", "mesh"),
            extents=tuple(float(x) for x in data.get("extents", [])),
            mass=float(data.get("mass", 0.0)),
            rgba=rgba,
        )


# JSON spelling of a Pose for this wire format. A canonical
# ``Pose.to_dict``/``from_dict`` in dimos/msgs would supersede these —
# until then, keep this the only JSON spelling entity payloads use.
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


class EntityStateBatch(Timestamped):
    """Aggregated snapshot: every entity with its descriptor + current pose.

    Wire format: JSON over LCM bytes (length-prefixed string payload).
    Note ``msg_name`` is wire-visible — LCM channel keys derive from it —
    so it is namespaced for the type's eventual home under dimos/msgs,
    not its current module path.
    """

    msg_name = "simulation_msgs.EntityStateBatch"

    def __init__(
        self,
        entries: list[tuple[EntityDescriptor, Pose]] | None = None,
        ts: float | None = None,
    ) -> None:
        super().__init__(ts if ts is not None else time.time())
        self.entries: list[tuple[EntityDescriptor, Pose]] = entries or []

    def _payload(self) -> dict[str, Any]:
        entities = []
        for d, p in self.entries:
            entry: dict[str, Any] = {
                "id": d.entity_id,
                "kind": d.kind,
                "mesh_ref": d.mesh_ref,
                "shape": d.shape_hint,
                "extents": list(d.extents),
                "mass": float(d.mass),
                "pose": pose_to_wire(p),
            }
            if d.rgba is not None:
                entry["rgba"] = list(d.rgba)
            entities.append(entry)
        return {"version": _WIRE_VERSION, "ts": self.ts, "entities": entities}

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
            raw_rgba = entry.get("rgba")
            rgba: tuple[float, float, float, float] | None = None
            if isinstance(raw_rgba, list | tuple) and len(raw_rgba) == 4:
                rgba = tuple(float(v) for v in raw_rgba)  # type: ignore[assignment]
            desc = EntityDescriptor(
                entity_id=str(entry["id"]),
                kind=entry.get("kind", "kinematic"),
                mesh_ref=str(entry.get("mesh_ref", "")),
                shape_hint=entry.get("shape", "mesh"),
                extents=tuple(float(x) for x in entry.get("extents", [])),
                mass=float(entry.get("mass", 0.0)),
                rgba=rgba,
            )
            entries.append((desc, pose_from_wire(entry["pose"])))
        return cls(entries=entries, ts=float(payload.get("ts", 0.0)))

    # LCM transport hooks — duck-typed by the transport layer (anything
    # with lcm_encode/lcm_decode rides LCM; everything else is pickled).
    def lcm_encode(self) -> bytes:
        return self.encode()

    @classmethod
    def lcm_decode(cls, data: bytes, **_: object) -> EntityStateBatch:
        return cls.decode(data)


__all__ = [
    "EntityDescriptor",
    "EntityKind",
    "EntityStateBatch",
    "ShapeHint",
]
