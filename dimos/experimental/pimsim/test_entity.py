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

"""Wire-format tests for the entity contract. The payload is consumed by
hand-written decoders (Rust, browser) — these tests pin the schema."""

from __future__ import annotations

import json
import struct

from dimos.experimental.pimsim.entity import EntityDescriptor, EntityStateBatch
from dimos.msgs.geometry_msgs.Pose import Pose


def _batch() -> EntityStateBatch:
    return EntityStateBatch(
        entries=[
            (
                EntityDescriptor(
                    entity_id="crate",
                    kind="dynamic",
                    shape_hint="box",
                    extents=(0.1, 0.4, 0.4),
                    mass=1.5,
                    rgba=(0.8, 0.2, 0.2, 1.0),
                ),
                Pose(0.4, 0.0, 0.1, 0.0, 0.0, 0.7071068, 0.7071068),
            ),
            (
                EntityDescriptor(entity_id="shelf", kind="static", mesh_ref="shelf.glb"),
                Pose(1.0, -0.5, 0.0),
            ),
        ],
        ts=123.456,
    )


def test_batch_roundtrip() -> None:
    decoded = EntityStateBatch.decode(_batch().encode())
    assert decoded.ts == 123.456
    assert len(decoded.entries) == 2

    crate, crate_pose = decoded.entries[0]
    assert crate == _batch().entries[0][0]  # frozen dataclass equality
    assert (crate_pose.position.x, crate_pose.position.y, crate_pose.position.z) == (
        0.4,
        0.0,
        0.1,
    )
    assert abs(crate_pose.orientation.z - 0.7071068) < 1e-9
    assert abs(crate_pose.orientation.w - 0.7071068) < 1e-9

    shelf, _ = decoded.entries[1]
    assert shelf.mesh_ref == "shelf.glb"
    assert shelf.rgba is None
    assert shelf.extents == ()


def test_payload_schema_is_versioned_and_length_prefixed() -> None:
    """External decoders parse this by hand — pin the envelope and keys."""
    raw = _batch().encode()
    (length,) = struct.unpack(">I", raw[:4])
    payload = json.loads(raw[4 : 4 + length].decode())

    assert payload["version"] == 1
    assert payload["ts"] == 123.456
    entry = payload["entities"][0]
    assert set(entry) == {"id", "kind", "mesh_ref", "shape", "extents", "mass", "pose", "rgba"}
    assert set(entry["pose"]) == {"x", "y", "z", "qw", "qx", "qy", "qz"}


def test_decode_ignores_unknown_keys() -> None:
    """Forward compatibility: producers may add fields before consumers learn them."""
    raw = _batch().encode()
    (length,) = struct.unpack(">I", raw[:4])
    payload = json.loads(raw[4 : 4 + length].decode())
    payload["future_field"] = {"nested": True}
    payload["entities"][0]["future_key"] = 42
    body = json.dumps(payload).encode()
    mutated = struct.pack(">I", len(body)) + body

    decoded = EntityStateBatch.decode(mutated)
    assert decoded.entries[0][0].entity_id == "crate"


def test_descriptor_wire_roundtrip() -> None:
    desc = EntityDescriptor(entity_id="cup", shape_hint="cylinder", extents=(0.03, 0.1))
    assert EntityDescriptor.from_wire(desc.to_wire()) == desc


def test_lcm_hooks_match_encode() -> None:
    batch = _batch()
    assert batch.lcm_encode() == batch.encode()
    assert EntityStateBatch.lcm_decode(batch.encode()).ts == batch.ts
