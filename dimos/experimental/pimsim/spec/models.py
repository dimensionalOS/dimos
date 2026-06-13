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

"""Data contracts the PimSim protocols traffic in.

The nouns that exist today are re-exported from their real definitions so
there is a single source of truth and the protocols read against real types:

    EntityDescriptor   what an entity *is*  — stable identity + how to
                       instantiate: id, kind, mesh_ref, shape_hint + extents,
                       mass, rgba. No pose. (../entity.py)

    EntityStateBatch   where everything *is now* — a timestamped snapshot of
                       ``(EntityDescriptor, Pose)`` entries, restreamed every
                       tick; versioned length-prefixed JSON over LCM so
                       non-Python consumers decode it. (../entity.py)

    ScenePackage       what geometry *exists* — the cooked, portable scene
                       (visual GLB, decimated collision GLB, per-entity GLBs,
                       CoACD hulls, objects.json, MuJoCo wrapper MJCF). A
                       concrete dataclass, imported where you cook/resolve a
                       package. (dimos/simulation/scene_assets/spec.py)

``ScenePackage`` is deliberately NOT re-exported here — importing it pulls the
scene-asset cooking dependencies, which referencing the type should not
require. Import it directly from ``dimos.simulation.scene_assets.spec``.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.experimental.pimsim.entity import (
    EntityDescriptor,
    EntityKind,
    EntityStateBatch,
    ShapeHint,
)

# ─────────────────────────────────────────────────────────────────────────
# PROPOSED (DESIGN.md §7-A, Decision A) — NOT implemented yet.
# ─────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class SceneObject:
    """PROPOSED unified scene noun — the merge of planning's ``Obstacle`` and
    PimSim's ``EntityDescriptor`` into one type for "a shaped thing in the
    scene." Identity + geometry only; **no pose** (pose lives in the world via
    ``SceneObjectWorld``'s verbs and in the ``EntityStateBatch`` stream).

    It is structurally ``EntityDescriptor`` with ``entity_id`` renamed to
    ``object_id`` — which is exactly why the two collapse: they describe the
    same thing. Once merged, ``EntityStateBatch`` becomes a stream of
    ``(SceneObject, Pose)``, so the entity stream and the planning world share
    one vocabulary. The perception ``Object`` (``Detection3D``: pointcloud +
    mask + detector output) stays separate and is *converted into* a
    ``SceneObject`` by the obstacle monitor — it is observation, not spawnable
    geometry, and forcing it here would drag open3d + cv2 into the Rust/browser
    consumers.
    """

    object_id: str
    kind: EntityKind = "kinematic"
    mesh_ref: str = ""
    shape_hint: ShapeHint = "mesh"
    extents: tuple[float, ...] = ()
    mass: float = 0.0
    rgba: tuple[float, float, float, float] | None = None


__all__ = ["EntityDescriptor", "EntityStateBatch", "SceneObject"]
