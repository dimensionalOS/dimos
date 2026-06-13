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

"""Protocol definitions for PimSim — the pluggable-physics simulation layer.

PimSim's thesis (see ``../DESIGN.md``): make *physics authority* a pluggable
role and decouple every other concern from it through three shared contracts,
so any authority can drive any consumer:

    1. SCENE PACKAGE   what geometry exists      -> ScenePackage (concrete,
                       (cooked offline, portable)   simulation/scene_assets/spec.py)
    2. ENTITY STREAM   where everything is now    -> EntityDescriptor /
                       (versioned JSON over LCM)     EntityStateBatch (../entity.py)
    3. LCM BUS         the transport              -> dimos LCM (+ an LCM-over-
                                                      WebSocket bridge for browsers)

Those three are already concrete types. What was only *implicit* in the
sprawling modules — and is named here — are the two roles they play.

Two interface styles appear below, each used where it is faithful:

* **Pub/sub port contracts** (``EntityAuthority``, ``EntityConsumer``). PimSim's
  entity flow is streaming: an authority *publishes* an ``EntityStateBatch``
  every tick on its own clock; no one calls it. So the contract IS the dimos
  port (``Out[...]`` / ``In[...]``), declared as a Protocol attribute, not a
  method you invoke.
* **Method stubs** (``SceneObjectWorld``, and the ``@rpc`` ``spawn_entity``).
  Used for surfaces that are genuinely *called* synchronously — a planner asks
  a world ``add_object(...)``; an operator RPCs ``spawn_entity(...)``. This is
  the ``dimos/manipulation/planning/spec`` ``WorldSpec`` style.

Read the concrete modules *through* these protocols:
    authority producers : BabylonSceneViewerModule (Havok, interactive)
                          MujocoSimModule          (headless, deterministic)
    consumers           : SceneLidarModule, SplatCameraModule,
                          MujocoWorld.sync_entity_poses, the reachability builder
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.experimental.pimsim.entity import EntityDescriptor, EntityStateBatch
from dimos.experimental.pimsim.spec.enums import AuthorityMode

if TYPE_CHECKING:
    # Typing-only: keep the spec import-light. ``In``/``Out`` are dimos stream
    # ports; ``ScenePackage`` is the concrete cooked-geometry dataclass;
    # ``SceneObject`` is the proposed unified noun (see models.py / §7-A).
    from dimos.core.stream import In, Out
    from dimos.experimental.pimsim.spec.models import SceneObject
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


# ─────────────────────────────────────────────────────────────────────────
# The two roles that exist today (streaming — expressed as port contracts)
# ─────────────────────────────────────────────────────────────────────────


@runtime_checkable
class EntityAuthority(Protocol):
    """A pluggable physics authority: owns a scene's physics and broadcasts it.

    An authority ingests a cooked ``ScenePackage`` once (at construction, from
    config — not via a call), then **publishes an ``EntityStateBatch`` every
    tick** onto the LCM bus: a snapshot of every entity's descriptor + world
    pose. Consumers subscribe and never learn which authority is upstream;
    swapping authorities is a blueprint flag and nothing downstream changes.

    Two authorities exist today, complementary rather than rival:

    * ``BabylonSceneViewerModule`` — Havok physics (WASM) in a browser tab;
      interactive, multi-user, grab-and-throw. ``authority_mode == OWNS`` when
      its ``entity_authority`` flag is ``"browser"``.
    * ``MujocoSimModule`` — headless, deterministic, control-grade.

    The contract is the **output port** below — not a method you call. The
    authority drives ``entity_state_batch`` from its own physics loop.
    """

    entity_state_batch: Out[EntityStateBatch]
    """THE contract every consumer depends on: a fresh snapshot of every entity
    (descriptor + world pose) published each tick."""

    odom: Out[PoseStamped]
    """The robot base pose, on the same bus."""

    @property
    def authority_mode(self) -> AuthorityMode:
        """``OWNS`` — simulates physics, is the source of truth.
        ``MIRROR`` — renders another authority's stream as kinematic bodies (a
        viewer). For the Babylon viewer this is ``entity_authority``
        ``"browser"`` vs ``"external"``."""
        ...

    def spawn_entity(self, descriptor: EntityDescriptor, pose: Pose) -> bool:
        """``@rpc`` — add one body at runtime that was not in the cooked
        package. Returns ``False`` if rejected — e.g. a ``MIRROR`` authority,
        which only echoes the upstream stream and does not simulate."""
        ...


@runtime_checkable
class EntityConsumer(Protocol):
    """An authority-blind consumer of the entity stream.

    Subscribes to ``EntityStateBatch`` and reacts; never references a physics
    engine, so it behaves identically no matter which authority is upstream.
    The asymmetry is the architecture working: adding a consumer is "subscribe
    to the stream," not "integrate with the simulator." Current consumers:

    * ``SceneLidarModule`` — BVH raycast vs the cooked collision GLB + the
      dynamic entities from the stream (same scene the sim simulates).
    * ``SplatCameraModule`` — composite live entity poses + arm hulls onto a
      Gaussian-splat render.
    * ``MujocoWorld.sync_entity_poses`` — write streamed poses into the
      planner's collision world.
    * the reachability map builder — sample arm FK against that world.

    The contract is the **input port** below.
    """

    entity_state_batch: In[EntityStateBatch]
    """The subscribed stream. The consumer keys on ``descriptor.entity_id`` for
    identity across ticks, and uses ``mesh_ref`` / ``shape_hint`` / ``extents``
    to instantiate geometry the first time it sees an id; pose updates
    thereafter are just the ``Pose`` half of each entry."""


# ─────────────────────────────────────────────────────────────────────────
# PROPOSED (DESIGN.md §7-A, Decision A) — NOT implemented yet.
# One scene-object noun, two verbs. Method-style because a planner calls it.
# ─────────────────────────────────────────────────────────────────────────


@runtime_checkable
class SceneObjectWorld(Protocol):
    """PROPOSED. Three types describe "a shaped thing at a pose" today:
    ``Obstacle`` (planning collision input), ``EntityDescriptor`` (PimSim scene
    state), and the perception ``Object`` (``Detection3D``: pointcloud + mask).
    Decision A merges the first two into one ``SceneObject`` noun and gives the
    world **two verbs**; perception ``Object`` stays separate (it pulls
    open3d + cv2 and is detector *output*, not spawnable geometry) and is
    *converted into* a ``SceneObject`` by the obstacle monitor.

    This is the planning ``WorldSpec``'s obstacle verbs renamed onto the
    unified noun. The payoff: ``EntityStateBatch`` becomes the *streaming form*
    of ``(SceneObject, pose)``, so the entity stream and the planning world
    speak one vocabulary. Two verbs, not one pipeline, because "inject new
    geometry" and "move known geometry" are genuinely different operations
    (one mutates the body set; one writes a pose).
    """

    def add_object(self, obj: SceneObject, pose: Pose) -> str:
        """Inject NEW geometry — what ``add_obstacle`` does, and what a
        perception detection becomes. Mutates the body set. Returns the id."""
        ...

    def update_object_pose(self, object_id: str, pose: Pose) -> bool:
        """Reposition KNOWN geometry — what ``sync_entity_poses`` does every
        tick from the entity stream. Writes a pose; never adds a body."""
        ...

    def remove_object(self, object_id: str) -> bool:
        """Remove a previously added object. Returns ``True`` if removed."""
        ...

    def get_objects(self) -> list[SceneObject]:
        """All objects currently in the world."""
        ...


__all__ = ["EntityAuthority", "EntityConsumer", "SceneObjectWorld"]
