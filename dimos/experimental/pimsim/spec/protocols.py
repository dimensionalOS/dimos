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

"""PimSim protocols — the interface every simulator backend implements.

PimSim's thesis: make the *physics backend* a pluggable role so Dimos talks to
a simulator exactly the way it talks to real hardware — same topics, same
clients, same blueprint. Dimos never learns which backend (or whether it is a
sim at all) is on the other side.

Two backends exist today and **both implement the protocols below**:

    MujocoSimModule            headless, deterministic — the eval/CI authority
    BabylonSceneViewerModule   browser + Havok — interactive, high visual fidelity

A new simulator joins PimSim by implementing these — nothing else changes.

Three protocols, by role:

* ``PhysicsAuthority`` — the **backend**. Owns a scene + an embodiment, steps
  physics on its own clock, and publishes the authority-agnostic streams. This
  is the interface a new simulator must satisfy.
* ``EntityConsumer`` — anything that **reads** the scene stream (the planning
  world, the rust lidar, a splat camera, a mirror-mode viewer). Blind to the
  backend.
* ``SceneControl`` — the out-of-process **control client** (scripts, tests,
  evals) that authors and drives a scene the same way regardless of backend.
  ``PimSimClient`` and ``DimSimClient`` both implement it; an e2e test
  parametrizes over backends and the test body is byte-for-byte identical —
  which is exactly the "sim == hardware" property.

The data shapes these traffic in live in ``models.py`` / ``../entity.py`` /
``simulation/scene_assets/spec.py`` (``ScenePackage``): one description
(``SceneObject`` / ``EntityDescriptor``), one streaming snapshot
(``EntityStateBatch``), one portable package (``ScenePackage``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.experimental.pimsim.entity import EntityDescriptor, EntityStateBatch
from dimos.experimental.pimsim.spec.enums import AuthorityMode

if TYPE_CHECKING:
    # Typing-only so the spec stays import-light. ``In``/``Out`` are the dimos
    # stream ports; the geometry msgs are the wire types; ``SceneObject`` is the
    # proposed unified scene noun (see models.py / §7-A).
    from dimos.core.stream import In, Out
    from dimos.experimental.pimsim.spec.models import SceneObject
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Twist import Twist


# ─────────────────────────────────────────────────────────────────────────
# The backend: a pluggable physics authority
# ─────────────────────────────────────────────────────────────────────────


@runtime_checkable
class PhysicsAuthority(Protocol):
    """A pluggable simulation backend — the interface a new simulator implements.

    A backend ingests a cooked ``ScenePackage`` and an embodiment at
    construction (from config, not via a call), steps physics on its own clock,
    and exposes the surface below. ``MujocoSimModule`` and
    ``BabylonSceneViewerModule`` both satisfy it; Havok and MuJoCo differ only in
    *which optional capabilities* they add (see below), never in this core.

    **Required of every backend** (the authority-agnostic contract Dimos binds
    to — identical to a real robot's topics):

    * ``entity_state_batch`` — a fresh snapshot of every scene entity
      (descriptor + world pose) published each tick. THE contract every
      consumer depends on.
    * ``odom`` — the embodiment's base pose in the world frame.
    * ``cmd_vel`` — the mobile-base velocity command the backend integrates.
      (Articulated/arm control flows through the ``ControlCoordinator``'s
      ``joint_command`` instead, which is likewise backend-blind — so it is not
      part of this protocol.)
    * ``authority_mode`` — ``OWNS`` (simulates, is the source of truth) or
      ``MIRROR`` (renders another authority's stream; a viewer, not a sim).
    * ``spawn_entity`` — inject a body at runtime beyond the cooked package.

    **Optional capabilities** (backend-dependent — declare via
    ``capabilities``; do NOT assume them):

    * RGBD / lidar / IMU sensors — ``MujocoSimModule`` publishes
      ``color_image`` / ``depth_image`` / ``camera_info`` / ``pointcloud`` /
      ``imu``; the Babylon viewer does not (splat/mesh cameras are *separate*
      ``EntityConsumer`` modules that composite the same stream).
    * Faster-than-realtime / determinism — MuJoCo headless; Babylon is locked to
      wall-clock for interactivity.

    A consumer must depend only on the required surface; reach for a capability
    only after checking ``capabilities``.
    """

    entity_state_batch: Out[EntityStateBatch]
    """Per-tick snapshot of every entity (descriptor + world pose). Live when
    ``authority_mode == OWNS``; a MIRROR instance consumes this instead."""

    odom: Out[PoseStamped]
    """The embodiment's base pose in the world frame."""

    cmd_vel: In[Twist]
    """The mobile-base velocity command the backend integrates each tick — the
    authority's SOLE velocity input, identical to a real robot's base.

    Every velocity *source* (browser/keyboard teleop, the nav planner's
    ``PathFollower``, an agentic skill, ``SceneControl.cmd_vel``) drives the base
    by publishing ``/cmd_vel``; the authority is a pure consumer and is blind to
    which source wrote it. Arbitration is **last-writer-wins** by default — grab
    the keyboard and you override the planner; release and the planner's stream
    resumes. When a blueprint needs richer arbitration (teleop priority, deadman,
    ramping), it inserts a ``MovementManager`` that fuses the sources into the
    single ``/cmd_vel`` the authority consumes. A backend must NOT invent a
    private per-source channel it alone reads (the Babylon viewer historically
    integrated a ``/nav_cmd_vel`` only it subscribed to, so planner/teleop
    commands published on ``/cmd_vel`` silently never moved the sim) — that
    breaks source-blindness and the sim==hardware contract."""

    @property
    def authority_mode(self) -> AuthorityMode:
        """``OWNS`` — this instance simulates and produces the stream.
        ``MIRROR`` — it renders another authority's stream (a viewer)."""
        ...

    @property
    def capabilities(self) -> frozenset[str]:
        """Optional features this backend provides beyond the required surface,
        e.g. ``{"rgbd", "lidar", "imu", "faster_than_realtime", "deterministic",
        "interactive"}``. Consumers check this before using a capability."""
        ...

    def spawn_entity(self, descriptor: EntityDescriptor, pose: Pose) -> bool:
        """Add one body at runtime beyond the cooked package. Returns ``False``
        if rejected — e.g. a ``MIRROR`` instance, which only echoes upstream.

        Both backends should support this; Babylon does (``@rpc``), MuJoCo today
        seeds entities from its ``scene_entities`` config and runtime spawn is a
        known gap to close — not a reason to weaken the contract."""
        ...


# ─────────────────────────────────────────────────────────────────────────
# The reader: a backend-blind consumer of the scene stream
# ─────────────────────────────────────────────────────────────────────────


@runtime_checkable
class EntityConsumer(Protocol):
    """Reads the entity stream and reacts; never references a physics engine, so
    it behaves identically no matter which ``PhysicsAuthority`` is upstream
    (including a MIRROR-mode one). Adding a consumer is "subscribe to the
    stream," not "integrate with the simulator." Consumers today:

    * ``MujocoWorld.sync_entity_poses`` (via the planning ``world_monitor``) —
      streamed poses become collision-world updates.
    * ``SceneLidarModule`` — rust BVH raycast vs the cooked collision GLB + the
      live entities.
    * ``SplatCameraModule`` — composites live entity poses + arm hulls onto a
      Gaussian-splat render.
    * ``BabylonSceneViewerModule`` in ``external`` mode — the viewer that is an
      authority in ``browser`` mode, here a pure reader.

    Bound by topic (``/entity_state_batch``), so the local attribute name is the
    consumer's choice; they key on ``descriptor.entity_id`` for identity across
    ticks and instantiate geometry from ``mesh_ref`` / ``shape_hint`` /
    ``extents`` the first time they see an id.
    """

    entity_states: In[EntityStateBatch]


# ─────────────────────────────────────────────────────────────────────────
# The control client: author + drive a scene, backend-agnostic
# ─────────────────────────────────────────────────────────────────────────


@runtime_checkable
class SceneControl(Protocol):
    """Out-of-process scene control for scripts, tests, and agentic evals.

    This is the surface that makes "sim == hardware" *testable*: a single e2e
    test parametrizes ``sim_client`` over ``("dimsim", "pimsim")`` (and, in
    principle, a real-robot client), receives a ``SceneControl``, and its body
    does not change. ``PimSimClient`` (Babylon, over the LCM-WS bridge) and
    ``DimSimClient`` (DimSim, over its scene bridge) both implement it.

    The methods are deliberately minimal — pose the embodiment, author static
    obstacles, set a navigation goal — because that is the common denominator
    every backend (and a real arena) can honour. Backends MAY add richer authoring
    (e.g. ``PimSimClient.clear_entities`` / live ``spawn_entity``) on top.
    """

    def start(self) -> None:
        """Connect to the running simulator (idempotent)."""
        ...

    def stop(self) -> None:
        """Disconnect."""
        ...

    def set_agent_position(self, x: float, y: float, z: float = ...) -> None:
        """Teleport the embodiment to a world position (test setup)."""
        ...

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None:
        """Author a static wall segment between two world points (collidable)."""
        ...

    def publish_goal(self, x: float, y: float) -> None:
        """Send a navigation goal to the robot's stack (same as a real goal)."""
        ...


# ─────────────────────────────────────────────────────────────────────────
# PROPOSED (DESIGN §7-A) — one scene-object noun shared with the planning world.
# Not a simulator concern per se; documented here because it unifies the scene
# vocabulary the authority streams and the planning world consumes.
# ─────────────────────────────────────────────────────────────────────────


@runtime_checkable
class SceneObjectWorld(Protocol):
    """PROPOSED. Merge planning's ``Obstacle`` and PimSim's ``EntityDescriptor``
    into one ``SceneObject`` noun and give the planning world two verbs, so the
    entity stream and the collision world speak one vocabulary (an
    ``EntityStateBatch`` becomes the streaming form of ``(SceneObject, pose)``).
    The perception ``Object`` (``Detection3D``: pointcloud + mask) stays separate
    and is *converted* into a ``SceneObject`` by the obstacle monitor.
    Joint-agreed across backends — not landed unilaterally.
    """

    def add_object(self, obj: SceneObject, pose: Pose) -> str:
        """Inject NEW geometry (what ``add_obstacle`` does). Returns the id."""
        ...

    def update_object_pose(self, object_id: str, pose: Pose) -> bool:
        """Reposition KNOWN geometry (what ``sync_entity_poses`` does per tick)."""
        ...

    def remove_object(self, object_id: str) -> bool:
        """Remove a previously added object."""
        ...

    def get_objects(self) -> list[SceneObject]:
        """All objects currently in the world."""
        ...


__all__ = [
    "EntityConsumer",
    "PhysicsAuthority",
    "SceneControl",
    "SceneObjectWorld",
]
