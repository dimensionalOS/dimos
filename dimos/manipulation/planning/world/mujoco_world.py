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

"""MuJoCo World Implementation - WorldSpec using MuJoCo as a kinematics/collision library.

MuJoCo is never stepped here: the model answers counterfactual queries
(``mj_kinematics``, ``mj_comPos``, ``mj_collision``, ``mj_jac``) for IK and
motion planning. The model is composed the same way ``MujocoSimModule``
composes the simulation — cooked scene wrapper + robot MJCF(s) attached via
``MjSpec`` + scene-package entities with their cooked collision hulls — so
planning, simulation, and rendering share one asset pipeline.

Differences from :class:`DrakeWorld`, by design:

- **Scoped collision checks.** ``is_collision_free(ctx, robot_id)`` only
  considers contacts involving the robot's *moving subtree* (the bodies
  downstream of its planned joints). A standing humanoid always has
  feet-on-floor contacts; scoping makes arm planning well-defined in a full
  scene. Drake's variant checks the whole world (its worlds have no floor).
- **Contexts are cheap.** A context is an ``MjData``; ``scratch_context()``
  is a pooled ``mj_copyData`` from the live context (microseconds, vs
  ~100 ms for Drake's ``CreateDefaultContext`` on a humanoid plant).
- **Scene entities are state, not obstacles.** Cooked entities enter the
  model at :meth:`finalize`; their poses are updated through
  :meth:`sync_entity_poses` (fed from ``/entity_state_batch``), not through
  ``add_obstacle()``.

State setters (``set_joint_state``, ``sync_*``, ``set_floating_base_pose``)
refresh kinematics; getters assume it is fresh.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from threading import RLock
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import JointPath, Obstacle, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

    from dimos.simulation.scene_assets.spec import ScenePackage

try:
    import mujoco

    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False

logger = setup_logger()

_SCRATCH_POOL_MAX = 8
_SLOT_PARK_POS = np.array([0.0, 0.0, -1000.0])
_SLOT_TYPES = {
    ObstacleType.BOX: "box",
    ObstacleType.SPHERE: "sphere",
    ObstacleType.CYLINDER: "cylinder",
}


@dataclass
class _RobotEntry:
    """Internal data for one registered robot (or arm view onto a shared model)."""

    robot_id: WorldRobotID
    config: RobotModelConfig
    prefix: str
    # Resolved at finalize:
    joint_ids: list[int] = field(default_factory=list)
    qpos_adr: Any = None  # NDArray[int], one entry per config joint
    dof_adr: Any = None  # NDArray[int]
    ee_body_id: int = -1
    grasp_offset: Any = None  # NDArray (3,) or None
    root_free_qpos_adr: int | None = None
    # Collision scoping: boolean mask over geom ids for the moving subtree,
    # and excluded geom-id pair keys (i * ngeom + j with i < j).
    check_geom_mask: Any = None  # NDArray[bool], shape (ngeom,)
    excluded_pair_keys: Any = None  # NDArray[int64]


@dataclass
class _ObstacleEntry:
    obstacle_id: str
    obstacle: Obstacle
    body_id: int = -1
    geom_id: int = -1
    slot_type: ObstacleType | None = None  # set when occupying a post-finalize slot
    removed: bool = False


def _pose_to_pos_quat(pose: PoseStamped) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """PoseStamped → (position (3,), quaternion wxyz (4,))."""
    matrix = Transform(translation=pose.position, rotation=pose.orientation).to_matrix()
    quat = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(matrix[:3, :3]).reshape(9))
    return matrix[:3, 3].copy(), quat


class MujocoWorld(WorldSpec):
    """MuJoCo implementation of WorldSpec.

    Args:
        scene_package: Scene to plan in — a catalog name (``"dimos-office"``),
            a package directory / ``scene.meta.json`` path, or a loaded
            ``ScenePackage``. ``None`` for an empty world.
        scene_xml: Explicit scene MJCF (overrides the package's wrapper).
        scene_entities: Explicit entity metadata list (overrides the
            package's entities). Entities are composed with their cooked
            collision hulls via ``add_entities_to_spec``.
        obstacle_slots: Pre-allocated post-finalize obstacle slots per
            primitive type (MuJoCo cannot add bodies after compile).
    """

    def __init__(
        self,
        scene_package: str | Path | ScenePackage | None = None,
        scene_xml: str | Path | None = None,
        scene_entities: list[dict[str, Any]] | None = None,
        obstacle_slots: int = 16,
        enable_viz: bool = False,  # accepted for factory parity; no renderer here
    ) -> None:
        if not MUJOCO_AVAILABLE:
            raise ImportError("MuJoCo is not installed. Install with: pip install mujoco")

        del enable_viz  # planning previews are published, not rendered (see design.md)
        self._lock = RLock()
        self._n_slots = obstacle_slots

        xml, entities = self._resolve_scene(scene_package, scene_xml, scene_entities)
        self._scene_entities = entities
        self._spec = mujoco.MjSpec.from_file(str(xml)) if xml else mujoco.MjSpec()

        self._robots: dict[WorldRobotID, _RobotEntry] = {}
        self._shared_with: dict[WorldRobotID, WorldRobotID] = {}
        self._robot_counter = 0

        self._obstacles: dict[str, _ObstacleEntry] = {}
        self._free_slots: dict[ObstacleType, list[tuple[int, int]]] = {}

        self._model: mujoco.MjModel | None = None
        self._live: mujoco.MjData | None = None
        self._scratch_pool: list[mujoco.MjData] = []

        # entity id → freejoint qpos address (dynamic) / None (static)
        self._entity_qpos_adr: dict[str, int | None] = {}
        self._warned_static_entities: set[str] = set()

        self._finalized = False

    @staticmethod
    def _resolve_scene(
        scene_package: str | Path | ScenePackage | None,
        scene_xml: str | Path | None,
        scene_entities: list[dict[str, Any]] | None,
    ) -> tuple[Path | None, list[dict[str, Any]]]:
        pkg = None
        if scene_package is not None:
            if isinstance(scene_package, str | Path):
                from dimos.simulation.scenes.catalog import resolve_scene_package

                pkg = resolve_scene_package(scene_package)
            else:
                pkg = scene_package
        xml = Path(scene_xml) if scene_xml else (pkg.mujoco_scene_path if pkg else None)
        entities = scene_entities if scene_entities is not None else (pkg.entities if pkg else [])
        return xml, list(entities)

    # ------------------------------------------------------------------
    # Robot management

    def add_robot(
        self,
        config: RobotModelConfig,
        share_model_with: WorldRobotID | None = None,
    ) -> WorldRobotID:
        """Add a robot. With ``share_model_with``, register a second arm as a
        view onto an already-attached model (dual arms on one humanoid MJCF)
        instead of attaching the file twice."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")

        with self._lock:
            self._robot_counter += 1
            robot_id = f"robot_{self._robot_counter}"

            if share_model_with is not None:
                if share_model_with not in self._robots:
                    raise KeyError(
                        f"share_model_with='{share_model_with}' not found "
                        f"(known: {list(self._robots.keys())})"
                    )
                prefix = self._robots[share_model_with].prefix
                self._shared_with[robot_id] = share_model_with
                logger.info(
                    f"Robot '{config.name}' shares model with '{share_model_with}' "
                    f"(no second MJCF attach)"
                )
            else:
                model_path = Path(config.model_path).resolve()
                if not model_path.exists():
                    raise FileNotFoundError(f"Robot model not found: {model_path}")
                spec_robot = mujoco.MjSpec.from_file(str(model_path))
                # Anchor relative asset paths: the parent spec usually has no
                # file directory of its own, so the child's meshdir must be
                # absolute before attach or compile fails to find the STLs.
                meshdir = (
                    Path(config.model_meshdir)
                    if config.model_meshdir
                    else model_path.parent / (spec_robot.meshdir or "")
                )
                spec_robot.meshdir = str(meshdir.resolve())
                if spec_robot.texturedir:
                    spec_robot.texturedir = str(
                        (model_path.parent / spec_robot.texturedir).resolve()
                    )
                prefix = f"{config.name}-"
                pos, quat = _pose_to_pos_quat(config.base_pose)
                frame = self._spec.worldbody.add_frame(pos=pos.tolist(), quat=quat.tolist())
                self._spec.attach(spec_robot, prefix=prefix, frame=frame)

            self._robots[robot_id] = _RobotEntry(robot_id=robot_id, config=config, prefix=prefix)
            return robot_id

    def get_robot_ids(self) -> list[WorldRobotID]:
        return list(self._robots.keys())

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        return self._entry(robot_id).config

    def _entry(self, robot_id: WorldRobotID) -> _RobotEntry:
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")
        return self._robots[robot_id]

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        entry = self._entry(robot_id)
        config = entry.config

        if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
            return np.array(config.joint_limits_lower), np.array(config.joint_limits_upper)

        if self._finalized:
            assert self._model is not None
            lower, upper = [], []
            for joint_name, jid in zip(config.joint_names, entry.joint_ids, strict=True):
                if self._model.jnt_limited[jid]:
                    lo, hi = self._model.jnt_range[jid]
                else:
                    logger.warning(
                        "Joint '%s' has no limits in model; falling back to ±π", joint_name
                    )
                    lo, hi = -np.pi, np.pi
                lower.append(float(lo))
                upper.append(float(hi))
            return np.array(lower), np.array(upper)

        n = len(config.joint_names)
        return np.full(n, -np.pi), np.full(n, np.pi)

    # ------------------------------------------------------------------
    # Finalization

    def finalize(self) -> None:
        """Compose entities + obstacle slots and compile the model."""
        if self._finalized:
            logger.warning("World already finalized")
            return

        with self._lock:
            if self._scene_entities:
                from dimos.simulation.mujoco.entity_scene import add_entities_to_spec

                add_entities_to_spec(self._spec, self._scene_entities)

            self._add_obstacle_slots()

            self._model = self._spec.compile()
            self._live = mujoco.MjData(self._model)

            for entry in self._robots.values():
                self._resolve_robot(entry)
            self._apply_sibling_exclusions()
            self._resolve_entities()
            self._resolve_slots()
            self._resolve_prefinalize_obstacles()

            for entry in self._robots.values():
                if entry.config.home_joints is not None:
                    home = np.asarray(entry.config.home_joints, dtype=np.float64)
                    self._live.qpos[entry.qpos_adr] = home
            self._refresh(self._live)

            self._finalized = True
            logger.info(f"MujocoWorld finalized with {len(self._robots)} robots")

    @property
    def is_finalized(self) -> bool:
        return self._finalized

    def _resolve_robot(self, entry: _RobotEntry) -> None:
        assert self._model is not None
        model = self._model
        config = entry.config

        joint_ids: list[int] = []
        for joint_name in config.joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, entry.prefix + joint_name)
            if jid < 0:
                raise ValueError(f"Joint '{entry.prefix + joint_name}' not found in compiled model")
            if model.jnt_type[jid] not in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                raise ValueError(f"Joint '{joint_name}' is not a 1-DOF joint")
            joint_ids.append(jid)
        entry.joint_ids = joint_ids
        entry.qpos_adr = np.array([model.jnt_qposadr[j] for j in joint_ids], dtype=int)
        entry.dof_adr = np.array([model.jnt_dofadr[j] for j in joint_ids], dtype=int)

        ee_name = entry.prefix + config.end_effector_link
        entry.ee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name)
        if entry.ee_body_id < 0:
            raise ValueError(f"End-effector body '{ee_name}' not found in compiled model")

        offset = np.asarray(config.grasp_offset_xyz, dtype=np.float64)
        entry.grasp_offset = offset if np.any(offset != 0.0) else None

        # Floating base: the robot MJCF carries its own freejoint (weld_base=False).
        entry.root_free_qpos_adr = None
        if not config.weld_base:
            for jid in range(model.njnt):
                if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE:
                    continue
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or ""
                if name.startswith(entry.prefix):
                    entry.root_free_qpos_adr = int(model.jnt_qposadr[jid])
                    break
            if entry.root_free_qpos_adr is None:
                logger.warning(
                    "Robot '%s': weld_base=False but its MJCF has no free joint; "
                    "set_floating_base_pose will be a no-op",
                    config.name,
                )

        # Moving subtree: bodies whose ancestry passes through any planned
        # joint's child body. Collisions are only flagged for their geoms.
        chain_bodies = {int(model.jnt_bodyid[j]) for j in joint_ids}
        subtree: set[int] = set()
        for body_id in range(model.nbody):
            b = body_id
            while b != 0:
                if b in chain_bodies:
                    subtree.add(body_id)
                    break
                b = int(model.body_parentid[b])
        mask = np.zeros(model.ngeom, dtype=bool)
        for body_id in subtree:
            adr, num = model.body_geomadr[body_id], model.body_geomnum[body_id]
            mask[adr : adr + num] = True
        entry.check_geom_mask = mask

        # Adjacent-link exclusions (Drake filter parity). MuJoCo's native
        # parent-child contact filter does not apply when the parent body is
        # welded to the world — so a welded-base arm's first link would be
        # checked against its own base, which overlaps at the joint in most
        # URDF-derived models. Exclude every jointed body ↔ its parent,
        # except when the parent is the worldbody (keep floor contacts).
        keys: list[int] = []
        for body_id in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            if not name.startswith(entry.prefix) or model.body_jntnum[body_id] == 0:
                continue
            parent_id = int(model.body_parentid[body_id])
            if parent_id == 0:
                continue
            adr, num = model.body_geomadr[body_id], model.body_geomnum[body_id]
            padr, pnum = model.body_geomadr[parent_id], model.body_geomnum[parent_id]
            if num and pnum:
                keys.extend(
                    self._pair_keys(np.arange(adr, adr + num), np.arange(padr, padr + pnum))
                )

        # User exclusions: body-name pairs → geom-pair keys.
        for name1, name2 in config.collision_exclusion_pairs:
            g1 = self._body_geoms(entry.prefix + name1)
            g2 = self._body_geoms(entry.prefix + name2)
            if g1 is None or g2 is None:
                logger.warning(f"Collision exclusion: link not found: {name1} or {name2}")
                continue
            keys.extend(self._pair_keys(g1, g2))
        entry.excluded_pair_keys = np.array(sorted(set(keys)), dtype=np.int64)

    def _body_geoms(self, body_name: str) -> NDArray[np.intp] | None:
        assert self._model is not None
        bid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            return None
        adr, num = self._model.body_geomadr[bid], self._model.body_geomnum[bid]
        return np.arange(adr, adr + num)

    def _pair_keys(self, geoms_a: NDArray[np.intp], geoms_b: NDArray[np.intp]) -> list[int]:
        assert self._model is not None
        ngeom = self._model.ngeom
        keys = []
        for a in geoms_a:
            for b in geoms_b:
                i, j = (int(a), int(b)) if a < b else (int(b), int(a))
                keys.append(i * ngeom + j)
        return keys

    def _apply_sibling_exclusions(self) -> None:
        """Arms sharing one model don't collision-check against each other
        (same policy as DrakeWorld): each arm still checks against the shared
        torso/legs, the scene, and obstacles. Reactive arm-arm avoidance is
        the IK layer's job."""
        by_prefix: dict[str, list[_RobotEntry]] = {}
        for entry in self._robots.values():
            by_prefix.setdefault(entry.prefix, []).append(entry)
        for entries in by_prefix.values():
            for i in range(len(entries)):
                for j in range(i + 1, len(entries)):
                    a, b = entries[i], entries[j]
                    geoms_a = np.flatnonzero(a.check_geom_mask)
                    geoms_b = np.flatnonzero(b.check_geom_mask)
                    keys = self._pair_keys(geoms_a, geoms_b)
                    for entry in (a, b):
                        entry.excluded_pair_keys = np.union1d(
                            entry.excluded_pair_keys, np.array(keys, dtype=np.int64)
                        )
                    logger.info(
                        f"Sibling-arm collision exclusion: {a.robot_id} ↔ {b.robot_id} "
                        f"({len(keys)} geom pairs)"
                    )

    def _resolve_entities(self) -> None:
        assert self._model is not None
        from dimos.simulation.mujoco.entity_scene import entity_body_name

        for entity in self._scene_entities:
            entity_id = str(entity.get("id"))
            jname = f"{entity_body_name(entity_id)}:free"
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                self._entity_qpos_adr[entity_id] = int(self._model.jnt_qposadr[jid])
            elif (
                mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_BODY, entity_body_name(entity_id)
                )
                >= 0
            ):
                self._entity_qpos_adr[entity_id] = None  # welded static

    # ------------------------------------------------------------------
    # Contexts

    def get_live_context(self) -> mujoco.MjData:
        """Live context (mirrors current robot/entity state).

        WARNING: not thread-safe for reads during writes — use
        scratch_context() for planning operations."""
        if not self._finalized or self._live is None:
            raise RuntimeError("World must be finalized first")
        return self._live

    @contextmanager
    def scratch_context(self) -> Generator[mujoco.MjData, None, None]:
        """Thread-safe context for planning, initialized from the live state."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        assert self._model is not None and self._live is not None

        with self._lock:
            ctx = self._scratch_pool.pop() if self._scratch_pool else mujoco.MjData(self._model)
            mujoco.mj_copyData(ctx, self._model, self._live)
        try:
            yield ctx
        finally:
            with self._lock:
                if len(self._scratch_pool) < _SCRATCH_POOL_MAX:
                    self._scratch_pool.append(ctx)

    def _refresh(self, data: mujoco.MjData) -> None:
        assert self._model is not None
        mujoco.mj_kinematics(self._model, data)
        mujoco.mj_comPos(self._model, data)

    # ------------------------------------------------------------------
    # State operations

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Sync live context from the driver's joint state (positions in
        config.joint_names order, matching DrakeWorld)."""
        if not self._finalized:
            return  # silently ignore before finalization
        assert self._live is not None
        entry = self._entry(robot_id)
        positions = np.asarray(joint_state.position, dtype=np.float64)
        with self._lock:
            self._live.qpos[entry.qpos_adr] = positions
            self._refresh(self._live)

    def set_joint_state(
        self, ctx: mujoco.MjData, robot_id: WorldRobotID, joint_state: JointState
    ) -> None:
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        entry = self._entry(robot_id)
        ctx.qpos[entry.qpos_adr] = np.asarray(joint_state.position, dtype=np.float64)
        self._refresh(ctx)

    def get_joint_state(self, ctx: mujoco.MjData, robot_id: WorldRobotID) -> JointState:
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        entry = self._entry(robot_id)
        positions = [float(v) for v in ctx.qpos[entry.qpos_adr]]
        return JointState(name=entry.config.joint_names, position=positions)

    def set_floating_base_pose(self, robot_id: WorldRobotID, pose: PoseStamped) -> None:
        """Write a floating-base robot's pelvis pose into the live context.

        Scratch contexts copy the live state, so every subsequent plan/IK
        starts from this pose — no model mutation needed."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        assert self._live is not None
        entry = self._entry(robot_id)
        if entry.root_free_qpos_adr is None:
            return  # welded base; nothing to update
        pos, quat = _pose_to_pos_quat(pose)
        adr = entry.root_free_qpos_adr
        with self._lock:
            self._live.qpos[adr : adr + 3] = pos
            self._live.qpos[adr + 3 : adr + 7] = quat
            self._refresh(self._live)

    def sync_entity_poses(self, poses: dict[str, PoseStamped]) -> None:
        """Write scene-entity poses into the live context (world frame).

        Fed from ``/entity_state_batch`` by the entity-state monitor; works
        identically whichever simulator (or perception) owns physics."""
        if not self._finalized:
            return
        assert self._live is not None
        with self._lock:
            for entity_id, pose in poses.items():
                adr = self._entity_qpos_adr.get(entity_id, -1)
                if adr is None:
                    if entity_id not in self._warned_static_entities:
                        self._warned_static_entities.add(entity_id)
                        logger.warning(
                            "entity %s is welded static; ignoring pose updates", entity_id
                        )
                    continue
                if adr < 0:
                    continue  # unknown entity — not in this scene package
                pos, quat = _pose_to_pos_quat(pose)
                self._live.qpos[adr : adr + 3] = pos
                self._live.qpos[adr + 3 : adr + 7] = quat
            self._refresh(self._live)

    # ------------------------------------------------------------------
    # Collision checking

    def _relevant_contacts(
        self, ctx: mujoco.MjData, entry: _RobotEntry
    ) -> tuple[NDArray[np.intp], NDArray[np.float64]]:
        """Indices and distances of contacts involving the robot's moving
        subtree, minus excluded pairs. Runs kinematics + collision."""
        assert self._model is not None
        mujoco.mj_kinematics(self._model, ctx)
        mujoco.mj_collision(self._model, ctx)
        if ctx.ncon == 0:
            return np.empty(0, dtype=np.intp), np.empty(0)
        geom = ctx.contact.geom[: ctx.ncon]
        dist = ctx.contact.dist[: ctx.ncon]
        mask = entry.check_geom_mask[geom[:, 0]] | entry.check_geom_mask[geom[:, 1]]
        if entry.excluded_pair_keys is not None and entry.excluded_pair_keys.size:
            lo = np.minimum(geom[:, 0], geom[:, 1]).astype(np.int64)
            hi = np.maximum(geom[:, 0], geom[:, 1]).astype(np.int64)
            keys = lo * self._model.ngeom + hi
            mask &= ~np.isin(keys, entry.excluded_pair_keys)
        idx = np.flatnonzero(mask)
        return idx, dist[idx]

    def is_collision_free(self, ctx: mujoco.MjData, robot_id: WorldRobotID) -> bool:
        """True iff no contact involving the robot's moving subtree penetrates."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        _, dist = self._relevant_contacts(ctx, self._entry(robot_id))
        return not bool(np.any(dist < 0.0))

    def get_min_distance(self, ctx: mujoco.MjData, robot_id: WorldRobotID) -> float:
        """Minimum distance over the robot's active contacts.

        MuJoCo only reports contacts within each pair's margin (default 0),
        so this is the min over touching/penetrating pairs — not a global
        signed-distance query like Drake's. +inf when contact-free."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        _, dist = self._relevant_contacts(ctx, self._entry(robot_id))
        return float(dist.min()) if dist.size else float("inf")

    def get_colliding_pairs(self, ctx: mujoco.MjData) -> list[tuple[str, str]]:
        """Body-name pairs currently in penetration (COLLISION_AT_START diagnostics)."""
        if not self._finalized:
            return []
        assert self._model is not None
        mujoco.mj_kinematics(self._model, ctx)
        mujoco.mj_collision(self._model, ctx)
        pairs: list[tuple[str, str]] = []
        for c in range(ctx.ncon):
            contact = ctx.contact[c]
            if contact.dist >= 0.0:
                continue
            names = []
            for gid in (int(contact.geom[0]), int(contact.geom[1])):
                bid = int(self._model.geom_bodyid[gid])
                names.append(
                    mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, bid) or str(bid)
                )
            pairs.append((names[0], names[1]))
        return pairs

    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        with self.scratch_context() as ctx:
            self.set_joint_state(ctx, robot_id, joint_state)
            return self.is_collision_free(ctx, robot_id)

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Linear interpolation at step_size; all configs checked in one scratch context."""
        q_start = np.asarray(start.position, dtype=np.float64)
        q_end = np.asarray(end.position, dtype=np.float64)

        dist = float(np.linalg.norm(q_end - q_start))
        if dist < 1e-8:
            return self.check_config_collision_free(robot_id, start)

        n_steps = max(2, int(np.ceil(dist / step_size)) + 1)
        entry = self._entry(robot_id)
        with self.scratch_context() as ctx:
            for i in range(n_steps):
                t = i / (n_steps - 1)
                ctx.qpos[entry.qpos_adr] = q_start + t * (q_end - q_start)
                _, dists = self._relevant_contacts(ctx, entry)
                if np.any(dists < 0.0):
                    return False
        return True

    # ------------------------------------------------------------------
    # Forward kinematics

    def get_ee_pose(self, ctx: mujoco.MjData, robot_id: WorldRobotID) -> PoseStamped:
        """Grasp-center pose (EE body pose shifted by grasp_offset_xyz)."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        entry = self._entry(robot_id)
        pos = ctx.xpos[entry.ee_body_id].copy()
        xmat = ctx.xmat[entry.ee_body_id].reshape(3, 3)
        if entry.grasp_offset is not None:
            pos = pos + xmat @ entry.grasp_offset
        quat = ctx.xquat[entry.ee_body_id]  # wxyz
        return PoseStamped(
            frame_id="world",
            position=[float(pos[0]), float(pos[1]), float(pos[2])],
            orientation=[float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])],
        )

    def get_link_pose(
        self, ctx: mujoco.MjData, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        assert self._model is not None
        entry = self._entry(robot_id)
        bid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, entry.prefix + link_name)
        if bid < 0:
            raise KeyError(f"Link '{link_name}' not found in robot '{robot_id}'")
        tf = np.eye(4)
        tf[:3, :3] = ctx.xmat[bid].reshape(3, 3)
        tf[:3, 3] = ctx.xpos[bid]
        return tf

    def get_jacobian(self, ctx: mujoco.MjData, robot_id: WorldRobotID) -> NDArray[np.float64]:
        """Geometric Jacobian (6 x n_joints), rows [vx, vy, vz, wx, wy, wz],
        taken at the grasp center (matches get_ee_pose)."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")
        assert self._model is not None
        entry = self._entry(robot_id)
        point = ctx.xpos[entry.ee_body_id].copy()
        if entry.grasp_offset is not None:
            point = point + ctx.xmat[entry.ee_body_id].reshape(3, 3) @ entry.grasp_offset
        jacp = np.zeros((3, self._model.nv))
        jacr = np.zeros((3, self._model.nv))
        mujoco.mj_jac(self._model, ctx, jacp, jacr, point, entry.ee_body_id)
        return np.vstack([jacp[:, entry.dof_adr], jacr[:, entry.dof_adr]])

    # ------------------------------------------------------------------
    # Obstacles

    def add_obstacle(self, obstacle: Obstacle) -> str:
        with self._lock:
            obstacle_id = obstacle.name
            existing = self._obstacles.get(obstacle_id)
            if existing is not None and not existing.removed:
                logger.debug(f"Obstacle '{obstacle_id}' already exists, skipping")
                return obstacle_id

            if not self._finalized:
                self._add_obstacle_to_spec(obstacle, obstacle_id)
            else:
                self._add_obstacle_to_slot(obstacle, obstacle_id)
            logger.debug(f"Added obstacle '{obstacle_id}': {obstacle.obstacle_type.value}")
            return obstacle_id

    def _obstacle_size_rbound(self, obstacle: Obstacle) -> tuple[list[float], float]:
        dims = obstacle.dimensions
        if obstacle.obstacle_type == ObstacleType.BOX:
            half = [dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0]
            return half, float(np.linalg.norm(half))
        if obstacle.obstacle_type == ObstacleType.SPHERE:
            return [dims[0], 0.0, 0.0], float(dims[0])
        if obstacle.obstacle_type == ObstacleType.CYLINDER:
            half_h = dims[1] / 2.0
            return [dims[0], half_h, 0.0], float(np.hypot(dims[0], half_h))
        raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")

    def _add_obstacle_to_spec(self, obstacle: Obstacle, obstacle_id: str) -> None:
        pos, quat = _pose_to_pos_quat(obstacle.pose)
        body = self._spec.worldbody.add_body(
            name=f"obstacle:{obstacle_id}", pos=pos.tolist(), quat=quat.tolist()
        )
        geom_kwargs: dict[str, Any] = {
            "name": f"obstacle:{obstacle_id}:geom",
            "rgba": list(obstacle.color),
        }
        if obstacle.obstacle_type == ObstacleType.MESH:
            if not obstacle.mesh_path:
                raise ValueError("MESH obstacle requires mesh_path")
            mesh_name = f"obstacle:{obstacle_id}:mesh"
            self._spec.add_mesh(name=mesh_name, file=str(obstacle.mesh_path))
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh_name, **geom_kwargs)
        else:
            size, _ = self._obstacle_size_rbound(obstacle)
            geom_type = {
                ObstacleType.BOX: mujoco.mjtGeom.mjGEOM_BOX,
                ObstacleType.SPHERE: mujoco.mjtGeom.mjGEOM_SPHERE,
                ObstacleType.CYLINDER: mujoco.mjtGeom.mjGEOM_CYLINDER,
            }[obstacle.obstacle_type]
            body.add_geom(type=geom_type, size=size, **geom_kwargs)
        self._obstacles[obstacle_id] = _ObstacleEntry(obstacle_id=obstacle_id, obstacle=obstacle)

    def _add_obstacle_to_slot(self, obstacle: Obstacle, obstacle_id: str) -> None:
        assert self._model is not None
        if obstacle.obstacle_type == ObstacleType.MESH:
            raise NotImplementedError(
                "MESH obstacles after finalize need an MjSpec recompile (see "
                "dimos/manipulation/design.md); add them before finalize, use a "
                "primitive approximation, or feed the object through the entity contract."
            )
        slots = self._free_slots.get(obstacle.obstacle_type)
        if not slots:
            raise RuntimeError(
                f"No free {obstacle.obstacle_type.value} obstacle slots "
                f"(world was created with obstacle_slots={self._n_slots})"
            )
        body_id, geom_id = slots.pop()
        size, rbound = self._obstacle_size_rbound(obstacle)
        pos, quat = _pose_to_pos_quat(obstacle.pose)
        self._model.body_pos[body_id] = pos
        self._model.body_quat[body_id] = quat
        self._model.geom_size[geom_id] = size
        # mj_collision prunes with cached per-geom and per-body bounds; all of
        # them must track the size mutation or contacts are silently dropped.
        self._model.geom_rbound[geom_id] = rbound
        self._model.geom_aabb[geom_id] = [0.0, 0.0, 0.0, *(max(s, 1e-3) for s in size)]
        self._model.geom_contype[geom_id] = 1
        self._model.geom_conaffinity[geom_id] = 1
        # body_contype/conaffinity are compile-time ORs of the body's geoms,
        # used for broadphase culling — stale zeros skip the body entirely.
        self._model.body_contype[body_id] = 1
        self._model.body_conaffinity[body_id] = 1
        self._obstacles[obstacle_id] = _ObstacleEntry(
            obstacle_id=obstacle_id,
            obstacle=obstacle,
            body_id=body_id,
            geom_id=geom_id,
            slot_type=obstacle.obstacle_type,
        )

    def _add_obstacle_slots(self) -> None:
        for obstacle_type, tag in _SLOT_TYPES.items():
            geom_type = {
                "box": mujoco.mjtGeom.mjGEOM_BOX,
                "sphere": mujoco.mjtGeom.mjGEOM_SPHERE,
                "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
            }[tag]
            for i in range(self._n_slots):
                body = self._spec.worldbody.add_body(
                    name=f"obstacle_slot:{tag}:{i}",
                    pos=[float(2 * i), 0.0, float(_SLOT_PARK_POS[2])],
                )
                body.add_geom(
                    name=f"obstacle_slot:{tag}:{i}:geom",
                    type=geom_type,
                    size=[0.01, 0.01, 0.01],
                    contype=0,
                    conaffinity=0,
                    rgba=[0.8, 0.2, 0.2, 0.8],
                )
            del obstacle_type

    def _resolve_slots(self) -> None:
        assert self._model is not None
        for obstacle_type, tag in _SLOT_TYPES.items():
            slots = []
            for i in range(self._n_slots):
                bid = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_BODY, f"obstacle_slot:{tag}:{i}"
                )
                gid = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_GEOM, f"obstacle_slot:{tag}:{i}:geom"
                )
                if bid >= 0 and gid >= 0:
                    slots.append((bid, gid))
            self._free_slots[obstacle_type] = slots

    def _resolve_prefinalize_obstacles(self) -> None:
        assert self._model is not None
        for entry in self._obstacles.values():
            if entry.body_id >= 0:
                continue
            bid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_BODY, f"obstacle:{entry.obstacle_id}"
            )
            gid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_GEOM, f"obstacle:{entry.obstacle_id}:geom"
            )
            entry.body_id, entry.geom_id = bid, gid

    def remove_obstacle(self, obstacle_id: str) -> bool:
        with self._lock:
            entry = self._obstacles.get(obstacle_id)
            if entry is None or entry.removed:
                return False
            if self._finalized and entry.body_id >= 0:
                assert self._model is not None
                self._model.body_pos[entry.body_id] = _SLOT_PARK_POS
                self._model.geom_contype[entry.geom_id] = 0
                self._model.geom_conaffinity[entry.geom_id] = 0
                self._model.body_contype[entry.body_id] = 0
                self._model.body_conaffinity[entry.body_id] = 0
                if entry.slot_type is not None:
                    self._free_slots[entry.slot_type].append((entry.body_id, entry.geom_id))
            entry.removed = True
            del self._obstacles[obstacle_id]
            return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        with self._lock:
            entry = self._obstacles.get(obstacle_id)
            if entry is None or entry.removed:
                return False
            if not self._finalized or entry.body_id < 0:
                entry.obstacle.pose = pose
                return True
            assert self._model is not None
            pos, quat = _pose_to_pos_quat(pose)
            self._model.body_pos[entry.body_id] = pos
            self._model.body_quat[entry.body_id] = quat
            entry.obstacle.pose = pose
            return True

    def clear_obstacles(self) -> None:
        for obstacle_id in list(self._obstacles):
            self.remove_obstacle(obstacle_id)

    def get_obstacles(self) -> list[Obstacle]:
        return [e.obstacle for e in self._obstacles.values() if not e.removed]

    # ------------------------------------------------------------------
    # Visualization (decoupled — see design.md; previews are published, not rendered)

    def get_visualization_url(self) -> str | None:
        return None

    def publish_visualization(self, ctx: mujoco.MjData | None = None) -> None:
        del ctx

    def animate_path(self, robot_id: WorldRobotID, path: JointPath, duration: float = 3.0) -> None:
        del robot_id, path, duration  # /planning/preview publishing lands with the viewer PR

    def close(self) -> None:
        with self._lock:
            self._scratch_pool.clear()


__all__ = ["MujocoWorld"]
