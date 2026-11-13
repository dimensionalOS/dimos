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

"""One URDF and the live joint state: every joint on tf, and the URDF and its files for renderers."""

from __future__ import annotations

import asyncio
from collections.abc import Mapping
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from threading import RLock
from typing import TYPE_CHECKING, Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from pydantic import Field, field_validator
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleBase, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import IO, In, Out
from dimos.msgs.files.File import File
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import JointDescription, RobotModel, UsdModel
from dimos.robot.assets.usd_utils.assets import (
    describe_unopenable_assets,
    is_published_asset,
    publish_usd_assets,
    referenced_asset_paths,
    require_asset_hashes,
    require_fetched_assets,
    resolve_usd_assets,
    unopenable_assets,
)
from dimos.robot.assets.usd_utils.frames import (
    prefix_usd_frames,
    rename_usd_frames,
    usd_frame_names,
)
from dimos.robot.assets.usd_utils.layers import graft_usd
from dimos.robot.assets.usd_utils.pxr import warm_usd
from dimos.robot.joint_tf import JointTf
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

logger = setup_logger()


SUBTREE_RPC = "get_usd"
SCENE_RPC = "get_scene"
ASSET_RPC = "get_asset"


WORLD_LINK = "world"


class CircularMountError(ValueError):
    pass


def _world_model() -> RobotModel:
    return RobotModel.from_root_link(WORLD_LINK, name=WORLD_LINK)


def _as_usd(model: RobotModel | UsdModel) -> UsdModel:
    return model if isinstance(model, UsdModel) else model.to_usd()


ASSET_ROOT, USD = 0, 1

USD_SUFFIXES = (".usd", ".usda")

_WAITING_LOG_EVERY = 20


@dataclass(frozen=True)
class Mount:
    module: type[ModuleBase] | None
    remote_name: str
    frame_prefix: str = ""
    subtree_rpc: str = SUBTREE_RPC

    @classmethod
    def of(cls, value: object) -> Mount:
        from dimos.core.coordination.blueprints import Blueprint

        if isinstance(value, Mount):
            return value
        if isinstance(value, Mapping) and "remote_name" in value:
            return cls(**value)
        if isinstance(value, Blueprint):
            return cls._from_blueprint(value)
        if isinstance(value, Mapping) and "blueprints" in value:
            return cls._from_flattened(value)
        if isinstance(value, str):
            return cls(None, value, value.rsplit("/", 1)[0] if "/" in value else "")
        if isinstance(value, type) and issubclass(value, Module):
            return cls(value, value.__name__, subtree_rpc=cls._tree_rpc(value))
        raise TypeError(
            f"mounts values are a Module subclass, a Blueprint or an instance name; got {value!r}"
        )

    @classmethod
    def _from_blueprint(cls, blueprint: Blueprint) -> Mount:
        atoms = [atom for atom in blueprint.blueprints if SUBTREE_RPC in atom.module.rpcs]
        if not atoms:
            raise ValueError(
                f"mounts was given a blueprint of "
                f"{sorted(atom.module.__name__ for atom in blueprint.blueprints)}, none of which "
                f"has an @rpc {SUBTREE_RPC}() to ask for its subtree"
            )
        if len(atoms) > 1:
            raise ValueError(
                f"mounts was given a blueprint with more than one module answering "
                f"{SUBTREE_RPC}() ({sorted(atom.name for atom in atoms)}); name the one that owns "
                f"the subtree instead of the whole blueprint"
            )
        return cls(
            atoms[0].module,
            atoms[0].name,
            atoms[0].kwargs.get("frame_id_prefix") or "",
            subtree_rpc=cls._tree_rpc(atoms[0].module),
        )

    @classmethod
    def _from_flattened(cls, blueprint: Mapping[str, Any]) -> Mount:
        atoms = [
            atom for atom in blueprint.get("blueprints") or () if SUBTREE_RPC in atom["module"].rpcs
        ]
        if len(atoms) != 1:
            named = sorted(atom["module"].__name__ for atom in blueprint.get("blueprints") or ())
            raise ValueError(
                f"mounts was given a blueprint of {named}, of which "
                f"{len(atoms)} answer an @rpc {SUBTREE_RPC}(); it has to be exactly one. Name "
                f"the module that owns the subtree instead of the whole blueprint."
            )
        return cls(
            atoms[0]["module"],
            atoms[0]["instance_name"],
            (atoms[0].get("kwargs") or {}).get("frame_id_prefix") or "",
            subtree_rpc=cls._tree_rpc(atoms[0]["module"]),
        )

    @staticmethod
    def _tree_rpc(module: type[ModuleBase]) -> str:
        return SCENE_RPC if SCENE_RPC in module.rpcs else SUBTREE_RPC


class UsdManagerConfig(ModuleConfig):
    model: RobotModel | UsdModel | None = None
    # Every single-DOF and fixed URDF joint as a parent -> child tf edge; off where another module owns those frames.
    publish_joint_tf: bool = True
    # An edge goes out when it moves, at most this often...
    joint_tf_hz: float = Field(default=30.0, gt=0.0)
    # ...and otherwise at this keepalive rate: a tf lookup needs a sample within its time tolerance, so keep
    # 1 / unchanged_tf_hz under twice the tightest tolerance of the tf readers (0.25 s in r1pro-nav: >= 2 Hz).
    unchanged_tf_hz: float = Field(default=4.0, gt=0.0)
    mounts: dict[str, Mount] = Field(default_factory=dict)
    tf_renames: dict[str, str] = Field(default_factory=dict)
    tf_renames_pre_namespace: dict[str, str] = Field(default_factory=dict)
    scene_retry_s: float = Field(default=0.5, gt=0.0)

    @field_validator("mounts", mode="before")
    @classmethod
    def _normalize_mounts(cls, value: object) -> object:
        if not isinstance(value, dict):
            return value
        return {frame: Mount.of(v) for frame, v in value.items()}


class UsdManager(Module):
    """Poses its description from the joint state onto tf, and serves the USD and the files it names
    to renderers."""

    config: UsdManagerConfig

    # IO: the joints it poses are published on it (and UsdManagerPointClearing reads a cloud's mount from it).
    tf: IO[TFMessage]
    coordinator_joint_state: In[JointState]
    observations: In[WorldState]
    # A request for a *.urdf gets the URDF; one for a file the URDF names (mesh, texture, splat, ...) by its full path
    # gets that file. Either goes out on file_info named as requested.
    render_file_request: In[String]
    file_info: Out[File]

    def __init__(self, **kwargs: object) -> None:
        self._ready = asyncio.Event()
        super().__init__(**kwargs)
        self._lock = RLock()
        self._description = _as_usd(self.config.model or _world_model())
        self._file_paths = self._description.referenced_files()
        self._frames = self._renamed_own_frames()
        # One description's joints on tf is its own job, not this module's; see dimos.robot.joint_tf.
        self._joint_tf = JointTf(
            self._description,
            frame=self.frame,
            joint_tf_hz=self.config.joint_tf_hz,
            unchanged_tf_hz=self.config.unchanged_tf_hz,
        )
        self._sim_clock: str | None = None
        self._sim_epoch = -1
        self._next_sim_tf_ns = 0
        self._usd: str | None = None
        self._usd_lock = RLock()
        self._scene_build = asyncio.Lock()
        self._scene: str | None = None
        self._scene_description: UsdModel | None = None
        self._subtree_clients: dict[str, RPCClient] = {}
        self._published_assets: dict[str, str] = {}
        self._asset_mounts: dict[str, Mount] = {}
        try:
            self._check_mounts()
            if not self.config.mounts:
                self._check_renames_matched(self._matched_own_renames())
        except Exception:
            self.dispose()
            raise
        self._ready.set()

    def _own_frame_names(self) -> set[str]:
        return self._description.frame_names()

    def _renamed_own_frames(self) -> dict[str, str]:
        """Each own frame that comes out of `frame()` under another name, by its name in the USD.

        The order is the one `_adopt` uses on a mounted subtree: the pre-namespace rename, then
        the namespace, then `tf_renames`. So `tf_renames` names a frame the way the rest of the
        system sees it -- `left/arm`, not `arm` -- for this description as much as for a mount.
        """
        pre = self.config.tf_renames_pre_namespace
        post = self.config.tf_renames
        renamed = {}
        for name in self._own_frame_names():
            namespaced = self._namespaced(pre.get(name, name))
            new = post.get(namespaced, namespaced)
            if new != name:
                renamed[name] = new
        return renamed

    def _namespaced(self, name: str) -> str:
        prefix = self.config.frame_id_prefix
        return f"{prefix}/{name}" if prefix else name

    def _matched_own_renames(self) -> set[str]:
        names = self._own_frame_names()
        pre = self.config.tf_renames_pre_namespace
        after_pre = {self._namespaced(pre.get(name, name)) for name in names}
        return (names & set(pre)) | (after_pre & set(self.config.tf_renames))

    def _check_mounts(self) -> None:
        for frame_id, mount in self.config.mounts.items():
            if mount.module is None:
                continue
            if not {SUBTREE_RPC, SCENE_RPC} & set(mount.module.rpcs):
                raise ValueError(
                    f"mounts attaches {mount.module.__name__} at {frame_id!r}, but it has "
                    f"neither an @rpc {SUBTREE_RPC}() nor an @rpc {SCENE_RPC}() to ask for its "
                    f"subtree"
                )

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()
        # A connected simulation supplies all edges at acquisition time.
        if self.config.publish_joint_tf and self.observations.transport is None:
            self._joint_tf.start(self.coordinator_joint_state, self.tfbuffer)

    @rpc
    def stop(self) -> None:
        self._joint_tf.stop()
        for client in self._subtree_clients.values():
            client.stop_rpc_client()
        self._subtree_clients.clear()
        super().stop()

    @rpc
    def get_usd(self) -> list[str]:
        with self._usd_lock:
            if self._usd is None:
                usd = self._description.usd
                self._usd = rename_usd_frames(usd, self._frames, self._asset_root() or None)
            return [self._asset_root(), self._usd]

    def _asset_root(self) -> str:
        source = self._description.source_path
        return str(source.parent) if source is not None else ""

    @rpc
    async def get_scene(self, visited: list[str] | None = None) -> list[str]:
        await self._ready.wait()
        chain = list(visited or [])
        self._refuse_a_cycle(chain)
        # Physics and every sensor request the same immutable startup scene.
        # Compose it once; simultaneous readers share the completed result.
        async with self._scene_build:
            if self._scene is None:
                root, scene = await self._compose(chain)
                where = root[ASSET_ROOT] or None
                await asyncio.to_thread(self._check_scene_assets, scene, where)
                self._scene_description = UsdModel.from_layer(
                    resolve_usd_assets(scene, root[ASSET_ROOT])
                )
                published, assets = await asyncio.to_thread(publish_usd_assets, scene, where)
                self._published_assets = assets
                self._scene = published
            return ["", self._scene]

    def _check_scene_assets(self, scene: str, root: str | None) -> None:
        unopenable = unopenable_assets(scene, root)
        if not unopenable:
            return
        what = f"the scene of {self._scene_identity}"
        require_fetched_assets(unopenable, root, what=what)
        logger.warning(describe_unopenable_assets(unopenable, root, what=what))

    @rpc
    async def get_asset(self, name: str) -> bytes:
        await self._ready.wait()
        real = self._published_assets.get(name, name if name in self._file_paths else None)
        if real is None:
            raise KeyError(
                f"{name!r} is neither a file this description names nor an asset the last scene "
                f"published. Ask for a scene before its assets: the names only exist once one "
                f"has been handed out."
            )
        mount = self._asset_mounts.get(real)
        if mount is not None:
            return await self._asset_from(mount, real)
        return await asyncio.to_thread(self._resolved(real).read_bytes)

    async def _asset_from(self, mount: Mount, name: str) -> bytes:
        """Ask the mount that published this name for the bytes; nothing here can read it."""
        client = self._subtree_client(mount)
        data = await asyncio.to_thread(partial(getattr(client, ASSET_RPC), name))
        if not isinstance(data, bytes):
            raise KeyError(
                f"{name!r} is an asset {mount.remote_name} published, but its "
                f"{ASSET_RPC}() answered {type(data).__name__}, not bytes"
            )
        return data

    def _refuse_a_cycle(self, chain: list[str]) -> None:
        if self._scene_identity in chain:
            raise CircularMountError(
                f"circular mount: {' -> '.join([*chain, self._scene_identity])}. One of these "
                f"modules is mounted on a module that is mounted on it, directly or through the "
                f"others, so neither scene can be built before the other. Break the cycle by "
                f"removing one of those mounts."
            )

    async def _compose(self, chain: list[str]) -> tuple[list[str], str]:
        subtrees = await self._subtrees([*chain, self._scene_identity])
        root = await asyncio.to_thread(self.get_usd)
        scene = (
            await asyncio.to_thread(
                self._graft_in_waves, root[USD], subtrees, root[ASSET_ROOT] or None
            )
            if subtrees
            else root[USD]
        )
        return root, scene

    @property
    def _scene_identity(self) -> str:
        return self.config.rpc_name or self.config.instance_name or type(self).__name__

    def _graft_in_waves(self, root_usd: str, subtrees: dict[str, str], root: str | None) -> str:
        scene = root_usd
        remaining = dict(subtrees)
        while remaining:
            present = usd_frame_names(scene, root)
            ready = {frame: usd for frame, usd in remaining.items() if frame in present}
            if not ready:
                raise ValueError(
                    f"cannot mount at {sorted(remaining)}: no link or joint by those names "
                    f"exists in this scene, and nothing left to graft would add one. Mount "
                    f"frames are the names as they end up in the scene, so a frame inside a "
                    f'mounted subtree carries that subtree\'s namespace prefix ("g1/head_mount", '
                    f'not "head_mount"). The scene has: {sorted(present)}'
                )
            scene = graft_usd(scene, ready, root)
            for frame in ready:
                del remaining[frame]
        return scene

    async def _subtrees(self, chain: list[str]) -> dict[str, str]:
        mounts = self.config.mounts
        if not mounts:
            self._check_renames_matched(self._matched_own_renames())
            return {}
        await asyncio.to_thread(warm_usd)
        results = await asyncio.gather(
            *(self._subtree(frame_id, mount, chain) for frame_id, mount in mounts.items())
        )
        matched = self._matched_own_renames()
        matched |= {name for _, subtree_matched in results for name in subtree_matched}
        self._check_renames_matched(matched)
        return {frame_id: usd for frame_id, (usd, _) in zip(mounts, results, strict=True)}

    def _check_renames_matched(self, matched: set[str]) -> None:
        wanted = set(self.config.tf_renames_pre_namespace) | set(self.config.tf_renames)
        missing = sorted(wanted - matched)
        if missing:
            raise ValueError(
                f"tf_renames named {missing}, which is neither a link or joint of this robot "
                f"nor of any mounted subtree (pre-namespace names are the subtree's own; "
                f"post-namespace names carry the namespace prefix)"
            )

    def _subtree_client(self, mount: Mount) -> RPCClient:
        client = self._subtree_clients.get(mount.remote_name)
        if client is None:
            if mount.module is None:
                client = RPCClient(None, None, mount.remote_name, {mount.subtree_rpc}, rpc=self.rpc)
            else:
                client = RPCClient.remote(mount.module, mount.remote_name, rpc=self.rpc)
            self._subtree_clients[mount.remote_name] = client
        return client

    async def _subtree(self, frame_id: str, mount: Mount, chain: list[str]) -> tuple[str, set[str]]:
        client = self._subtree_client(mount)
        name = mount.remote_name
        attempts = 0
        while True:
            attempts += 1
            answer = await asyncio.to_thread(self._ask_for_usd, client, frame_id, mount, chain)
            if answer is not None:
                logger.info(f"subtree for {frame_id} from {name} after {attempts} ask(s)")
                await asyncio.to_thread(
                    partial(
                        require_asset_hashes,
                        answer[USD],
                        what=f"the subtree {name} answered for frame {frame_id!r}",
                        root=answer[ASSET_ROOT] or None,
                    )
                )
                return await asyncio.to_thread(self._adopt, answer, mount)
            if attempts % _WAITING_LOG_EVERY == 0:
                logger.warning(
                    f"still waiting on {name} for {mount.subtree_rpc}() at frame {frame_id!r} "
                    f"after {attempts} asks"
                )
            await asyncio.sleep(self.config.scene_retry_s)

    def _adopt(self, answer: list[str], mount: Mount) -> tuple[str, set[str]]:
        pre = self.config.tf_renames_pre_namespace
        post = self.config.tf_renames
        where = answer[ASSET_ROOT] or None
        self._remember_published_assets(answer[USD], where, mount)
        usd = resolve_usd_assets(answer[USD], answer[ASSET_ROOT])
        matched = usd_frame_names(usd, where) & set(pre) if pre else set()
        usd = prefix_usd_frames(rename_usd_frames(usd, pre, where), mount.frame_prefix, where)
        if post:
            matched |= usd_frame_names(usd, where) & set(post)
        return rename_usd_frames(usd, post, where), matched

    def _remember_published_assets(self, usd: str, where: str | None, mount: Mount) -> None:
        """Note every asset in a subtree that is a name its owner published, not a file here.

        A mount that answers get_scene() hands back a scene whose assets are already published
        names -- there is no such file on this side of the RPC. Republishing them renames them
        again, so get_asset has to ask the mount that owns one instead of reading the path.
        """
        for asset in referenced_asset_paths(usd, where):
            if is_published_asset(asset):
                self._asset_mounts[asset] = mount

    def _ask_for_usd(
        self, client: RPCClient, frame_id: str, mount: Mount, chain: list[str]
    ) -> list[str] | None:
        try:
            if mount.subtree_rpc == SCENE_RPC:
                answer = getattr(client, SCENE_RPC)(chain)
            else:
                answer = getattr(client, SUBTREE_RPC)()
        except Exception as exc:
            if _is_circular_mount(exc):
                # Raise our own, not the RemoteError we caught: one hop further out that
                # error's remote_type is RemoteError, and the manager there would read a
                # cycle as a mount that has not answered yet and ask forever.
                raise CircularMountError(str(exc)) from exc
            logger.debug(f"{mount.subtree_rpc}() for {frame_id!r} did not answer yet: {exc}")
            return None
        if (
            not isinstance(answer, (list, tuple))
            or len(answer) != 2
            or not all(isinstance(part, str) for part in answer)
            or not answer[USD]
        ):
            logger.debug(f"{mount.subtree_rpc}() for {frame_id!r} answered {answer!r}, retrying")
            return None
        return list(answer)

    async def handle_render_file_request(self, request: String) -> None:
        name = request.data
        urdf = self._description.urdf()
        if (
            name.endswith(".urdf")
            and Path(name).stem == self.config.instance_name
            and urdf is not None
        ):
            self.file_info.publish(File(name, urdf.xml.encode()))
            return
        if name.endswith(USD_SUFFIXES):
            usd = self._scene
            if usd is None and self.config.mounts:
                usd = (await self.get_scene())[USD]
            if usd is None:
                usd = (await asyncio.to_thread(self.get_usd))[USD]
            self.file_info.publish(File(name, usd.encode()))
            return
        # Only files this description names or the last scene published: a request is never an
        # arbitrary path into the filesystem.
        real = self._published_assets.get(name, name if name in self._file_paths else None)
        if real is None:
            # another module's file (the sim's scene, ...): it answers, this stays silent
            logger.debug(
                f"render_file_request for {name!r}, which is neither a file this URDF names nor "
                f"an asset the last scene published"
            )
            return
        data = await asyncio.to_thread(self._resolved(real).read_bytes)
        self.file_info.publish(File(name, data))

    def _resolved(self, path: str) -> Path:
        found = Path(path)
        root = self._asset_root()
        return found if found.is_absolute() or not root else Path(root) / found

    @property
    def joints(self) -> dict[str, JointDescription]:
        """This description's joints by name, which is what it poses on tf."""
        return self._joint_tf.joints

    async def handle_observations(self, sample: WorldState) -> None:
        """Publish the same model tree from a complete physical pose sample."""
        if not self.config.publish_joint_tf:
            return
        h = sample.header
        if self._sim_clock is not None and self._sim_clock != h.clock_id:
            raise ValueError("robot model TF clock changed; restart its binding")
        self._sim_clock = h.clock_id
        if h.epoch < self._sim_epoch:
            return
        if h.epoch > self._sim_epoch:
            self._sim_epoch, self._next_sim_tf_ns = h.epoch, 0
        if h.time_ns < self._next_sim_tf_ns:
            return
        period = round(1e9 / self.config.joint_tf_hz)
        self._next_sim_tf_ns = (h.time_ns // period + 1) * period
        poses = {}
        for body in sample.bodies:
            p, q = body.pose.position, body.pose.orientation
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            matrix[:3, 3] = [p.x, p.y, p.z]
            poses[body.body_id] = matrix
        edges = []
        model = self._scene_description or self._description
        poses["world"] = np.eye(4)
        if model.root_link not in poses:
            raise ValueError(f"missing model root in body poses: {model.root_link}")
        if model.root_link != "world":
            edges.append(
                Transform.from_matrix(
                    poses[model.root_link],
                    frame_id="world",
                    child_frame_id=model.root_link,
                    ts=sample.header.time_ns / 1e9,
                )
            )
        for joint in model.joints:
            parent_id, child_id = joint.parent_link, joint.child_link
            parent, child = poses[parent_id], poses[child_id]
            relative = np.eye(4)
            relative[:3, :3] = parent[:3, :3].T @ child[:3, :3]
            relative[:3, 3] = parent[:3, :3].T @ (child[:3, 3] - parent[:3, 3])
            edges.append(
                Transform.from_matrix(
                    relative,
                    frame_id=parent_id,
                    child_frame_id=child_id,
                    ts=sample.header.time_ns / 1e9,
                )
            )
        self.tfbuffer.publish(*edges)

    def static_transforms(self) -> list[Transform]:
        """The edges that never move, so they need no joint state to be known."""
        return self._joint_tf.static_transforms()

    def joint_transforms(self, state: JointState) -> list[Transform]:
        """One parent -> child transform per joint, under this manager's frame names."""
        return self._joint_tf.joint_transforms(state)

    def add_joint_state(self, state: JointState) -> None:
        """Pose this description's joints from a state the caller already has."""
        if self.config.publish_joint_tf:
            self._joint_tf.add_joint_state(state, self.tfbuffer)

    def frame(self, link_or_joint: str) -> str:
        # _frames already holds the finished name, namespace included.
        known = self._frames.get(link_or_joint)
        return known if known is not None else self._namespaced(link_or_joint)


def _is_circular_mount(exc: BaseException) -> bool:
    if isinstance(exc, CircularMountError):
        return True
    wanted = f"{CircularMountError.__module__}.{CircularMountError.__name__}"
    return getattr(exc, "remote_type", None) == wanted
