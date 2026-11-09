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

"""One URDF and the live joint state: every joint on tf, and the robot as USD for renderers."""

from __future__ import annotations

import asyncio
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from threading import RLock
import time
from typing import Any

import numpy as np
from pydantic import Field, field_validator

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import IO, In, Out
from dimos.msgs.files.File import File
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.assets.usd import (
    graft_usd,
    prefix_usd_frames,
    publish_usd_assets,
    rename_usd_frames,
    require_asset_hashes,
    resolve_usd_assets,
    trivialize_usd,
    usd_frame_names,
    warm_usd,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Their pose comes from tf (odometry, a planar base), so they are read from tf, never published to it.
MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})

# The RPC every mounted module answers with its own subtree as a USD string.
SUBTREE_RPC = "get_usd"
# The RPC a mounted module answers with its *whole* tree, its own mounts included. A manager
# has one; a plain sensor does not.
SCENE_RPC = "get_scene"


# The root link a manager given no model stands on.
WORLD_LINK = "world"


class CircularMountError(ValueError):
    """Two managers each waiting for the other's scene. Retrying cannot fix it."""


def _world_model() -> RobotModel:
    """The stand-in description for a manager given no model: one link, called "world"."""
    return RobotModel.from_root_link(WORLD_LINK, name=WORLD_LINK)


def _as_usd(model: RobotModel | UsdModel) -> UsdModel:
    """Whatever was configured, as the one USD description the manager works from."""
    return model if isinstance(model, UsdModel) else model.to_usd()


# What get_usd and get_scene answer with: [asset_root, usd].
#
# A USD layer names its meshes relative to something, and only the module that answered for it
# knows what. Without the root travelling alongside, a subtree's mesh paths are unresolvable the
# moment they leave the process that produced them -- the answering module's cwd is not the
# asker's, and two subtrees need not share a root at all.
#
# asset_root is absolute, or relative to the answering process's cwd, or empty when the layer
# names no files.
ASSET_ROOT, USD = 0, 1

# What a renderer asks for to get the scene itself. The URDF is this module's input, not its
# output: what it serves is USD.
USD_SUFFIXES = (".usd", ".usda")

# get_scene waits forever, so say so periodically rather than hang silently.
_WAITING_LOG_EVERY = 20


@dataclass(frozen=True)
class Mount:
    """A module owning a subtree: the name it answers RPC under, and the one its frames take.

    Both come apart from the class as soon as a robot has two of the same arm. The class is
    shared, so only the deployed instance name reaches the right one, and the description is
    shared, so only its namespace tells their frames apart. The mounts config builds these;
    nothing constructs one by hand.
    """

    # None when only the instance name was given: the class is not importable here, so the
    # fail-fast get_usd check has to be skipped for it.
    module: type[Module] | None
    remote_name: str
    # The namespaced module's own frame_id_prefix, read straight off it rather than derived from
    # remote_name: namespace() sets the two together but keeps a frame_id_prefix the user set
    # by hand, and that override is the answer to "what are this module's frames called".
    frame_prefix: str = ""
    # The rpc that hands over this module's whole tree. A manager that has mounts of its own
    # answers get_scene, and asking it that rather than get_usd is what makes mounting
    # iterative: a lidar on a g1 arrives with the g1. Anything else only has get_usd.
    # A bare instance name has no class to ask, so it gets get_usd and says so.
    subtree_rpc: str = SUBTREE_RPC

    @classmethod
    def of(cls, value: object) -> Mount:
        """Normalize a mounts value.

        A module class names the only instance of itself. A ``Blueprint`` (the useful form:
        ``XArm.blueprint(...).namespace("left")``) names the exact instance it deploys and
        carries its frame_id_prefix, so two copies of one class stay both addressable and
        distinguishable. A plain string is the instance name of a module deployed elsewhere,
        where the class is not importable here.
        """
        # Imported here, as Module.blueprint does, to keep this module out of the import cycle.
        from dimos.core.coordination.blueprints import Blueprint

        if isinstance(value, Mount):
            return value
        if isinstance(value, Mapping) and "remote_name" in value:
            # A Mount that has already been normalized once. The config is validated again in
            # the worker the manager is deployed into, and what arrives there is the plain dict
            # this one was dumped to -- so without this branch every mount a blueprint actually
            # deploys is rejected, and mounts only ever work in-process.
            return cls(**value)
        if isinstance(value, Blueprint):
            return cls._from_blueprint(value)
        if isinstance(value, Mapping) and "blueprints" in value:
            # A Blueprint that has been through the config parser. It is the useful form to
            # write, and by the time the manager's worker validates its config again the
            # blueprint is a plain dict -- so without this branch the documented spelling is
            # rejected in the one place it matters ("mounts values are a Module subclass, a
            # Blueprint or an instance name; got {'blueprints': ...}") and only an instance
            # name written out by hand ever worked. The dict keeps what identifies the
            # instance: each atom's own `instance_name`, which is what namespace() set.
            return cls._from_flattened(value)
        if isinstance(value, str):
            # No object to read a frame_id_prefix off, so take the namespace out of the name:
            # everything before the last "/", the rule the coordinator's _atom_namespace uses.
            return cls(None, value, value.rsplit("/", 1)[0] if "/" in value else "")
        if isinstance(value, type) and issubclass(value, Module):
            # Matches RPCClient.remote's own default for a module with no instance_name.
            return cls(value, value.__name__, subtree_rpc=cls._tree_rpc(value))
        raise TypeError(
            f"mounts values are a Module subclass, a Blueprint or an instance name; got {value!r}"
        )

    @classmethod
    def _from_blueprint(cls, blueprint: Any) -> Mount:
        """The one atom of ``blueprint`` that answers get_usd, under its deployed name."""
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
        """The same rule as `_from_blueprint`, over the plain dict a parsed blueprint becomes.

        The atoms keep their `module` class and their `instance_name`, which is everything that
        identifies the instance; what they lose is the attribute access, so this reads the same
        three things out of subscripts.
        """
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
    def _tree_rpc(module: type[Module]) -> str:
        """get_scene when the module has one, so its own mounts come along; else get_usd."""
        return SCENE_RPC if SCENE_RPC in module.rpcs else SUBTREE_RPC


class UrdfManagerConfig(ModuleConfig):
    # The robot this manager answers for: a URDF to convert, or a USD layer to serve as it is
    # (a simulated world, say, which is already USD). None gives a single link called "world"
    # and nothing else, which is a scene with no robot of its own -- useful when all this
    # manager does is hold mounts, so a tree of sensors and arms has somewhere to hang from.
    # A UsdModel has no joints, so nothing is posed on tf from one.
    model: RobotModel | UsdModel | None = None
    # Keep the frames, drop the shapes: the links, joints and poses stay, the visuals and
    # collision meshes go. The same description then serves a real robot and a simulated one,
    # since only the sim needs the geometry, and the mount points stay identical either way.
    # This robot's own layer only -- what a mounted module answers with is its own say.
    trivialize: bool = False
    # Every single-DOF and fixed URDF joint as a parent -> child tf edge; off where another module owns those frames.
    publish_joint_tf: bool = True
    # An edge goes out when it moves, at most this often...
    joint_tf_hz: float = Field(default=30.0, gt=0.0)
    # ...and otherwise at this keepalive rate: a tf lookup needs a sample within its time tolerance, so keep
    # 1 / unchanged_tf_hz under twice the tightest tolerance of the tf readers (0.25 s in r1pro-nav: >= 2 Hz).
    unchanged_tf_hz: float = Field(default=4.0, gt=0.0)
    # What is bolted to this robot: a frame of its URDF -> the module owning the subtree there.
    # Keys are the names as they end up in the scene, so a frame inside a mounted subtree
    # carries that subtree's prefix.
    mounts: dict[str, Mount] = Field(default_factory=dict)
    # Frame renames, old -> new, reaching joints as well as links. They apply to this robot's
    # own frames -- on tf and in its USD -- and to every subtree get_scene collects.
    # tf_renames is in the names after the namespace prefix has gone on, which is the usual
    # place to work; tf_renames_pre_namespace runs first, in each subtree's own names. This
    # robot takes no prefix, so for its frames the two differ only in order. A key matching
    # nothing at all is an error.
    tf_renames: dict[str, str] = Field(default_factory=dict)
    tf_renames_pre_namespace: dict[str, str] = Field(default_factory=dict)
    # How often to re-ask a module that is not up yet. There is no deadline: a scene missing an
    # arm is not a scene, so a mount that never comes up hangs the caller.
    scene_retry_s: float = Field(default=0.5, gt=0.0)

    @field_validator("mounts", mode="before")
    @classmethod
    def _normalize_mounts(cls, value: object) -> object:
        """Accept a class, a (namespaced) Blueprint or an instance name for each value."""
        if not isinstance(value, dict):
            return value
        return {frame: Mount.of(v) for frame, v in value.items()}


class UrdfManager(Module):
    """Poses the URDF from the joint state onto tf, and serves the robot as USD plus the files it names."""

    config: UrdfManagerConfig

    # IO: the joints it poses are published on it (and UrdfManagerPointClearing reads a cloud's mount from it).
    tf: IO[TFMessage]
    coordinator_joint_state: In[JointState]
    # A request for a *.usd or *.usda gets the scene as a USD layer; one for a file that layer
    # names (mesh, texture, splat, ...) gets that file. Either goes out on file_info as requested.
    render_file_request: In[String]
    file_info: Out[File]

    def __init__(self, **kwargs: object) -> None:
        # Set once this manager is wholly built. super().__init__ takes an RPC session, so from
        # that line on a caller can ask for the scene -- and one does, the moment a simulation
        # goes looking for the world it is about to run. That reached a half-built object:
        # `AttributeError: 'UrdfManager' object has no attribute '_subtree_clients'`. The rpcs
        # that need this module assembled wait on it instead, which they can because they are
        # async: the handler is entered, yields, and resumes once __init__ has returned.
        self._ready = asyncio.Event()
        super().__init__(**kwargs)
        self._lock = RLock()
        # Converted here and once: everything below works on the USD, whatever it came from.
        self._description = _as_usd(self.config.model or _world_model())
        self._file_paths = self._description.referenced_files()
        self._joints = {joint.name: joint for joint in self._description.joints}
        # tf_renames reach this robot's own frames too, so every name it emits goes through here.
        self._frames = self._renamed_own_frames()
        self._next_joint_tf_t = 0.0
        # Per edge: the last transform sent (translation + rotation) and when.
        self._sent_edges: dict[tuple[str, str], tuple[tuple[float, ...], float]] = {}
        self._usd: str | None = None
        # Its own lock: baking meshes into a USD takes seconds, and self._lock is on the tf path.
        self._usd_lock = RLock()
        # Serializes scene assembly across the file requests that arrive while it runs.
        self._scene_build = asyncio.Lock()
        # The last scene get_scene assembled. A renderer wants the whole tree, not just this
        # robot, and only get_scene knows what is mounted.
        self._scene: str | None = None
        # Keyed by remote name: two instances of one class are two different modules to ask.
        self._subtree_clients: dict[str, RPCClient] = {}
        # What the last scene promised to answer for: public asset path -> the real file.
        # render_file_request serves these as well as the URDF's own names, which is the whole
        # point of handing out paths that are not paths on anyone's disk.
        self._published_assets: dict[str, str] = {}
        try:
            self._check_mounts()
            if not self.config.mounts:
                # With no subtrees coming, this robot's frames are every name there will be, so
                # a rename that matches none of them is a typo and can be caught here. With
                # mounts it has to wait for get_scene, which is where the other names arrive.
                self._check_renames_matched(self._matched_own_renames())
        except Exception:
            # super().__init__ already took a loop thread and an RPC session; hand them back
            # rather than leave a half-built module behind for the caller to notice. _ready
            # stays clear: there is nothing to be ready, and a caller waiting on a manager that
            # failed to build is waiting on a deployment that is going down anyway.
            self.dispose()
            raise
        self._ready.set()

    def _own_frame_names(self) -> set[str]:
        """Every link and joint name this robot's own layer carries, before any rename."""
        return self._description.frame_names()

    def _renamed_own_frames(self) -> dict[str, str]:
        """This robot's renamed frames, old -> new; only the ones that actually change.

        Pre-namespace then post: this robot takes no namespace prefix of its own, so the two
        dicts are just applied in order, and one key can be written in either.
        """
        pre = self.config.tf_renames_pre_namespace
        post = self.config.tf_renames
        renamed = {}
        for name in self._own_frame_names():
            once = pre.get(name, name)
            new = post.get(once, once)
            if new != name:
                renamed[name] = new
        return renamed

    def _matched_own_renames(self) -> set[str]:
        """The rename keys this robot's own frames account for."""
        names = self._own_frame_names()
        pre = self.config.tf_renames_pre_namespace
        after_pre = {pre.get(name, name) for name in names}
        return (names & set(pre)) | (after_pre & set(self.config.tf_renames))

    def _check_mounts(self) -> None:
        """Fail here rather than at get_scene time, which is a blueprint's last step before start.

        Only the module is checkable this early. The frame is not: a mount may legitimately name
        a frame that belongs to another mounted subtree and so does not exist until the scene is
        assembled ("g1/head_mount"). That one is caught by _graft_in_waves, which by then can
        list every frame there actually is.
        """
        for frame_id, mount in self.config.mounts.items():
            # An instance name on its own carries no class, so there is nothing to check; it fails
            # at get_scene time instead, which is why a class or a blueprint is the better value.
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

    @rpc
    def stop(self) -> None:
        for client in self._subtree_clients.values():
            client.stop_rpc_client()
        self._subtree_clients.clear()
        super().stop()

    @rpc
    def get_usd(self) -> list[str]:
        """``[asset_root, usd]``: this robot's own URDF as a USD layer, nothing attached.

        Anything mountable answers this, UrdfManager included, so a robot on a robot is no
        special case. The root is the URDF's own directory: a mounted module whose asset paths
        are relative needs one, and the asker cannot tell which kind it got.
        """
        with self._usd_lock:
            if self._usd is None:
                usd = self._description.usd
                if self.config.trivialize:
                    usd = trivialize_usd(usd)
                # Renamed here as well as on tf, or the layer would name frames that no
                # transform ever arrives for.
                self._usd = rename_usd_frames(usd, self._frames)
            return [self._asset_root(), self._usd]

    def _asset_root(self) -> str:
        """The directory this description's asset paths resolve against."""
        source = self._description.source_path
        return str(source.parent) if source is not None else ""

    @rpc
    async def get_scene(self, visited: list[str] | None = None) -> list[str]:
        """``[asset_root, usd]``: this URDF with every mounted subtree grafted on.

        Async and meant to be called before start(). It waits with no deadline: half a scene is
        a robot with no arm, so a mount that never answers hangs the caller instead.

        Each subtree is rebased onto the root its own module reported before it is grafted, so
        the one root this answers with covers the whole scene.

        A mount whose frame belongs to another subtree rather than to this URDF is grafted once
        that subtree is in place, so a lidar can hang off a g1 that is itself mounted here. See
        _graft_in_waves.

        ``visited`` is the chain of managers already assembling a scene above this one, which is
        how a mount cycle is caught instead of recursing until something falls over. Callers
        leave it alone; a manager asking one of its own mounts passes it on.
        """
        await self._ready.wait()
        chain = list(visited or [])
        self._refuse_a_cycle(chain)
        root, scene = await self._compose(chain)
        # Grafting works in absolute paths, which say nothing to whoever ends up asking for the
        # file. So the scene leaves under names this manager answers for instead.
        published, assets = await asyncio.to_thread(publish_usd_assets, scene)
        self._published_assets = assets
        self._scene = published
        # No asset root: every path in the published scene is a name this manager answers for
        # over render_file_request, not a path that resolves against a directory on any disk.
        return ["", published]

    @rpc
    async def get_scene_on_disk(self, visited: list[str] | None = None) -> list[str]:
        """``[asset_root, usd]``: the same scene, still naming the files it was composed from.

        For a consumer that opens files rather than asking for them. A physics engine reads a
        layer off the disk; `get_scene` hands out content names it serves over
        `render_file_request` instead, which is right for a viewer across a bus and unusable to
        MuJoCo -- "at least 4 vertices required" on the first piece of furniture is what asking
        the wrong one of these looks like.

        Resolve it against the root this answers with before reading it: the mounted subtrees
        already carry absolute paths, having each been rebased as it arrived, and what is left
        relative is this manager's own layer -- relative to its own directory, which is the root.

        Same machine only, and deliberately so. A backend somewhere else has to be handed the
        bytes, not a path, and that is `get_scene` plus `render_file_request`.
        """
        await self._ready.wait()
        chain = list(visited or [])
        self._refuse_a_cycle(chain)
        root, scene = await self._compose(chain)
        return [root[ASSET_ROOT], scene]

    def _refuse_a_cycle(self, chain: list[str]) -> None:
        if self._scene_identity in chain:
            raise CircularMountError(
                f"circular mount: {' -> '.join([*chain, self._scene_identity])}. One of these "
                f"modules is mounted on a module that is mounted on it, directly or through the "
                f"others, so neither scene can be built before the other. Break the cycle by "
                f"removing one of those mounts."
            )

    async def _compose(self, chain: list[str]) -> tuple[list[str], str]:
        """This manager's own layer, and it with every mounted subtree grafted on."""
        subtrees = await self._subtrees([*chain, self._scene_identity])
        # Off the loop thread: building the layer imports usd-core and converts the whole
        # URDF, which is a tenth of a second on a real robot and would stall tf meanwhile.
        root = await asyncio.to_thread(self.get_usd)
        scene = (
            await asyncio.to_thread(self._graft_in_waves, root[USD], subtrees)
            if subtrees
            else root[USD]
        )
        return root, scene

    @property
    def _scene_identity(self) -> str:
        """What this manager is called in a mount chain: the name it answers RPC under."""
        return self.config.rpc_name or self.config.instance_name or type(self).__name__

    def _graft_in_waves(self, root_usd: str, subtrees: dict[str, str]) -> str:
        """Graft each subtree once the frame it names exists, which may be another subtree's.

        Mounts are not independent: "g1/head_mount" is a frame that only exists once the g1 is
        in place. So this grafts whatever can be grafted, then looks again, until either
        everything is in or nothing moved -- and nothing moving means the frames that are left
        are never going to appear.
        """
        scene = root_usd
        remaining = dict(subtrees)
        while remaining:
            present = usd_frame_names(scene)
            ready = {frame: usd for frame, usd in remaining.items() if frame in present}
            if not ready:
                raise ValueError(
                    f"cannot mount at {sorted(remaining)}: no link or joint by those names "
                    f"exists in this scene, and nothing left to graft would add one. Mount "
                    f"frames are the names as they end up in the scene, so a frame inside a "
                    f'mounted subtree carries that subtree\'s namespace prefix ("g1/head_mount", '
                    f'not "head_mount"). The scene has: {sorted(present)}'
                )
            scene = graft_usd(scene, ready)
            for frame in ready:
                del remaining[frame]
        return scene

    async def _subtrees(self, chain: list[str]) -> dict[str, str]:
        mounts = self.config.mounts
        if not mounts:
            self._check_renames_matched(self._matched_own_renames())
            return {}
        # Before the gather, not inside it: each subtree is read and hashed on its own thread,
        # and the USD bindings finish loading themselves the first time a layer is read. Several
        # threads reaching that together deadlock in the import machinery, and this manager then
        # answers nothing, for ever, without raising.
        await asyncio.to_thread(warm_usd)
        results = await asyncio.gather(
            *(self._subtree(frame_id, mount, chain) for frame_id, mount in mounts.items())
        )
        matched = self._matched_own_renames()
        matched |= {name for _, subtree_matched in results for name in subtree_matched}
        self._check_renames_matched(matched)
        return {frame_id: usd for frame_id, (usd, _) in zip(mounts, results, strict=True)}

    def _check_renames_matched(self, matched: set[str]) -> None:
        """A rename key that matched nothing in the scene is a typo, not a no-op."""
        wanted = set(self.config.tf_renames_pre_namespace) | set(self.config.tf_renames)
        missing = sorted(wanted - matched)
        if missing:
            raise ValueError(
                f"tf_renames named {missing}, which is neither a link or joint of this robot "
                f"nor of any mounted subtree (pre-namespace names are the subtree's own; "
                f"post-namespace names carry the namespace prefix)"
            )

    def _subtree_client(self, mount: Mount) -> RPCClient:
        """The client for one mount, keyed by remote name so two of one class stay apart.

        Hand-rolled rather than a module ref: a ref is one annotated attribute resolved by type,
        and this is a mapping.
        """
        client = self._subtree_clients.get(mount.remote_name)
        if client is None:
            if mount.module is None:
                # No class to take a signature or an rpc list from, so declare the one rpc we call.
                client = RPCClient(None, None, mount.remote_name, {mount.subtree_rpc}, rpc=self.rpc)
            else:
                client = RPCClient.remote(mount.module, mount.remote_name, rpc=self.rpc)
            self._subtree_clients[mount.remote_name] = client
        return client

    async def _subtree(self, frame_id: str, mount: Mount, chain: list[str]) -> tuple[str, set[str]]:
        """The mounted module's USD, rebased and renamed, retried; no deadline (see get_scene)."""
        client = self._subtree_client(mount)
        name = mount.remote_name
        attempts = 0
        while True:
            attempts += 1
            answer = await asyncio.to_thread(self._ask_for_usd, client, frame_id, mount, chain)
            if answer is not None:
                logger.info(f"subtree for {frame_id} from {name} after {attempts} ask(s)")
                # Refused rather than retried: a subtree whose assets are not identified by
                # content will not become identified by asking again, and the error carries
                # the hashes it should have had.
                await asyncio.to_thread(
                    require_asset_hashes,
                    answer[USD],
                    what=f"the subtree {name} answered for frame {frame_id!r}",
                )
                return await asyncio.to_thread(self._adopt, answer, mount)
            if attempts % _WAITING_LOG_EVERY == 0:
                logger.warning(
                    f"still waiting on {name} for {mount.subtree_rpc}() at frame {frame_id!r} "
                    f"after {attempts} asks"
                )
            await asyncio.sleep(self.config.scene_retry_s)

    def _adopt(self, answer: list[str], mount: Mount) -> tuple[str, set[str]]:
        """One subtree, made fit to graft onto this robot.

        Returns it with the rename keys it matched, so the caller can tell a key that was meant
        for another subtree from one that matches nothing at all.

        Its asset paths are rebased onto the root it reported, because it is leaving the only
        process that knew that root. Its frames are renamed here rather than at its end: a
        module holds one description and does not know which copy of itself it is, while the
        name it was deployed under says exactly that.

        The order is rename, prefix, rename again, so a rename can be written in the subtree's
        own names or in the prefixed ones, whichever is easier to express it in.
        """
        pre = self.config.tf_renames_pre_namespace
        post = self.config.tf_renames
        usd = resolve_usd_assets(answer[USD], answer[ASSET_ROOT])
        matched = usd_frame_names(usd) & set(pre) if pre else set()
        usd = prefix_usd_frames(rename_usd_frames(usd, pre), mount.frame_prefix)
        if post:
            matched |= usd_frame_names(usd) & set(post)
        return rename_usd_frames(usd, post), matched

    def _ask_for_usd(
        self, client: Any, frame_id: str, mount: Mount, chain: list[str]
    ) -> list[str] | None:
        """One RPC attempt; a module that is not serving yet is a retry, not a failure.

        A manager is asked for its whole scene and handed the chain, so it can refuse a cycle
        rather than ask back round. Anything else is asked for its own layer.
        """
        try:
            if mount.subtree_rpc == SCENE_RPC:
                answer = getattr(client, SCENE_RPC)(chain)
            else:
                answer = getattr(client, SUBTREE_RPC)()
        except Exception as exc:
            # A circular mount is the one failure worth stopping for: retrying cannot fix it,
            # and the manager below has already named the whole chain.
            if _is_circular_mount(exc):
                raise
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
        if name.endswith(USD_SUFFIXES):
            async with self._scene_build:
                usd = self._scene
                if usd is None and self.config.mounts:
                    # A manager with mounts answers for the whole tree, and nothing else
                    # assembles it: get_scene is an rpc for a manager above this one to call,
                    # and a top-level manager has nobody above it. Falling through to get_usd
                    # here answered with this robot's own layer alone -- for a manager that is
                    # only a set of mount points, a stage of empty frames with no geometry in
                    # it at all, and not an error anywhere to say so.
                    usd = (await self.get_scene())[USD]
                if usd is None:
                    # Off the loop thread: the first request is the one that pays for the import
                    # and the conversion, and it arrives while tf is flowing.
                    usd = (await asyncio.to_thread(self.get_usd))[USD]
            self.file_info.publish(File(name, usd.encode()))
            return
        # Only files this manager has published: a request is never an arbitrary path into the
        # filesystem. That is the URDF's own names, plus the public names the last scene handed
        # out, which is how a mounted arm's mesh gets served at all.
        real = self._published_assets.get(name, name if name in self._file_paths else None)
        if real is None:
            # debug, not warning: another module on this topic (the sim's world) answers for
            # its own files, and every one of those would otherwise be logged here as a miss.
            logger.debug(
                f"render_file_request for {name!r}, which is neither a file this URDF names nor "
                f"an asset the last scene published"
            )
            return
        data = await asyncio.to_thread(self._resolved(real).read_bytes)
        self.file_info.publish(File(name, data))

    def _resolved(self, path: str) -> Path:
        """An asset path as a file to open: a relative one is relative to the description."""
        found = Path(path)
        root = self._asset_root()
        return found if found.is_absolute() or not root else Path(root) / found

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        await asyncio.to_thread(self.add_joint_state, state)

    def add_joint_state(self, state: JointState) -> None:
        """Publish the state's joints on tf."""
        if not self.config.publish_joint_tf or time.monotonic() < self._next_joint_tf_t:
            return
        self._next_joint_tf_t = time.monotonic() + 1.0 / self.config.joint_tf_hz
        with self._lock:
            transforms = self._edges_to_send(self.joint_transforms(state))
        if transforms:
            self.tfbuffer.publish(*transforms)

    def _edges_to_send(self, transforms: list[Transform]) -> list[Transform]:
        """The edges that moved since they were last sent, and those not sent for one keepalive period."""
        now = time.monotonic()
        keepalive_s = 1.0 / self.config.unchanged_tf_hz
        due = []
        for transform in transforms:
            key = (transform.frame_id, transform.child_frame_id)
            value = (*transform.translation.to_tuple(), *transform.rotation.to_tuple())
            sent = self._sent_edges.get(key)
            if (
                sent is None
                or now - sent[1] >= keepalive_s
                or not np.allclose(value, sent[0], atol=1e-6)
            ):
                due.append(transform)
                self._sent_edges[key] = (value, now)
        return due

    def joint_transforms(self, state: JointState) -> list[Transform]:
        """One parent -> child transform per URDF joint; joints the state leaves out stay at zero."""
        positions = dict(zip(state.name, state.position, strict=False))
        for joint in self._joints.values():
            if joint.mimic is not None and joint.mimic[0] in positions:
                leader, multiplier, offset = joint.mimic
                positions[joint.name] = multiplier * positions[leader] + offset
        # Fixed joints are republished with the moving ones: a chained lookup is only as fresh as its stalest edge.
        return [
            Transform.from_matrix(
                joint.parent_from_child(positions.get(joint.name, 0.0)),
                frame_id=self.frame(joint.parent_link),
                child_frame_id=self.frame(joint.child_link),
                ts=state.ts,
            )
            for joint in self._joints.values()
            if joint.type not in MULTI_DOF_JOINT_TYPES
        ]

    def frame(self, link_or_joint: str) -> str:
        """The name this manager publishes one of its own frames under, after tf_renames.

        The description keeps its own names -- they are what the hardware speaks, and what a
        JointState arrives keyed by. Only what goes out is renamed, so anything reading tf for
        a frame of this robot has to ask through here.

        A namespaced manager puts its namespace in front, the same prefix every other module
        puts on its frames. Without it two robots in one world both publish "pelvis", and the
        links hanging off them land in a tree of their own with no route to the world: tf then
        has two roots and can draw neither robot where it stands.
        """
        renamed = self._frames.get(link_or_joint, link_or_joint)
        prefix = self.config.frame_id_prefix
        return f"{prefix}/{renamed}" if prefix else renamed


def _is_circular_mount(exc: BaseException) -> bool:
    """True for a CircularMountError raised here or by a manager in another process.

    The RPC transport only rebuilds builtin exception types; anything else arrives as a
    RemoteError that names the type it came from, so that is what identifies it across a
    process boundary.
    """
    if isinstance(exc, CircularMountError):
        return True
    wanted = f"{CircularMountError.__module__}.{CircularMountError.__name__}"
    return getattr(exc, "remote_type", None) == wanted
