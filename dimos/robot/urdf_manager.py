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
from dataclasses import dataclass
from pathlib import Path
from threading import RLock
import time
from typing import Any
import xml.etree.ElementTree as ET

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
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd import (
    graft_usd,
    prefix_usd_frames,
    rename_usd_frames,
    require_asset_hashes,
    resolve_usd_assets,
    urdf_to_usd,
    usd_frame_names,
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


def _world_model() -> RobotModel:
    """The stand-in description for a manager given no model: one link, called "world"."""
    return RobotModel.from_root_link(WORLD_LINK, name=WORLD_LINK)


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

# get_scene waits forever, so say so periodically rather than hang silently.
_WAITING_LOG_EVERY = 20


@dataclass(frozen=True)
class Mount:
    """A module owning a subtree: the name it answers RPC under, and the one its frames take.

    Both come apart from the class as soon as a robot has two of the same arm. The class is
    shared, so only the deployed instance name reaches the right one, and the description is
    shared, so only its namespace tells their frames apart. Build one with
    ``Mount.of(...)`` rather than by hand.
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
        if isinstance(value, Blueprint):
            return cls._from_blueprint(value)
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

    @staticmethod
    def _tree_rpc(module: type[Module]) -> str:
        """get_scene when the module has one, so its own mounts come along; else get_usd."""
        return SCENE_RPC if SCENE_RPC in module.rpcs else SUBTREE_RPC


class UrdfManagerConfig(ModuleConfig):
    # The robot this manager answers for. None gives a single link called "world" and nothing
    # else, which is a scene with no robot of its own -- useful when all this manager does is
    # hold mounts, so a tree of sensors and arms has somewhere to hang from.
    model: RobotModel | None = None
    # Every single-DOF and fixed URDF joint as a parent -> child tf edge; off where another module owns those frames.
    publish_joint_tf: bool = True
    # An edge goes out when it moves, at most this often...
    joint_tf_hz: float = Field(default=30.0, gt=0.0)
    # ...and otherwise at this keepalive rate: a tf lookup needs a sample within its time tolerance, so keep
    # 1 / unchanged_tf_hz under twice the tightest tolerance of the tf readers (0.25 s in r1pro-nav: >= 2 Hz).
    unchanged_tf_hz: float = Field(default=4.0, gt=0.0)
    # What is physically bolted to this robot: a frame of its URDF -> the module owning the subtree that hangs off
    # that frame. get_scene asks each of them for its own USD and grafts the result onto the root URDF, so every
    # module named here must answer the get_usd RPC. Nothing else reads this yet; the manager's tf and file serving
    # are unchanged by it, which is what keeps the empty default backwards compatible.
    # Keys are links of this robot's own URDF, unprefixed: the user writes them against the URDF
    # they passed in, and frame_id_prefix is applied on the way out to tf, not here.
    mounts: dict[str, Mount] = Field(default_factory=dict)
    # Frame renames for the subtrees get_scene collects, old name -> new name, reaching joints
    # as well as links (a joint name is what a JointState keys on). A vendor description names
    # its frames whatever it names them, which is rarely what the robot they are bolted to calls
    # them, and these are where that is said.
    #
    # ..._pre_namespace is in each subtree's own names, as its own description spells them, which
    # is what is in front of you when reading a vendor URDF. ..._post_namespace is in the names
    # after the namespace prefix has gone on, and is the only way to reach a name that does not
    # start with the namespace. Both apply to every subtree, so a key only has to match one of
    # them -- but a key matching none of them at all is an error, since a misspelt frame would
    # otherwise be a silent no-op.
    tf_renames_pre_namespace: dict[str, str] = Field(default_factory=dict)
    tf_renames: dict[str, str] = Field(default_factory=dict)
    # A module that is not up yet answers nothing, so retry it this often. There is no deadline:
    # get_scene has every subtree or it has nothing, and a scene missing an arm is not a scene.
    # A mount that never comes up hangs the caller, which is the honest failure -- the blueprint
    # named a module that has to be there.
    scene_retry_s: float = Field(default=0.5, gt=0.0)

    @field_validator("mounts", mode="before")
    @classmethod
    def _normalize_mounts(cls, value: object) -> object:
        """Accept a class, a (namespaced) Blueprint or an instance name for each value."""
        if not isinstance(value, dict):
            return value
        return {frame: Mount.of(v) for frame, v in value.items()}


class UrdfManager(Module):
    """Poses the URDF from the joint state onto tf, and serves the URDF and the files it names to renderers."""

    config: UrdfManagerConfig

    # IO: the joints it poses are published on it (and UrdfManagerPointClearing reads a cloud's mount from it).
    tf: IO[TFMessage]
    coordinator_joint_state: In[JointState]
    # A request for a *.urdf gets the URDF; one for a file the URDF names (mesh, texture, splat, ...) by its full path
    # gets that file. Either goes out on file_info named as requested.
    render_file_request: In[String]
    file_info: Out[File]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._description = (self.config.model or _world_model()).load()
        self._file_paths = _files_named_by(self._description.xml)
        self._joints = {joint.name: joint for joint in self._description.joints}
        self._next_joint_tf_t = 0.0
        # Per edge: the last transform sent (translation + rotation) and when.
        self._sent_edges: dict[tuple[str, str], tuple[tuple[float, ...], float]] = {}
        self._usd: str | None = None
        # Its own lock: baking meshes into a USD takes seconds, and self._lock is on the tf path.
        self._usd_lock = RLock()
        # Keyed by remote name: two instances of one class are two different modules to ask.
        self._subtree_clients: dict[str, RPCClient] = {}
        try:
            self._check_mounts()
        except Exception:
            # super().__init__ already took a loop thread and an RPC session; hand them back
            # rather than leave a half-built module behind for the caller to notice.
            self.dispose()
            raise

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

        Every module that can be mounted on another answers this, UrdfManager included, so one
        bolted to another robot's frame needs no special case.

        The root is the URDF's own directory. RobotModel.load() has already rewritten the
        description's relative asset paths to absolute ones, so in practice nothing in this
        layer needs it -- but a mounted module whose paths are relative does, and the asker
        cannot tell the two apart.
        """
        with self._usd_lock:
            if self._usd is None:
                self._usd = urdf_to_usd(self._description)
            return [self._asset_root(), self._usd]

    def _asset_root(self) -> str:
        """The directory this description's asset paths resolve against."""
        source = self._description.source_path
        return str(source.parent) if source is not None else ""

    @rpc
    async def get_scene(self, visited: list[str] | None = None) -> list[str]:
        """``[asset_root, usd]``: this URDF with every mounted subtree grafted on.

        Async and meant to be called before start(): it waits for every module in the map, with no
        deadline, because a scene missing an arm is not a scene. A mount that never answers hangs
        the caller rather than handing back half a robot.

        Each subtree is rebased onto the root its own module reported before it is grafted, so
        the one root this answers with covers the whole scene.

        A mount whose frame belongs to another subtree rather than to this URDF is grafted once
        that subtree is in place, so a lidar can hang off a g1 that is itself mounted here. See
        _graft_in_waves.

        ``visited`` is the chain of managers already assembling a scene above this one, which is
        how a mount cycle is caught instead of recursing until something falls over. Callers
        leave it alone; a manager asking one of its own mounts passes it on.
        """
        chain = list(visited or [])
        if self._scene_identity in chain:
            raise ValueError(
                f"circular mount: {' -> '.join([*chain, self._scene_identity])}. One of these "
                f"modules is mounted on a module that is mounted on it, directly or through the "
                f"others, so neither scene can be built before the other. Break the cycle by "
                f"removing one of those mounts."
            )
        subtrees = await self._subtrees([*chain, self._scene_identity])
        root = self.get_usd()
        if not subtrees:
            return root
        return [root[ASSET_ROOT], self._graft_in_waves(root[USD], subtrees)]

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
            self._check_renames_matched(set())
            return {}
        results = await asyncio.gather(
            *(self._subtree(frame_id, mount, chain) for frame_id, mount in mounts.items())
        )
        self._check_renames_matched({name for _, matched in results for name in matched})
        return {frame_id: usd for frame_id, (usd, _) in zip(mounts, results, strict=True)}

    def _check_renames_matched(self, matched: set[str]) -> None:
        """A rename key that matched no subtree at all is a typo, not a no-op."""
        wanted = set(self.config.tf_renames_pre_namespace) | set(self.config.tf_renames)
        missing = sorted(wanted - matched)
        if missing:
            raise ValueError(
                f"tf_renames named {missing}, which no mounted subtree has as a link or joint "
                f"(pre-namespace names are the subtree's own; post-namespace names carry the "
                f"namespace prefix)"
            )

    def _subtree_client(self, mount: Mount) -> RPCClient:
        """The client for one mount, keyed by remote name so two of one class stay apart.

        Hand-rolled rather than a declared module ref: a ref is one annotated attribute resolved
        by type, and this is a mapping whose keys are only known per robot.
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
        """The mounted module's USD, rebased and renamed, retried until it answers.

        No deadline: a mount is a module the blueprint says is physically there, so not
        answering is a thing to wait out rather than to give up on.
        """
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
            if "circular mount" in str(exc):
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
        if name.endswith(".urdf"):
            self.file_info.publish(File(name, self._description.xml.encode()))
            return
        # Only files the URDF names: a request is never an arbitrary path into the filesystem.
        if name not in self._file_paths:
            logger.warning(f"render_file_request for {name!r}, which is not a file the URDF names")
            return
        data = await asyncio.to_thread(Path(name).read_bytes)
        self.file_info.publish(File(name, data))

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
                frame_id=joint.parent_link,
                child_frame_id=joint.child_link,
                ts=state.ts,
            )
            for joint in self._joints.values()
            if joint.type not in MULTI_DOF_JOINT_TYPES
        ]


def _files_named_by(xml: str) -> set[str]:
    """The full path of every file the URDF names (mesh, texture, splat, ...): any filename attribute."""
    return {
        element.attrib["filename"]
        for element in ET.fromstring(xml).iter()
        if "filename" in element.attrib
    }
