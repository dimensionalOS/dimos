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
from dimos.robot.assets.usd import MeshMode, graft_usd, prefix_usd_frames, urdf_to_usd
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Their pose comes from tf (odometry, a planar base), so they are read from tf, never published to it.
MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})

# The RPC every module in physical_connections answers with its own subtree as a USD string.
SUBTREE_RPC = "get_usd"


@dataclass(frozen=True)
class PhysicalConnection:
    """A module owning a subtree: the name it answers RPC under, and the one its frames take.

    Both come apart from the class as soon as a robot has two of the same arm. The class is
    shared, so only the deployed instance name reaches the right one, and the description is
    shared, so only its namespace tells their frames apart. Build one with
    ``PhysicalConnection.of(...)`` rather than by hand.
    """

    # None when only the instance name was given: the class is not importable here, so the
    # fail-fast get_usd check has to be skipped for it.
    module: type[Module] | None
    remote_name: str
    # The namespaced module's own frame_id_prefix, read straight off it rather than derived from
    # remote_name: namespace() sets the two together but keeps a frame_id_prefix the user set
    # by hand, and that override is the answer to "what are this module's frames called".
    frame_prefix: str = ""

    @classmethod
    def of(cls, value: object) -> PhysicalConnection:
        """Normalize a physical_connections value.

        A module class names the only instance of itself. A ``Blueprint`` (the useful form:
        ``XArm.blueprint(...).namespace("left")``) names the exact instance it deploys and
        carries its frame_id_prefix, so two copies of one class stay both addressable and
        distinguishable. A plain string is the instance name of a module deployed elsewhere,
        where the class is not importable here.
        """
        # Imported here, as Module.blueprint does, to keep this module out of the import cycle.
        from dimos.core.coordination.blueprints import Blueprint

        if isinstance(value, PhysicalConnection):
            return value
        if isinstance(value, Blueprint):
            return cls._from_blueprint(value)
        if isinstance(value, str):
            # No object to read a frame_id_prefix off, so take the namespace out of the name:
            # everything before the last "/", the rule the coordinator's _atom_namespace uses.
            return cls(None, value, value.rsplit("/", 1)[0] if "/" in value else "")
        if isinstance(value, type) and issubclass(value, Module):
            # Matches RPCClient.remote's own default for a module with no instance_name.
            return cls(value, value.__name__)
        raise TypeError(
            f"physical_connections values are a Module subclass, a Blueprint or an instance "
            f"name; got {value!r}"
        )

    @classmethod
    def _from_blueprint(cls, blueprint: Any) -> PhysicalConnection:
        """The one atom of ``blueprint`` that answers get_usd, under its deployed name."""
        atoms = [atom for atom in blueprint.blueprints if SUBTREE_RPC in atom.module.rpcs]
        if not atoms:
            raise ValueError(
                f"physical_connections was given a blueprint of "
                f"{sorted(atom.module.__name__ for atom in blueprint.blueprints)}, none of which "
                f"has an @rpc {SUBTREE_RPC}() to ask for its subtree"
            )
        if len(atoms) > 1:
            raise ValueError(
                f"physical_connections was given a blueprint with more than one module answering "
                f"{SUBTREE_RPC}() ({sorted(atom.name for atom in atoms)}); name the one that owns "
                f"the subtree instead of the whole blueprint"
            )
        return cls(atoms[0].module, atoms[0].name, atoms[0].kwargs.get("frame_id_prefix") or "")


class UrdfManagerConfig(ModuleConfig):
    model: RobotModel
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
    physical_connections: dict[str, PhysicalConnection] = Field(default_factory=dict)
    # get_scene has every subtree or it has nothing, so it waits this long for the slowest one.
    scene_timeout_s: float = Field(default=60.0, gt=0.0)
    # A module that is not up yet answers nothing, so retry it this often inside that budget.
    scene_retry_s: float = Field(default=0.5, gt=0.0)
    # "inline" bakes mesh triangles into the USD (self-contained, which is what a scene string has to be);
    # "reference" leaves the mesh paths for a consumer that fetches files over render_file_request itself.
    usd_mesh_mode: MeshMode = "inline"

    @field_validator("physical_connections", mode="before")
    @classmethod
    def _normalize_physical_connections(cls, value: object) -> object:
        """Accept a class, a (namespaced) Blueprint or an instance name for each value."""
        if not isinstance(value, dict):
            return value
        return {frame: PhysicalConnection.of(v) for frame, v in value.items()}


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
        self._description = self.config.model.load()
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
            self._check_physical_connections()
        except Exception:
            # super().__init__ already took a loop thread and an RPC session; hand them back
            # rather than leave a half-built module behind for the caller to notice.
            self.dispose()
            raise

    def _check_physical_connections(self) -> None:
        """Fail here rather than at get_scene time, which is a blueprint's last step before start."""
        links = {self._description.root_link, *(j.child_link for j in self._description.joints)}
        for frame_id, connection in self.config.physical_connections.items():
            if frame_id not in links:
                raise ValueError(
                    f"physical_connections names frame {frame_id!r}, which is not a link of this "
                    f"URDF (it has {sorted(links)})"
                )
            # An instance name on its own carries no class, so there is nothing to check; it fails
            # at get_scene time instead, which is why a class or a blueprint is the better value.
            if connection.module is not None and SUBTREE_RPC not in connection.module.rpcs:
                raise ValueError(
                    f"physical_connections attaches {connection.module.__name__} at {frame_id!r}, "
                    f"but it has no @rpc {SUBTREE_RPC}() to ask for its subtree"
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
    def get_usd(self) -> str:
        """This robot's own URDF as a USD layer, with nothing attached to it.

        Every module that can be a physical_connections value answers this, UrdfManager included,
        so one bolted to another robot's frame needs no special case.
        """
        with self._usd_lock:
            if self._usd is None:
                self._usd = urdf_to_usd(self._description, mesh_mode=self.config.usd_mesh_mode)
            return self._usd

    @rpc
    async def get_scene(self) -> str:
        """The whole tree as one USD layer: this URDF with every physical_connections subtree on it.

        Async and meant to be called before start(): it waits for every module in the map, because a
        scene missing an arm is worse than a scene that arrives late. Raises TimeoutError when one of
        them has not answered within scene_timeout_s.
        """
        subtrees = await self._subtrees()
        root = self.get_usd()
        return graft_usd(root, subtrees) if subtrees else root

    async def _subtrees(self) -> dict[str, str]:
        connections = self.config.physical_connections
        if not connections:
            return {}
        deadline = time.monotonic() + self.config.scene_timeout_s
        results = await asyncio.gather(
            *(
                self._subtree(frame_id, connection, deadline)
                for frame_id, connection in connections.items()
            )
        )
        return dict(zip(connections, results, strict=True))

    def _subtree_client(self, connection: PhysicalConnection) -> RPCClient:
        """The client for one connection, keyed by remote name so two of one class stay apart.

        Hand-rolled rather than a declared module ref: a ref is one annotated attribute resolved
        by type, and this is a mapping whose keys are only known per robot.
        """
        client = self._subtree_clients.get(connection.remote_name)
        if client is None:
            if connection.module is None:
                # No class to take a signature or an rpc list from, so declare the one rpc we call.
                client = RPCClient(None, None, connection.remote_name, {SUBTREE_RPC}, rpc=self.rpc)
            else:
                client = RPCClient.remote(connection.module, connection.remote_name, rpc=self.rpc)
            self._subtree_clients[connection.remote_name] = client
        return client

    async def _subtree(self, frame_id: str, connection: PhysicalConnection, deadline: float) -> str:
        """The connected module's USD, retried until it answers or the deadline passes."""
        client = self._subtree_client(connection)
        name = connection.remote_name
        attempts = 0
        while True:
            attempts += 1
            usd = await asyncio.to_thread(self._ask_for_usd, client, frame_id)
            if usd:
                logger.info(f"subtree for {frame_id} from {name} after {attempts} ask(s)")
                # A namespaced module's frames are renamed here rather than at its end: it holds
                # one description and does not know which copy of itself it is, while the name it
                # was deployed under says exactly that.
                return await asyncio.to_thread(prefix_usd_frames, usd, connection.frame_prefix)
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{name} did not answer {SUBTREE_RPC}() for frame {frame_id!r} "
                    f"within {self.config.scene_timeout_s}s ({attempts} attempts)"
                )
            await asyncio.sleep(self.config.scene_retry_s)

    def _ask_for_usd(self, client: Any, frame_id: str) -> str | None:
        """One RPC attempt; a module that is not serving yet is a retry, not a failure."""
        try:
            usd = getattr(client, SUBTREE_RPC)()
        except Exception as exc:
            logger.debug(f"{SUBTREE_RPC}() for {frame_id!r} did not answer yet: {exc}")
            return None
        if not isinstance(usd, str) or not usd:
            logger.debug(f"{SUBTREE_RPC}() for {frame_id!r} answered {usd!r}, retrying")
            return None
        return usd

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
