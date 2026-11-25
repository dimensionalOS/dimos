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

"""A piece of hardware described by its own USD, which other hardware can mount onto by name.

A robot's USD holds the robot alone. A sensor on it is its own HardwareModule with its own USD,
mounted onto one of the robot's mount frames::

    autoconnect(Robot.blueprint(), Lidar.attach(Robot.mounts.lidar))

A frame is named by its prim path, with the namespace in front: the Go2's ``/go2/head`` is
``go2/head``, and a lidar whose USD holds ``/lidar/imu``, attached at ``go2/head``, is
``go2/head/lidar/imu``. ``get_usd()`` serves the prims nested under the namespace, so every served
prim path is already its tf frame and the layers of an assembly overlay each other as they are. The USD fills the module's
``tf_frames_local`` (role = frame path) and ``tf_edges``, so TfModule's checks see every edge.

Each HardwareModule publishes the tf of its own USD; a mounted one also publishes the edge from
the mount frame it sits on to its top prim. The edges that never move (fixed joints and the
mount) go out once on ``tf_static`` at start, then on ``tf`` at ``static_tf_rate_hz``. It serves its
USD and every file it reaches on ``hardware_file_request``/``hardware_file_response``, named
relative to itself, so whatever needs the whole assembly (a simulator, on this machine or another)
fetches them with ``fetch_hardware_files`` and composes them (each already sits at its tf frames).
"""

from collections import defaultdict
from collections.abc import Callable, Iterable, Iterator, Mapping
from functools import cached_property
from typing import TYPE_CHECKING, Any, ClassVar, Protocol, cast

import numpy as np
from pydantic import model_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.hardware_files import HARDWARE_USD, HardwareFiles
from dimos.core.stream import In, Out
from dimos.core.tf_module import TfIO, TfModule, TfModuleConfig
from dimos.msgs.files.FileInfo import FileInfo
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.frame_paths import check_frame_path
from dimos.robot.assets.joint_transforms import JointTransforms
from dimos.robot.assets.model import JointDescription, UsdModel
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom

logger = setup_logger()


class Mounts:
    """The frames of a HardwareModule's USD that other hardware mounts onto, by last segment: ``Robot.mounts.lidar`` is ``go2/base/lidar``."""

    def __init__(self, *frames: str) -> None:
        self._frames: dict[str, str] = {}
        for frame in frames:
            check_frame_path(frame, "the mount")
            name = frame.rpartition("/")[2]
            if name in self._frames:
                raise ValueError(
                    f"the mounts {self._frames[name]!r} and {frame!r} both end in {name!r}, "
                    f"which is how a mount is named"
                )
            self._frames[name] = frame

    def __getattr__(self, name: str) -> str:
        if name.startswith("_"):
            raise AttributeError(name)
        if name not in self._frames:
            raise AttributeError(f"no mount {name!r}; the mounts are {sorted(self._frames)}")
        return self._frames[name]

    def __iter__(self) -> Iterator[str]:
        return iter(sorted(self._frames.values()))

    def __contains__(self, frame: object) -> bool:
        return frame in self._frames.values()


# The role of the frame a mounted HardwareModule sits on: its namespace, another module's frame.
MOUNT_ROLE = "mount"


class HardwareModuleConfig(TfModuleConfig):
    # This hardware alone: a robot's USD has no sensors in it, they mount on with their own.
    # Required: hardware without a USD is not a HardwareModule.
    usd: UsdModel
    # The mount frame of the hardware this sits on (set by HardwareModule.attach); None for the hardware everything mounts to.
    attached_to: str | None = None
    # The edges that never move go out once on tf_static at start, then on tf this often (0: never on tf), for whatever joins later.
    static_tf_rate_hz: float = 10.0

    @model_validator(mode="after")
    def _check_roles(self) -> "HardwareModuleConfig":
        # Filled before TfModuleConfig's role check runs, so tf_edges may name the USD's frames.
        self.tf_frames_local, self.tf_frames_global, self.tf_edges = usd_tf(
            self, (self.instance_name or "").rpartition("/")[0]
        )
        # The parent's validator is a plain function at runtime; mypy sees pydantic's decorator proxy.
        cast("Callable[[TfModuleConfig], TfModuleConfig]", TfModuleConfig._check_roles)(self)
        return self


def usd_tf(
    config: Any, namespace: str, mounts: "Mounts | None" = None
) -> tuple[dict[str, str], dict[str, str], list[tuple[str, str]]]:
    """A HardwareModule's tf: its USD's frames (role = frame path) and edges, its mount edge, then what its config adds.

    ``config`` is a HardwareModuleConfig or a blueprint atom's kwargs (read the same way).
    """
    values: Mapping[str, Any] = config if isinstance(config, Mapping) else vars(config)
    usd: UsdModel | None = values.get("usd")
    if usd is None:
        raise ValueError(
            "a HardwareModule needs a usd: hardware without one is not a HardwareModule"
        )
    if values.get("frame_id_prefix"):
        raise ValueError(
            "a HardwareModule's tf frames are its namespace and its USD's prim paths, so "
            "frame_id_prefix (which would name them apart from the USD) is refused; namespace "
            "the blueprint instead"
        )
    frames = usd.frame_names()
    for mount in mounts or ():
        if mount not in frames:
            raise ValueError(
                f"the mount {mount!r} is not a frame of the USD (its frames: {sorted(frames)})"
            )
    # A floating joint has no single position: what moves its child (odometry) is another parent.
    joints = [joint for joint in usd.joints if joint.type != "floating"]
    edges = [(edge.parent_link, edge.child_link) for edge in (*joints, *usd.prim_edges)]
    # A joint may name a frame no prim carries: the anchor usd_to_urdf adds off a child's origin.
    local = {frame: frame for frame in sorted({*frames, *(f for edge in edges for f in edge)})}
    shared: dict[str, str] = {}
    attached_to: str | None = values.get("attached_to")
    if attached_to is not None:
        check_frame_path(attached_to, "attached_to")
        if namespace != attached_to and not namespace.endswith(f"/{attached_to}"):
            raise ValueError(
                f"attached_to is {attached_to!r} but the namespace is {namespace!r}; a mounted "
                f"HardwareModule sits under its mount's namespace (use .attach(mount))"
            )
        shared[MOUNT_ROLE] = namespace
        edges.append((MOUNT_ROLE, usd.top_frame))
    for role, frame in values["tf_frames_local"].items():
        if frame not in local:
            raise ValueError(
                f"tf_frames_local[{role!r}] is {frame!r}, which is not a frame of this hardware's "
                f"USD (its frames: {sorted(frames)})"
            )
        if local.setdefault(role, frame) != frame:
            raise ValueError(
                f"tf_frames_local[{role!r}] is {frame!r}, but the USD's frame {role!r} is that role"
            )
    if values.get("skip_tf_checks"):
        raise ValueError(
            "a HardwareModule cannot skip_tf_checks: it publishes its USD's edges and its mount, "
            "and the check is what keeps it to those"
        )
    for role, frame in values["tf_frames_global"].items():
        if shared.get(role) != frame:
            raise ValueError(
                f"tf_frames_global[{role!r}] is {frame!r}, but a HardwareModule publishes only its "
                f"USD's frames and its mount; an estimate (odom, map, ...) belongs in a separate "
                f"module that is not a HardwareModule"
            )
    roles = {**local, **shared}
    usd_frames = {(roles[parent], roles[child]) for parent, child in edges}
    for parent, child in values["tf_edges"]:
        if (roles.get(parent), roles.get(child)) not in usd_frames:
            raise ValueError(
                f"tf_edges holds ({parent!r}, {child!r}), which is not an edge of this hardware's USD "
                f"or its mount; a HardwareModule never publishes an estimate (odom -> base, ...), "
                f"so that belongs in a separate module that is not a HardwareModule"
            )
    return local, shared, list(dict.fromkeys([*edges, *values["tf_edges"]]))


# needed for sims to detect hardware modules on hosted systems
class HardwareSpec(Spec, Protocol):
    def get_usd(self) -> str: ...
    # possible to have a namespace without being mounted, this differentiates that
    def attached_to(self) -> str | None: ...
    def get_namespace(self) -> str: ...


class _HardwareTfIO(TfIO):
    """A HardwareModule's tf port: every frame it publishes goes on the wire with a leading "/"."""

    owner: "HardwareModule"

    def publish(self, msg: TFMessage) -> None:
        super().publish(TFMessage(*self.owner.on_wire(msg.transforms)))


class HardwareModule(TfModule):
    """Hardware with its own USD; each namespace has one unmounted HardwareModule, the rest are mounted.

    On the wire every frame it publishes carries one leading "/" (``/go2_A/go2/base``), which says the
    edge is hardware; the tf buffer drops it, so code never sees it.
    """

    config: HardwareModuleConfig
    # Frames of this hardware's USD that other hardware can mount onto.
    mounts: ClassVar[Mounts] = Mounts()
    # A FileInfo naming a file this hardware serves, with the sha256 the asker already has.
    hardware_file_request: In[FileInfo]
    # The reply (header only when the asker's sha256 is current), and the hardware.usda header announced at start.
    hardware_file_response: Out[FileInfo]
    # The edges that never move, once at start (not latched: only what listens then gets them).
    tf_static: Out[TFMessage]
    tf_slashes_frames: ClassVar[bool] = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.tf = _HardwareTfIO(TFMessage, "tf", self)

    @classmethod
    def attach(cls, mount_frame: str, **kwargs: Any) -> "Blueprint":
        """This hardware on the mount frame ``mount_frame``, under the namespace of the same path (``a/b/c/<module>``)."""
        check_frame_path(mount_frame, "the mount frame")
        blueprint = cls.blueprint(attached_to=mount_frame, **kwargs)
        # namespace() puts its prefix in front, so the last segment goes on first.
        for segment in reversed(mount_frame.split("/")):
            blueprint = blueprint.namespace(segment)
        return blueprint

    @classmethod
    def blueprint_tf(
        cls, atom: "BlueprintAtom"
    ) -> tuple[dict[str, str], dict[str, str], list[tuple[str, str]]]:
        """The tf this hardware's USD and config declare, as its config would fill it in."""
        return usd_tf(atom.kwargs, atom.name.rpartition("/")[0], cls.mounts)

    def on_wire(self, transforms: "Iterable[Transform]") -> list[Transform]:
        """``transforms`` as they go on the wire: every frame with a leading "/"."""
        return [
            Transform(
                translation=t.translation,
                rotation=t.rotation,
                frame_id=f"/{t.frame_id}",
                child_frame_id=f"/{t.child_frame_id}",
                ts=t.ts,
            )
            for t in transforms
        ]

    @staticmethod
    def blueprint_checks(atoms: "list[BlueprintAtom]") -> None:
        """Refuse a namespace with more than one unmounted HardwareModule, and mounts that do not form a tree."""
        hardware = [atom for atom in atoms if issubclass(atom.module, HardwareModule)]
        unmounted: dict[str, list[str]] = defaultdict(list)
        for atom in hardware:
            if atom.kwargs.get("attached_to") is None:
                unmounted[atom.name.rpartition("/")[0]].append(atom.name)
        errors = [
            f"    - {namespace or 'the top level'}: {', '.join(sorted(names))}"
            for namespace, names in sorted(unmounted.items())
            if len(names) > 1
        ]
        if errors:
            raise ValueError(
                "\n".join(
                    [
                        "Blueprint cannot start: a namespace has more than one unmounted "
                        "HardwareModule (attach all but one with .attach(mount)).",
                        *errors,
                    ]
                )
            )
        _check_mount_tree(hardware)

    @rpc
    def start(self) -> None:
        super().start()
        self.serve_hardware_files()
        self.publish_static_tf()

    def publish_static_tf(self) -> None:
        """Publish the edges that never move once on tf_static, then on tf at static_tf_rate_hz; start() does this."""
        if not self._static_edges:
            return
        self.tf_static.publish(TFMessage(*self.on_wire(self.static_transforms(ts=self.time()))))
        if self.config.static_tf_rate_hz > 0:
            republishing = self.spawn(self._republish_static_tf())
            self.register_disposable(Disposable(republishing.cancel))

    async def _republish_static_tf(self) -> None:
        while True:
            self.tfbuffer.publish(*self.on_wire(self.static_transforms(ts=self.time())))
            await self.sleep(1 / self.config.static_tf_rate_hz)

    def serve_hardware_files(self) -> None:
        """Answer hardware_file_request and announce hardware.usda; start() does this once its streams are wired."""
        self.register_disposable(
            Disposable(self.hardware_file_request.subscribe(self._on_hardware_file_request))
        )
        try:
            files = self._hardware_files
        except ImportError as error:
            logger.warning(f"{self.config.instance_name} serves no hardware files: {error}")
            return
        self.hardware_file_response.publish(files.announcement())

    @rpc
    def get_usd(self) -> str:
        """This hardware's USD exactly as served as hardware.usda: prims under the namespace, so each frame's prim path is its tf frame; files relative to it."""
        return self._hardware_files.usd

    @rpc
    def attached_to(self) -> str | None:
        """The tf frame this hardware is mounted on, or None if nothing holds it."""
        return self.namespace if self.config.attached_to is not None else None

    def local_usd(self) -> str:
        """This hardware's USD for its own use: its prims under ``/<namespace>``, so each frame's prim path is its tf frame; files as paths on this machine."""
        from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
        from dimos.robot.assets.usd_utils.layers import usd_under_namespace

        source = self.config.usd.source_path
        root = str(source.parent) if source is not None else ""
        return usd_under_namespace(
            resolve_usd_assets(self.config.usd.usd, root), self.namespace, root or None
        )

    @cached_property
    def _hardware_files(self) -> HardwareFiles:
        source = self.config.usd.source_path
        return HardwareFiles(
            self.local_usd(), source.stat().st_mtime if source and source.is_file() else None
        )

    def _on_hardware_file_request(self, request: FileInfo) -> None:
        # Answered on the request stream's own thread, so a burst (one per mesh) is never coalesced.
        try:
            files = self._hardware_files
        except ImportError:
            if request.uri == HARDWARE_USD:
                self.hardware_file_response.publish(FileInfo(HARDWARE_USD))
            return
        reply = files.reply(request)
        if reply is not None:
            self.hardware_file_response.publish(reply)

    @property
    def attach_frame(self) -> str:
        """The frame the mount holds this hardware by: its USD's top prim, whose path-parent the mount is."""
        return self.tf_frames[self.config.usd.top_frame]

    def static_transforms(self, ts: float | None = None) -> list[Transform]:
        """The edges that never move (fixed joints and the mount), stamped ``ts``."""
        return [
            Transform(
                translation=edge.translation,
                rotation=edge.rotation,
                ts=ts,
                frame_id=edge.frame_id,
                child_frame_id=edge.child_frame_id,
            )
            for edge in self._static_edges
        ]

    def moving_transforms(
        self, joint_positions: Mapping[str, float] | None = None, ts: float | None = None
    ) -> list[Transform]:
        """The edges of joints that move, at ``joint_positions`` (missing joints sit at zero)."""
        return self._edge_transforms(self._moving_edges, joint_positions or {}, ts)

    def _edge_transforms(
        self, edges: JointTransforms, positions: Mapping[str, float], ts: float | None
    ) -> list[Transform]:
        translations, rotations = edges.at(positions)
        return [
            Transform(
                translation=Vector3(*translation),
                rotation=Quaternion(*rotation),
                ts=ts,
                frame_id=self.tf_frames[parent],
                child_frame_id=self.tf_frames[child],
            )
            for (parent, child), translation, rotation in zip(
                edges.links, translations.tolist(), rotations.tolist(), strict=True
            )
        ]

    @cached_property
    def _posable_joints(self) -> tuple[JointDescription, ...]:
        # A floating joint has no single position: the robot owning it publishes its pose from odometry.
        return tuple(j for j in self.config.usd.joints if j.type != "floating")

    @cached_property
    def _moving_edges(self) -> JointTransforms:
        # Worked out once: a robot publishes these edges at every tick, and only the positions change.
        return JointTransforms(tuple(j for j in self._posable_joints if j.type != "fixed"))

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The fixed joints, the prim edges and the mount, worked out once and only restamped; a subclass may narrow them."""
        fixed = tuple(j for j in self._posable_joints if j.type == "fixed")
        transforms = self._edge_transforms(
            JointTransforms((*fixed, *self.config.usd.prim_edges)), {}, None
        )
        if self.config.attached_to is not None:
            transforms.append(
                Transform.from_matrix(
                    np.eye(4),
                    frame_id=self.tf_frames[MOUNT_ROLE],
                    child_frame_id=self.attach_frame,
                )
            )
        return transforms


def _check_mount_tree(hardware: "list[BlueprintAtom]") -> None:
    """Refuse a mount no other HardwareModule offers under its real namespace, and two HardwareModules answering on one namespace."""
    by_namespace: dict[str, list[str]] = defaultdict(list)
    offered_by: dict[str, list[str]] = defaultdict(list)
    for atom in hardware:
        namespace = atom.name.rpartition("/")[0]
        by_namespace[namespace].append(atom.name)
        for mount in cast("type[HardwareModule]", atom.module).mounts:
            offered_by[f"{namespace}/{mount}" if namespace else mount].append(atom.name)
    errors = [
        f"    - {namespace or 'the top level'} holds {', '.join(sorted(names))}, whose "
        f"hardware_file streams would answer each other's requests"
        for namespace, names in sorted(by_namespace.items())
        if len(names) > 1
    ]
    errors += [
        f"    - '{frame}' is offered by several HardwareModules: {', '.join(sorted(names))}"
        for frame, names in sorted(offered_by.items())
        if len(names) > 1
    ]
    for atom in sorted(hardware, key=lambda atom: atom.name):
        if atom.kwargs.get("attached_to") is None:
            continue
        # A mounted module sits on the frame named by its namespace (attached_to).
        frame = atom.name.rpartition("/")[0]
        if frame not in offered_by:
            errors.append(
                f"    - {atom.name} is mounted on '{frame}', which no other HardwareModule in the "
                f"blueprint offers (is it namespaced apart from the hardware it mounts on?)"
            )
    if errors:
        offered = ", ".join(f"'{frame}'" for frame in sorted(offered_by)) or "none"
        raise ValueError(
            "\n".join(
                [
                    "Blueprint cannot start: its HardwareModules' mounts do not form a tree "
                    f"(mount frames offered: {offered}).",
                    *errors,
                ]
            )
        )
