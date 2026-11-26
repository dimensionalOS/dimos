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
``go2/head/lidar/imu``. ``get_usd()`` provides the prims nested under the namespace, so every provided
prim path is already its tf frame and the layers of an assembly overlay each other as they are. The USD fills the module's
``tf_frames_local`` (role = frame path) and ``tf_edges``, so TfModule's checks see every edge.

Each HardwareModule publishes the tf of its own USD; a mounted one also publishes the edge from
the mount frame it sits on to its top prim. The edges that never move (its USD's static ones and
the mount) go out once on ``tf_static`` at start, then on ``tf`` at ``static_tf_rate_hz``. It provides its
USD and every file it reaches on ``hardware_file_request``/``hardware_file_response``, named
relative to itself, so whatever needs the whole assembly (a simulator, on this machine or another)
fetches them with ``fetch_hardware_files`` and composes them (each already sits at its tf frames).
"""

from collections import defaultdict
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass
from functools import cached_property
from typing import TYPE_CHECKING, Any, ClassVar, Protocol, cast, get_type_hints

import numpy as np
from pydantic import model_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.hardware_files import HARDWARE_USD, HardwareFiles
from dimos.core.stream import In, Out
from dimos.core.tf_module import TfModule, TfModuleConfig, _frame_ids
from dimos.msgs.files.FileInfo import FileInfo
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.frame_paths import check_frame_path
from dimos.robot.assets.model import TfEdge, UsdModel
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom

logger = setup_logger()


@dataclass(frozen=True)
class Mount:
    """A frame of a HardwareModule's USD that other hardware mounts onto; ``Robot.mounts.<name>`` gives one."""

    frame: str


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

    def __getattr__(self, name: str) -> Mount:
        if name.startswith("_"):
            raise AttributeError(name)
        if name not in self._frames:
            raise AttributeError(f"no mount {name!r}; the mounts are {sorted(self._frames)}")
        return Mount(self._frames[name])

    def __iter__(self) -> Iterator[str]:
        return iter(sorted(self._frames.values()))

    def __contains__(self, frame: object) -> bool:
        return frame in self._frames.values()


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
        check_declared_tf_against_usd(vars(self))
        check_attached_namespace(self.attached_to, (self.instance_name or "").rpartition("/")[0])
        # The parent's validator is a plain function at runtime; mypy sees pydantic's decorator proxy.
        cast("Callable[[TfModuleConfig], TfModuleConfig]", TfModuleConfig._check_roles)(self)
        return self


def check_declared_tf_against_usd(
    values: Mapping[str, Any], mounts: "Mounts | None" = None
) -> None:
    """Refuse a HardwareModule's declared tf unless it names its USD: each local frame a USD prim path, each edge a USD edge, no global frames.

    ``values`` are a HardwareModuleConfig's fields or a blueprint atom's kwargs.
    """
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
    if values.get("skip_tf_checks"):
        raise ValueError(
            "a HardwareModule cannot skip_tf_checks: it publishes its USD's edges and its mount, "
            "and the check is what keeps it to those"
        )
    frames = usd.tf_frames
    for mount in mounts or ():
        if mount not in frames:
            raise ValueError(
                f"the mount {mount!r} is not a frame of the USD (its frames: {sorted(frames)})"
            )
    local: dict[str, str] = values["tf_frames_local"]
    for role, frame in local.items():
        if frame not in frames:
            raise ValueError(
                f"tf_frames_local[{role!r}] is {frame!r}, which is not a prim path of this "
                f"hardware's USD (its frames: {sorted(frames)})"
            )
    if values["tf_frames_global"]:
        raise ValueError(
            f"tf_frames_global is {values['tf_frames_global']!r}, but a HardwareModule publishes "
            f"only its USD's frames and its mount; an estimate (odom, map, ...) belongs in a "
            f"separate module that is not a HardwareModule"
        )
    usd_edges = set(usd.tf_edges)
    for parent, child in values["tf_edges"]:
        if (local.get(parent), local.get(child)) not in usd_edges:
            raise ValueError(
                f"tf_edges holds ({parent!r}, {child!r}), which is not an edge of this hardware's "
                f"USD; a HardwareModule never publishes an estimate (odom -> base, ...), so that "
                f"belongs in a separate module that is not a HardwareModule"
            )


def check_attached_namespace(attached_to: str | None, namespace: str) -> None:
    """Refuse a mounted HardwareModule that is not under its mount's namespace."""
    if (
        attached_to is not None
        and namespace != attached_to
        and not namespace.endswith(f"/{attached_to}")
    ):
        raise ValueError(
            f"attached_to is {attached_to!r} but the namespace is {namespace!r}; a mounted "
            f"HardwareModule sits under its mount's namespace (use .attach(mount))"
        )


# needed for sims to detect hardware modules on hosted systems
class HardwareSpec(Spec, Protocol):
    def get_usd(self) -> str: ...
    # possible to have a namespace without being mounted, this differentiates that
    def attached_to(self) -> str | None: ...
    # The (parent, child) frame ids it publishes, so a recorder can tell which edges are hardware.
    def get_hardware_transform_ids(self) -> list[tuple[str, str]]: ...


class HardwareModule(TfModule):
    """Hardware with its own USD; each namespace has one unmounted HardwareModule, the rest are mounted.

    It publishes only its USD's edges and its mount, never an estimate; ``get_hardware_transform_ids()``
    names them, which is how a recorder tells which edges are hardware.
    """

    config: HardwareModuleConfig
    # Frames of this hardware's USD that other hardware can mount onto.
    mounts: ClassVar[Mounts] = Mounts()
    # A FileInfo naming a file this hardware provides, with the sha256 the asker already has.
    hardware_file_request: In[FileInfo]
    # The reply (header only when the asker's sha256 is current), and the hardware.usda header announced at start.
    hardware_file_response: Out[FileInfo]
    # The edges that never move, once at start (not latched: only what listens then gets them).
    tf_static: Out[TFMessage]

    @classmethod
    def attach(cls, mount: Mount, **kwargs: Any) -> "Blueprint":
        """This hardware on ``mount`` (``Robot.mounts.<name>``), under the namespace of its frame's path (``a/b/c/<module>``)."""
        if not isinstance(mount, Mount):
            raise TypeError(
                f"attach takes a mount (Robot.mounts.<name>), not {type(mount).__name__} {mount!r}"
            )
        blueprint = cls.blueprint(**kwargs, attached_to=mount.frame)
        # namespace() puts its prefix in front, so the last segment goes on first.
        for segment in reversed(mount.frame.split("/")):
            blueprint = blueprint.namespace(segment)
        return blueprint

    @cached_property
    def tf_frames(self) -> dict[str, str]:
        """Every role -> frame id: each USD frame under its own prim path, then the declared roles."""
        usd_frames = {frame: frame for frame in self.config.usd.tf_frames}
        return {
            **_frame_ids(usd_frames, {}, self.namespace),
            **_frame_ids(self.config.tf_frames_local, self.config.tf_frames_global, self.namespace),
        }

    @cached_property
    def tf_edge_ids(self) -> frozenset[tuple[str, str]]:
        """What tf and tf_static may carry, as frame ids: the declared edges, the USD's, and the mount."""
        frames = self.tf_frames
        usd_edges = {(frames[parent], frames[child]) for parent, child in self.config.usd.tf_edges}
        declared = {(frames[parent], frames[child]) for parent, child in self.config.tf_edges}
        mount = (
            {(self.namespace, self.attach_frame)} if self.config.attached_to is not None else set()
        )
        return frozenset(declared | usd_edges | mount)

    @rpc
    def get_hardware_transform_ids(self) -> list[tuple[str, str]]:
        """Every (parent, child) frame id pair this hardware publishes on tf and tf_static: its USD's edges and its mount."""
        return sorted(self.tf_edge_ids)

    @staticmethod
    def blueprint_checks(atoms: "list[BlueprintAtom]") -> None:
        """Refuse a namespace with more than one unmounted HardwareModule, and mounts that do not form a tree."""
        hardware = [atom for atom in atoms if issubclass(atom.module, HardwareModule)]
        unmounted: dict[str, list[str]] = defaultdict(list)
        for atom in hardware:
            namespace = atom.name.rpartition("/")[0]
            # Its namespace is a prefix of its USD's prim paths, so each segment must be a prim name.
            if namespace:
                check_frame_path(namespace, f"{atom.name}'s namespace")
            # Validated as the module will be, which reads a USD the blueprint config took apart back into one.
            config = get_type_hints(atom.module)["config"].model_validate(atom.kwargs)
            check_declared_tf_against_usd(
                vars(config), cast("type[HardwareModule]", atom.module).mounts
            )
            check_attached_namespace(atom.kwargs.get("attached_to"), namespace)
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
        self.provide_hardware_files()
        self.publish_static_tf()

    def publish_static_tf(self) -> None:
        """Publish the edges that never move once on tf_static, then on tf at static_tf_rate_hz; start() does this."""
        if not self._static_edges:
            return
        self.tf_static.publish(TFMessage(*self.static_transforms(ts=self.time())))
        if self.config.static_tf_rate_hz > 0:
            republishing = self.spawn(self._republish_static_tf())
            self.register_disposable(Disposable(republishing.cancel))

    async def _republish_static_tf(self) -> None:
        while True:
            self.tfbuffer.publish(*self.static_transforms(ts=self.time()))
            await self.sleep(1 / self.config.static_tf_rate_hz)

    def provide_hardware_files(self) -> None:
        """Answer hardware_file_request and announce hardware.usda; start() does this once its streams are wired."""
        self.register_disposable(
            Disposable(self.hardware_file_request.subscribe(self._on_hardware_file_request))
        )
        try:
            files = self._hardware_files
        except ImportError as error:
            logger.warning(f"{self.config.instance_name} provides no hardware files: {error}")
            return
        self.hardware_file_response.publish(files.announcement())

    @rpc
    def get_usd(self) -> str:
        """This hardware's USD exactly as provided as hardware.usda: prims under the namespace, so each frame's prim path is its tf frame; files relative to it."""
        return self._hardware_files.usd

    @rpc
    def attached_to(self) -> str | None:
        """The tf frame this hardware is mounted on, or None if nothing holds it."""
        return self.namespace if self.config.attached_to is not None else None

    @cached_property
    def _hardware_files(self) -> HardwareFiles:
        from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
        from dimos.robot.assets.usd_utils.layers import usd_under_namespace

        # Its prims under /<namespace> and files as paths on this machine; HardwareFiles makes them relative to provide them.
        source = self.config.usd.source_path
        root = str(source.parent) if source is not None else ""
        usd = usd_under_namespace(
            resolve_usd_assets(self.config.usd.usd, root), self.namespace, root or None
        )
        return HardwareFiles(usd, source.stat().st_mtime if source and source.is_file() else None)

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
        """The edges that never move (the USD's static ones and the mount), stamped ``ts``."""
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
        self, positions: Mapping[str, float] | None = None, ts: float | None = None
    ) -> list[Transform]:
        """The USD's edges that move, at ``positions`` (the USD's own names; one not named sits at zero)."""
        return self._usd_edges_as_namespaced_transforms(
            self.config.usd.moving_transforms(positions), ts
        )

    def _usd_edges_as_namespaced_transforms(
        self, edges: "list[TfEdge]", ts: float | None
    ) -> list[Transform]:
        """The USD's ``edges`` as Transforms stamped ``ts``, between this hardware's tf frame ids (its namespace in front)."""
        return [
            Transform(
                translation=Vector3(*edge.translation),
                rotation=Quaternion(*edge.rotation),
                ts=ts,
                frame_id=self.tf_frames[edge.parent],
                child_frame_id=self.tf_frames[edge.child],
            )
            for edge in edges
        ]

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The USD's static edges and the mount, worked out once and only restamped; a subclass may narrow them."""
        transforms = self._usd_edges_as_namespaced_transforms(
            self.config.usd.static_transforms(), None
        )
        if self.config.attached_to is not None:
            transforms.append(
                Transform.from_matrix(
                    np.eye(4),
                    frame_id=self.namespace,
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
