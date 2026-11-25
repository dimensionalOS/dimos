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

Each HardwareModule publishes the tf of its own USD; a mounted one also publishes the edge from
the mount frame it sits on to its own frames. The edges that never move (fixed joints and the
mount) go out once on ``tf_static`` at start, then on ``tf`` at ``static_tf_rate_hz``. It serves its
USD and every file it reaches on ``hardware_file_request``/``hardware_file_response``, named
relative to itself, so whatever needs the whole assembly (a simulator, on this machine or another)
fetches them with ``fetch_hardware_files`` and grafts them where ``attached_to()`` says.
"""

from collections import defaultdict
from collections.abc import Iterator, Mapping
from functools import cached_property
from typing import TYPE_CHECKING, Any, ClassVar, Protocol, cast

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.hardware_files import HARDWARE_USD, HardwareFiles
from dimos.core.stream import In, Out
from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.files.FileInfo import FileInfo
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF
from dimos.robot.assets.model import JointDescription, UsdModel
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom

logger = setup_logger()


class Mounts:
    """The frames of a HardwareModule's USD that other hardware mounts onto: ``Robot.mounts.lidar``."""

    def __init__(self, *frames: str) -> None:
        self._frames = frozenset(frames)

    def __getattr__(self, frame: str) -> str:
        if frame.startswith("_"):
            raise AttributeError(frame)
        if frame not in self._frames:
            raise AttributeError(f"no mount {frame!r}; the mounts are {sorted(self._frames)}")
        return frame

    def __iter__(self) -> Iterator[str]:
        return iter(sorted(self._frames))

    def __contains__(self, frame: object) -> bool:
        return frame in self._frames


class HardwareModuleConfig(TfModuleConfig):
    # This hardware alone: a robot's USD has no sensors in it, they mount on with their own.
    # Required: hardware without a USD is not a HardwareModule.
    usd: UsdModel
    # The mount frame of the hardware this sits on (set by HardwareModule.attach); None for the hardware everything mounts to.
    attached_to: str | None = None
    # The edges that never move go out once on tf_static at start, then on tf this often (0: never on tf), for whatever joins later.
    static_tf_rate_hz: float = 10.0


# needed for sims to detect hardware modules on hosted systems
class HardwareSpec(Spec, Protocol):
    def get_usd(self) -> str: ...
    # possible to have a namespace without being mounted, this differentiates that
    def attached_to(self) -> str | None: ...
    def get_namespace(self) -> str: ...


class HardwareModule(TfModule):
    """Hardware with its own USD; each namespace has one unmounted HardwareModule, the rest are mounted."""

    config: HardwareModuleConfig
    # Frames of this hardware's USD that other hardware can mount onto.
    mounts: ClassVar[Mounts] = Mounts()
    # A FileInfo naming a file this hardware serves, with the sha256 the asker already has.
    hardware_file_request: In[FileInfo]
    # The reply (header only when the asker's sha256 is current), and the hardware.usda header announced at start.
    hardware_file_response: Out[FileInfo]
    # The edges that never move, once at start (not latched: only what listens then gets them).
    tf_static: Out[TFMessage]

    @classmethod
    def attach(cls, mount_frame: str, **kwargs: Any) -> "Blueprint":
        """This hardware on the mount frame ``mount_frame``, under the namespace of the same name."""
        return cls.blueprint(attached_to=mount_frame, **kwargs).namespace(mount_frame)

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
        self.tf_static.publish(TFMessage(*self.static_transforms(ts=self.time())))
        if self.config.static_tf_rate_hz > 0:
            republishing = self.spawn(self._republish_static_tf())
            self.register_disposable(Disposable(republishing.cancel))

    async def _republish_static_tf(self) -> None:
        while True:
            self.tfbuffer.publish(*self.static_transforms(ts=self.time()))
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
        """This hardware's USD exactly as served as hardware.usda: frames named as on tf, files relative to it."""
        return self._hardware_files.usd

    @rpc
    def attached_to(self) -> str | None:
        """The tf frame this hardware is mounted on, or None if nothing holds it."""
        return self.namespace if self.config.attached_to is not None else None

    def local_usd(self) -> str:
        """This hardware's USD for its own use: frames named as on tf, files as paths on this machine."""
        from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
        from dimos.robot.assets.usd_utils.frames import prefix_usd_frames

        source = self.config.usd.source_path
        root = str(source.parent) if source is not None else ""
        return prefix_usd_frames(
            resolve_usd_assets(self.config.usd.usd, root), self.frame_id_prefix, root or None
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

    def usd_frame(self, name: str) -> str:
        """The tf frame id of the frame ``name`` in this hardware's USD."""
        return f"{self.frame_id_prefix}/{name}" if self.frame_id_prefix else name

    @property
    def attach_frame(self) -> str:
        """The frame the mount holds this hardware by; a mobile robot holds by its odometry frame instead."""
        return self.usd_frame(self.config.usd.root_link)

    def usd_transforms(
        self, joint_positions: Mapping[str, float] | None = None, ts: float | None = None
    ) -> list[Transform]:
        """Every edge of this hardware's own tf at ``joint_positions`` (missing joints sit at zero), plus its mount."""
        return [*self.static_transforms(ts), *self.moving_transforms(joint_positions, ts)]

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

    def publish_usd_tf(
        self, joint_positions: Mapping[str, float] | None = None, ts: float | None = None
    ) -> None:
        """Publish the edges that move at ``joint_positions``; the static ones go out on their own."""
        transforms = self.moving_transforms(joint_positions, ts)
        if transforms:
            self.tfbuffer.publish(*transforms)

    def _edge_transforms(
        self, edges: "_JointEdges", positions: Mapping[str, float], ts: float | None
    ) -> list[Transform]:
        at = np.array([_position(joint, positions) for joint in edges.joints], dtype=np.float64)
        translations, rotations = edges.at(at)
        return [
            Transform(
                translation=Vector3(*translation),
                rotation=Quaternion(*rotation),
                ts=ts,
                frame_id=self.usd_frame(parent),
                child_frame_id=self.usd_frame(child),
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
    def _moving_edges(self) -> "_JointEdges":
        # Worked out once: a robot publishes these edges at every tick, and only the positions change.
        return _JointEdges(tuple(j for j in self._posable_joints if j.type != "fixed"))

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The fixed joints and the mount, worked out once and only restamped; a subclass may narrow them."""
        fixed = _JointEdges(tuple(j for j in self._posable_joints if j.type == "fixed"))
        transforms = self._edge_transforms(fixed, {}, None)
        attached_to = self.attached_to()
        if attached_to is not None:
            transforms.append(
                Transform.from_matrix(
                    np.eye(4), frame_id=attached_to, child_frame_id=self.attach_frame
                )
            )
        return transforms

    def _create_tfbuffer(self, port: Any) -> TF:
        if self.config.skip_tf_checks:
            return super()._create_tfbuffer(port)
        frames = self.tf_frames
        declared = {(frames[parent], frames[child]) for parent, child in self.config.tf_edges}
        own = {(t.frame_id, t.child_frame_id) for t in self.usd_transforms()}
        return TF(
            port,
            edges=declared | own,
            module_name=self.config.instance_name or type(self).__name__,
        )


def _check_mount_tree(hardware: "list[BlueprintAtom]") -> None:
    """Refuse a mount no other HardwareModule offers under its real namespace, and two HardwareModules answering on one namespace."""
    by_namespace: dict[str, list[str]] = defaultdict(list)
    offered_by: dict[str, list[str]] = defaultdict(list)
    for atom in hardware:
        namespace = atom.name.rpartition("/")[0]
        by_namespace[namespace].append(atom.name)
        prefix = "/".join(part for part in (namespace, atom.kwargs.get("frame_id_prefix")) if part)
        for mount in cast("type[HardwareModule]", atom.module).mounts:
            offered_by[f"{prefix}/{mount}" if prefix else mount].append(atom.name)
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


class _JointEdges:
    """A hardware's posable joints as arrays, so every edge of its tf is worked out in one step."""

    def __init__(self, joints: tuple[JointDescription, ...]) -> None:
        self.joints = joints
        self.links = [(joint.parent_link, joint.child_link) for joint in joints]
        origins = np.array([joint.origin() for joint in joints]).reshape(-1, 4, 4)
        self._origin_rotation = origins[:, :3, :3]
        self._origin_translation = origins[:, :3, 3]
        self._origin_quaternion = _quaternions(self._origin_rotation)
        axes = np.array([joint.axis for joint in joints], dtype=np.float64).reshape(-1, 3)
        self._axis = axes / np.linalg.norm(axes, axis=1, keepdims=True)
        kinds = [joint.type for joint in joints]
        unknown = sorted(
            {k for k in kinds if k not in ("fixed", "revolute", "continuous", "prismatic")}
        )
        if unknown:
            raise ValueError(f"joints of type {unknown} have no single position")
        self._turns = np.array([k in ("revolute", "continuous") for k in kinds], dtype=bool)
        self._slides = np.array([k == "prismatic" for k in kinds], dtype=bool)

    def at(self, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Each edge's translation and its rotation as an x, y, z, w quaternion, at these joint positions."""
        turned = np.where(self._turns, positions, 0.0)
        half = turned / 2.0
        motion = np.concatenate([self._axis * np.sin(half)[:, None], np.cos(half)[:, None]], axis=1)
        rotations = _canonical(_multiply(self._origin_quaternion, motion))
        slid = self._axis * np.where(self._slides, positions, 0.0)[:, None]
        translations = self._origin_translation + np.einsum(
            "nij,nj->ni", self._origin_rotation, slid
        )
        return translations, rotations


def _multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Row-wise Hamilton product of x, y, z, w quaternions."""
    ax, ay, az, aw = a.T
    bx, by, bz, bw = b.T
    return np.stack(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        axis=1,
    )


def _canonical(quaternions: np.ndarray) -> np.ndarray:
    return np.where(quaternions[:, 3:4] < 0.0, -quaternions, quaternions)


def _quaternions(rotations: np.ndarray) -> np.ndarray:
    """x, y, z, w quaternions of rotation matrices, once at setup."""
    if not len(rotations):
        return np.zeros((0, 4))
    from scipy.spatial.transform import Rotation

    return _canonical(np.asarray(Rotation.from_matrix(rotations).as_quat()).reshape(-1, 4))


def _position(joint: JointDescription, positions: Mapping[str, float]) -> float:
    if joint.mimic is not None:
        leader, multiplier, offset = joint.mimic
        return multiplier * positions.get(leader, 0.0) + offset
    return positions.get(joint.name, 0.0)
