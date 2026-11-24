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

    autoconnect(Robot.blueprint(), Lidar.mount(Robot.mounts.lidar))

Each HardwareModule publishes the tf of its own USD; a mounted one also publishes the edge from
the mount frame it sits on to its own frames. Whatever needs the whole assembly (a simulator)
asks every module meeting ``HardwareSpec`` for ``get_usd()`` and ``mounted_on()`` and grafts them.
"""

from collections import defaultdict
from collections.abc import Iterator, Mapping
from functools import cached_property
from typing import TYPE_CHECKING, Any, ClassVar, Protocol

import numpy as np
from pydantic import ConfigDict

from dimos.core.core import rpc
from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.tf import TF
from dimos.robot.assets.model import JointDescription, UsdModel
from dimos.spec.utils import Spec

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom


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
    model_config = ConfigDict(arbitrary_types_allowed=True)

    # This hardware alone: a robot's USD has no sensors in it, they mount on with their own.
    # Required: hardware without a USD is not a HardwareModule.
    usd: UsdModel
    # The mount frame of the hardware this sits on (set by HardwareModule.mount); None for the hardware everything mounts to.
    mount: str | None = None


class HardwareSpec(Spec, Protocol):
    def get_usd(self) -> str: ...
    def mounted_on(self) -> str | None: ...


class HardwareModule(TfModule):
    """Hardware with its own USD; each namespace has one unmounted HardwareModule, the rest are mounted."""

    config: HardwareModuleConfig
    # Frames of this hardware's USD that other hardware can mount onto.
    mounts: ClassVar[Mounts] = Mounts()
    # Streams a mounted copy keeps unprefixed, so they still reach whatever is outside its mount's namespace.
    mount_shared: ClassVar[frozenset[str]] = frozenset()

    @classmethod
    def mount(cls, mount_frame: str, **kwargs: Any) -> "Blueprint":
        """This hardware on the mount frame ``mount_frame``, under the namespace of the same name."""
        return cls.blueprint(mount=mount_frame, **kwargs).namespace(
            mount_frame, shared=cls.mount_shared
        )

    @staticmethod
    def blueprint_checks(atoms: "list[BlueprintAtom]") -> None:
        """Refuse a namespace with more than one unmounted HardwareModule."""
        unmounted: dict[str, list[str]] = defaultdict(list)
        for atom in atoms:
            if issubclass(atom.module, HardwareModule) and atom.kwargs.get("mount") is None:
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
                        "HardwareModule (mount all but one with .mount(name)).",
                        *errors,
                    ]
                )
            )

    @rpc
    def get_usd(self) -> str:
        """This hardware's USD layer, its frames named as they are on tf and its assets as absolute paths."""
        usd = self.config.usd
        from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
        from dimos.robot.assets.usd_utils.frames import prefix_usd_frames

        root = str(usd.source_path.parent) if usd.source_path is not None else ""
        return prefix_usd_frames(resolve_usd_assets(usd.usd, root), self.namespace, root or None)

    @rpc
    def mounted_on(self) -> str | None:
        """The tf frame this hardware is mounted on, or None if nothing holds it."""
        return self.namespace if self.config.mount is not None else None

    def usd_frame(self, name: str) -> str:
        """The tf frame id of the frame ``name`` in this hardware's USD."""
        return f"{self.namespace}/{name}" if self.namespace else name

    @property
    def mounted_frame(self) -> str:
        """The frame the mount holds this hardware by; a mobile robot holds by its odometry frame instead."""
        return self.usd_frame(self.config.usd.root_link)

    def usd_transforms(
        self, joint_positions: Mapping[str, float] | None = None, ts: float | None = None
    ) -> list[Transform]:
        """Every edge of this hardware's own tf at ``joint_positions`` (missing joints sit at zero), plus its mount."""
        positions = joint_positions or {}
        transforms = [
            Transform.from_matrix(
                joint.parent_from_child(_position(joint, positions)),
                ts=ts,
                frame_id=self.usd_frame(joint.parent_link),
                child_frame_id=self.usd_frame(joint.child_link),
            )
            for joint in self._posable_joints
        ]
        mounted_on = self.mounted_on()
        if mounted_on is not None:
            transforms.append(
                Transform.from_matrix(
                    np.eye(4), ts=ts, frame_id=mounted_on, child_frame_id=self.mounted_frame
                )
            )
        return transforms

    def publish_usd_tf(
        self, joint_positions: Mapping[str, float] | None = None, ts: float | None = None
    ) -> None:
        self.tfbuffer.publish(*self.usd_transforms(joint_positions, ts))

    @cached_property
    def _posable_joints(self) -> tuple[JointDescription, ...]:
        # A floating joint has no single position: the robot owning it publishes its pose from odometry.
        return tuple(j for j in self.config.usd.joints if j.type != "floating")

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


def _position(joint: JointDescription, positions: Mapping[str, float]) -> float:
    if joint.mimic is not None:
        leader, multiplier, offset = joint.mimic
        return multiplier * positions.get(leader, 0.0) + offset
    return positions.get(joint.name, 0.0)
