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

"""A module whose tf frames are declared in its config, so a blueprint can namespace and check them."""

from collections import defaultdict
from functools import cached_property
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, TypeAlias

from pydantic import Field, model_validator

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF, refuse_undeclared_edges
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import BlueprintAtom

logger = setup_logger()

# A frame that belongs to one robot (base_link, lidar); it goes under the module's namespace.
TransformFrameLocal: TypeAlias = str
# A frame shared by everything in the blueprint (world, map); never prefixed.
TransformFrameGlobal: TypeAlias = str


class TfModuleConfig(ModuleConfig):
    # Role -> frame id, e.g. {"body": "base_link"}. Code reads frames from here, never from literals.
    tf_frames_local: dict[str, TransformFrameLocal] = Field(default_factory=dict)
    tf_frames_global: dict[str, TransformFrameGlobal] = Field(default_factory=dict)
    # (parent role, child role) for every edge this module publishes; its tfbuffer refuses any other.
    tf_edges: list[tuple[str, str]] = Field(default_factory=list)
    # Opt this module out of tf checks (its tfbuffer publishes any edge).
    skip_tf_checks: bool = False

    @model_validator(mode="after")
    def _check_roles(self) -> "TfModuleConfig":
        shared = sorted(self.tf_frames_local.keys() & self.tf_frames_global.keys())
        if shared:
            raise ValueError(f"tf roles {shared} are in both tf_frames_local and tf_frames_global")
        roles = self.tf_frames_local.keys() | self.tf_frames_global.keys()
        unknown = sorted({role for edge in self.tf_edges for role in edge} - roles)
        if unknown:
            raise ValueError(
                f"tf_edges use undeclared tf roles {unknown} (declared: {sorted(roles)})"
            )
        return self


class TfIO(IO[TFMessage]):
    """A TfModule's tf port: ``publish`` refuses undeclared edges, ``get`` refuses undeclared frames."""

    owner: "TfModule"

    def publish(self, msg: TFMessage) -> None:
        if not self.owner.config.skip_tf_checks:
            refuse_undeclared_edges(
                self.owner.tf_edge_ids, self.owner.tf_owner_name, msg.transforms
            )
        super().publish(msg)

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
        warn: bool = True,
    ) -> Transform | None:
        """Look ``parent_frame -> child_frame`` up in the module's tfbuffer; both must be declared frames."""
        if not self.owner.config.skip_tf_checks:
            declared = set(self.owner.tf_frames.values())
            for frame in (parent_frame, child_frame):
                if frame not in declared:
                    raise ValueError(
                        f"{self.owner.tf_owner_name} looks up undeclared tf frame {frame!r} "
                        f"(declared: {sorted(declared)}); add it to tf_frames_local or tf_frames_global"
                    )
        return self.owner.tfbuffer.get(
            parent_frame,
            child_frame,
            time_point,
            time_tolerance,
            forward_tolerance=forward_tolerance,
            warn=warn,
        )


class TfModule(Module):
    """Its ``tf`` port and ``tfbuffer`` publish only the edges in ``tf_edges``, unless ``skip_tf_checks``."""

    config: TfModuleConfig
    if TYPE_CHECKING:
        tf: TfIO
    else:
        # Module binds ports from the runtime annotation, so it stays a plain IO.
        tf: IO[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Same name and type as the port Module bound, so wiring and topics are unchanged.
        self.tf = TfIO(TFMessage, "tf", self)

    @property
    def tf_owner_name(self) -> str:
        """The name tf errors give for this module."""
        return self.config.instance_name or type(self).__name__

    @cached_property
    def tf_edge_ids(self) -> frozenset[tuple[str, str]]:
        """The declared edges as (parent frame id, child frame id)."""
        frames = self.tf_frames
        return frozenset((frames[parent], frames[child]) for parent, child in self.config.tf_edges)

    @cached_property
    def tf_frames(self) -> dict[str, str]:
        """Every role -> frame id, local (under the namespace) and global together."""
        return _frame_ids(self.config.tf_frames_local, self.config.tf_frames_global, self.namespace)

    @cached_property
    def frame(self) -> SimpleNamespace:
        """Frame ids by role, local and global together: ``self.frame.body``."""
        return SimpleNamespace(**self.tf_frames)

    @staticmethod
    def blueprint_checks(atoms: "list[BlueprintAtom]") -> None:
        """Refuse a frame with two parents or declared edges that form a cycle; warn on an edge with two owners or several trees. Modules with skip_tf_checks are left out."""
        publishers: dict[str, dict[str, list[str]]] = defaultdict(lambda: defaultdict(list))
        for atom in atoms:
            if not _is_checked(atom):
                continue
            frames = _frame_ids(
                atom.kwargs["tf_frames_local"],
                atom.kwargs["tf_frames_global"],
                atom.name.rpartition("/")[0],
            )
            for parent_role, child_role in atom.kwargs["tf_edges"]:
                publishers[frames[child_role]][frames[parent_role]].append(atom.name)
        if not publishers:
            return

        errors = []
        for child, parents in sorted(publishers.items()):
            if len(parents) > 1:
                listed = ", ".join(
                    f"{parent} (from {', '.join(names)})"
                    for parent, names in sorted(parents.items())
                )
                errors.append(f"    - '{child}' has several parents: {listed}")
            for parent, names in parents.items():
                if len(names) > 1:
                    logger.warning(
                        f"tf edge '{parent}' -> '{child}' is declared by several modules "
                        f"({', '.join(names)}); only one module should own an edge"
                    )

        # Union-find over the declared edges: one component per tree.
        component: dict[str, str] = {}

        def root_of(frame: str) -> str:
            while component.setdefault(frame, frame) != frame:
                frame = component[frame]
            return frame

        for child, parents in publishers.items():
            for parent in parents:
                component[root_of(child)] = root_of(parent)
        trees: dict[str, set[str]] = defaultdict(set)
        for frame in list(component):
            trees[root_of(frame)].add(frame)
        for tree in trees.values():
            if not tree - publishers.keys():
                errors.append(f"    - frames {sorted(tree)} form a cycle (none of them is a root)")

        if len(trees) > 1:
            roots = sorted(
                sorted(tree - publishers.keys()) or sorted(tree)[:1] for tree in trees.values()
            )
            # Only a warning: separate robots' odometry stays disconnected until something localizes them.
            undeclared_publishers = sorted(
                atom.name
                for atom in atoms
                if not _is_checked(atom)
                and any(stream.name == "tf" and stream.direction != "in" for stream in atom.streams)
            )
            joiners = (
                f"; {', '.join(undeclared_publishers)} may join them (not checked TfModules)"
                if undeclared_publishers
                else ""
            )
            logger.warning(
                f"declared tf edges form {len(trees)} separate trees, rooted at {roots}{joiners}"
            )

        if errors:
            raise ValueError(
                "\n".join(["Blueprint cannot start because its tf tree is broken.", *errors])
            )

    def _create_tfbuffer(self, port: Any) -> TF:
        return TF(
            port,
            edges=None if self.config.skip_tf_checks else self.tf_edge_ids,
            module_name=self.tf_owner_name,
        )


def _frame_ids(local: dict[str, str], shared: dict[str, str], prefix: str) -> dict[str, str]:
    if prefix:
        local = {role: f"{prefix}/{frame}" for role, frame in local.items()}
    return {**shared, **local}


def _is_checked(atom: "BlueprintAtom") -> bool:
    return issubclass(atom.module, TfModule) and not atom.kwargs["skip_tf_checks"]
