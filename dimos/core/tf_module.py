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

from collections import defaultdict, deque
from functools import cached_property
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, TypeAlias

from pydantic import Field, model_validator

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF
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


class TfModule(Module):
    """Its ``tf`` port and ``tfbuffer`` publish only the edges in ``tf_edges``, unless ``skip_tf_checks``."""

    config: TfModuleConfig
    tf: IO[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if self.config.skip_tf_checks:
            return
        # The port stays the plain IO Module bound; only its publish is wrapped with the edge check.
        publish = self.tf.publish

        def checked_publish(msg: TFMessage) -> None:
            for t in msg.transforms:
                if (t.frame_id, t.child_frame_id) not in self.tf_edge_ids:
                    raise ValueError(
                        f"{self.config.instance_name or type(self).__name__} publishes undeclared tf edge ({t.frame_id!r}, {t.child_frame_id!r}); add it to tf_edges"
                    )
            publish(msg)

        self.tf.publish = checked_publish  # type: ignore[method-assign]

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
        """Refuse declared edges that close a loop, so there is one path between any two frames; warn on an edge with two owners or several trees. Modules with skip_tf_checks are left out.

        A frame may have several parents: lookups search the tree both ways, so only a loop makes a lookup ambiguous.
        """
        owners: dict[tuple[str, str], list[str]] = defaultdict(list)
        for atom in atoms:
            if not _is_checked(atom):
                continue
            frames = _frame_ids(
                atom.kwargs["tf_frames_local"],
                atom.kwargs["tf_frames_global"],
                atom.name.rpartition("/")[0],
            )
            for parent_role, child_role in atom.kwargs["tf_edges"]:
                owners[frames[parent_role], frames[child_role]].append(atom.name)

        if not owners:
            return

        errors = []
        for (parent, child), names in sorted(owners.items()):
            if len(names) > 1:
                logger.warning(
                    f"tf edge '{parent}' -> '{child}' is declared by several modules "
                    f"({', '.join(names)}); only one module should own an edge"
                )

        # Union-find over the declared edges, undirected: an edge between two frames already joined closes a loop.
        component: dict[str, str] = {}
        neighbours: dict[str, set[str]] = defaultdict(set)

        def root_of(frame: str) -> str:
            while component.setdefault(frame, frame) != frame:
                frame = component[frame]
            return frame

        for parent, child in sorted(owners):
            if root_of(parent) == root_of(child):
                path = _path_between(neighbours, child, parent)
                errors.append(
                    f"    - frames {sorted(path)} form a cycle: '{parent}' -> '{child}' closes "
                    f"the loop {' -> '.join([*path, parent])}"
                )
                continue
            component[root_of(child)] = root_of(parent)
            neighbours[parent].add(child)
            neighbours[child].add(parent)
        trees: dict[str, set[str]] = defaultdict(set)
        for frame in list(component):
            trees[root_of(frame)].add(frame)

        if len(trees) > 1:
            children = {child for _, child in owners}
            roots = sorted(sorted(tree - children) or sorted(tree)[:1] for tree in trees.values())
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
            module_name=self.config.instance_name or type(self).__name__,
        )


def _frame_ids(local: dict[str, str], shared: dict[str, str], prefix: str) -> dict[str, str]:
    if prefix:
        local = {role: f"{prefix}/{frame}" for role, frame in local.items()}
    return {**shared, **local}


def _path_between(neighbours: dict[str, set[str]], start: str, end: str) -> list[str]:
    """The frames from ``start`` to ``end`` along edges already joined (a tree, so the one path)."""
    came_from: dict[str, str | None] = {start: None}
    queue = deque([start])
    while queue:
        frame = queue.popleft()
        for neighbour in neighbours[frame]:
            if neighbour not in came_from:
                came_from[neighbour] = frame
                queue.append(neighbour)
    path: list[str] = []
    step: str | None = end
    while step is not None:
        path.append(step)
        step = came_from[step]
    return path


def _is_checked(atom: "BlueprintAtom") -> bool:
    return issubclass(atom.module, TfModule) and not atom.kwargs["skip_tf_checks"]
