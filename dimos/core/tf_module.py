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
from typing import TYPE_CHECKING, Any, TypeAlias, get_type_hints

from pydantic import Field, model_validator

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import BlueprintAtom

logger = setup_logger()

# A frame that belongs to one robot (base_link, lidar); a namespaced blueprint prefixes it.
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

    @property
    def tf_edge_frames(self) -> set[tuple[str, str]]:
        frames = {**self.tf_frames_global, **self.tf_frames_local}
        return {(frames[parent], frames[child]) for parent, child in self.tf_edges}


class TfModule(Module):
    """Its ``tfbuffer`` publishes only the edges in ``tf_edges``, unless ``skip_tf_checks``."""

    config: TfModuleConfig
    tf: IO[TFMessage]

    @classmethod
    def handle_blueprint_namespace(cls, kwargs: dict[str, Any], *, prefix: str) -> dict[str, Any]:
        """Also prefix the local frames, so namespaced copies don't share frames."""
        kwargs = super().handle_blueprint_namespace(kwargs, prefix=prefix)
        frames = kwargs.get("tf_frames_local", _config_default(cls, "tf_frames_local"))
        return {
            **kwargs,
            "tf_frames_local": {role: f"{prefix}/{frame}" for role, frame in frames.items()},
        }

    @staticmethod
    def blueprint_checks(atoms: "list[BlueprintAtom]") -> None:
        """Refuse a frame with two parents, or declared edges that form a cycle or several trees; warn on an edge with two owners. Modules with skip_tf_checks are left out."""
        publishers: dict[str, dict[str, list[str]]] = defaultdict(lambda: defaultdict(list))
        for atom in atoms:
            if not _is_checked(atom):
                continue
            frames = {
                **_atom_field(atom, "tf_frames_global"),
                **_atom_field(atom, "tf_frames_local"),
            }
            for parent_role, child_role in _atom_field(atom, "tf_edges"):
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
            message = f"declared tf edges form {len(trees)} separate trees, rooted at {roots}"
            # A module that isn't a TfModule may publish the edges that join them, so only warn then.
            undeclared_publishers = sorted(
                atom.name
                for atom in atoms
                if not _is_checked(atom)
                and any(stream.name == "tf" and stream.direction != "in" for stream in atom.streams)
            )
            if undeclared_publishers:
                logger.warning(
                    f"{message}; {', '.join(undeclared_publishers)} may join them (not checked TfModules)"
                )
            else:
                errors.append(f"    - {message}")

        if errors:
            raise ValueError(
                "\n".join(["Blueprint cannot start because its tf tree is broken.", *errors])
            )

    def _create_tfbuffer(self, port: Any) -> TF:
        return TF(
            port,
            edges=None if self.config.skip_tf_checks else self.config.tf_edge_frames,
            module_name=self.config.instance_name or type(self).__name__,
        )


def _config_default(module: type[TfModule], name: str) -> Any:
    return (
        get_type_hints(module)["config"].model_fields[name].get_default(call_default_factory=True)
    )


def _is_checked(atom: "BlueprintAtom") -> bool:
    return issubclass(atom.module, TfModule) and not _atom_field(atom, "skip_tf_checks")


def _atom_field(atom: "BlueprintAtom", name: str) -> Any:
    if name in atom.kwargs:
        return atom.kwargs[name]
    return _config_default(atom.module, name)  # type: ignore[arg-type]
