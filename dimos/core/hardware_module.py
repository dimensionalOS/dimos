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

"""A piece of hardware described by a USD, which other hardware can mount onto by name."""

from collections import defaultdict
from typing import TYPE_CHECKING, Any, TypeAlias

from pydantic import Field

from dimos.core.tf_module import TfModule, TfModuleConfig

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom

# A USD prim path, e.g. "/go2/base_link/front_camera_mount".
PrimPath: TypeAlias = str


class HardwareModuleConfig(TfModuleConfig):
    # The hardware's own description, only read when something asks for it.
    base_usd: Any = None
    # Common name -> the prim in base_usd that something else mounts onto, e.g. {"wrist": "/arm/link6"}.
    mounts: dict[str, PrimPath] = Field(default_factory=dict)
    # The mount this hardware sits on (set by HardwareModule.mount); None for the hardware everything mounts to.
    mount: str | None = Field(default=None, alias="_mount")


class HardwareModule(TfModule):
    """Hardware with a USD description; each namespace has one unmounted HardwareModule, the rest are mounted."""

    config: HardwareModuleConfig

    @classmethod
    def mount(cls, mount_name: str, **kwargs: Any) -> "Blueprint":
        """This hardware on the mount ``mount_name``, under the namespace of the same name."""
        return cls.blueprint(_mount=mount_name, **kwargs).namespace(mount_name)

    @staticmethod
    def blueprint_checks(atoms: "list[BlueprintAtom]") -> None:
        """Refuse a namespace with more than one unmounted HardwareModule."""
        unmounted: dict[str, list[str]] = defaultdict(list)
        for atom in atoms:
            if issubclass(atom.module, HardwareModule) and atom.kwargs.get("_mount") is None:
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
