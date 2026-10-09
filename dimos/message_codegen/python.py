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

"""Expose native rosbags classes without rewriting their declarations or methods."""

from __future__ import annotations

from rosbags.typesys import get_types_from_msg
from rosbags.typesys.codegen import generate_python_code

from .definitions import Message
from .registry import ABI as ABI


def generate(
    messages: tuple[Message, ...],
    module: str,
    *,
    version: str = "0.1.0",
    imports: dict[str, str] | None = None,
) -> dict[str, str]:
    imports = imports or {}
    definitions = {}
    for message in messages:
        definitions.update(get_types_from_msg(message.text, message.name))
    # Keep native rosbags declarations verbatim as typing-only source. Dependency
    # aliases reference their owning package; no second runtime class is created.
    declarations = generate_python_code(definitions)
    declarations += "\n" + "".join(
        f"from {owner}._types import {name.replace('/', '__')} as {name.replace('/', '__')}\n"
        for name, owner in sorted(imports.items())
    )
    packages = sorted({message.package for message in messages})
    result = {
        "_types.py": (
            "from pathlib import Path\n"
            "from dimos_message_build.registry import initialize\n"
            f"from {module}_schemas.provider import package_root\n"
            "store = initialize((Path(package_root()),))\n"
        ),
        "__init__.py": f"__dimos_version__ = {version!r}\n__dimos_abi__ = {ABI!r}\n"
        + "".join(f"from . import {package} as {package}\n" for package in packages),
        "py.typed": "",
        "_types.pyi": declarations,
    }
    for package in packages:
        result[f"{package}/__init__.py"] = "from . import msg as msg\n"
        result[f"{package}/msg/__init__.py"] = "from ..._types import store\n\n" + "".join(
            f"{message.short_name} = store.types[{message.name!r}]\n"
            for message in messages
            if message.package == package
        )
        result[f"{package}/msg/__init__.pyi"] = (
            "__all__ = "
            + repr([message.short_name for message in messages if message.package == package])
            + "\n"
            + "".join(
                f"from ..._types import {message.name.replace('/', '__')} as {message.short_name}\n"
                for message in messages
                if message.package == package
            )
        )
    return result
