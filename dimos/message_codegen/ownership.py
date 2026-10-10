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

"""Explicit ownership contracts for separately compiled message packages."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import json
from pathlib import Path
import re

from .definitions import Definitions, Message
from .registry import ABI as ABI


@dataclass(frozen=True)
class Dependency:
    module: str
    version: str
    root: Path
    owned: tuple[str, ...]
    schemas: dict[str, str]
    codec_owner: str = ""

    @property
    def crate(self) -> str:
        return self.module + "_messages"

    @classmethod
    def load(cls, root: Path) -> Dependency:
        data = json.loads((root / "message-package.json").read_text())
        if data.get("format") != 1 or data.get("abi") != ABI or not data.get("shared"):
            raise ValueError(f"Incompatible message package ABI: {root}")
        module, version = data["module"], data["version"]
        if not re.fullmatch(r"[a-z][a-z0-9_]*", module):
            raise ValueError(f"Invalid dependency module: {module}")
        if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
            raise ValueError(f"Invalid dependency version: {version}")
        return cls(
            module,
            version,
            root.resolve(),
            tuple(data["owned"]),
            data["schemas"],
            data.get("codec_owner", module),
        )


def schema_hash(schema: str) -> str:
    return sha256(schema.encode()).hexdigest()


def resolve_owners(
    messages: tuple[Message, ...], definitions: Definitions, dependencies: tuple[Dependency, ...]
) -> dict[str, Dependency]:
    owners: dict[str, Dependency] = {}
    modules: dict[str, str] = {}
    for dependency in dependencies:
        previous = modules.setdefault(dependency.module, dependency.version)
        if previous != dependency.version:
            raise ValueError(f"Conflicting package versions for {dependency.module}")
        for name in dependency.owned:
            if name in owners:
                raise ValueError(f"Multiple owners for message {name}")
            owners[name] = dependency
    for dependency in dependencies:
        missing = set(dependency.schemas) - owners.keys()
        if missing:
            raise ValueError(
                f"Missing transitive message owners for {dependency.module}: {sorted(missing)}"
            )
    for message in messages:
        owner = owners.get(message.name)
        if owner is not None and owner.schemas.get(message.name) != schema_hash(
            definitions.schema(message.name)
        ):
            raise ValueError(f"Dependency schema mismatch: {message.name} from {owner.module}")
    return {message.name: owners[message.name] for message in messages if message.name in owners}
