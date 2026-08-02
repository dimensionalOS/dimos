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

"""The bake registry: rust crates that declare a native module in their manifest.

A crate opts in with a `[package.metadata.dimos.module.<id>]` table. Reading it
is pure TOML, so the whole graph can be drawn and checked before cargo runs.
`#[module(name = "<id>")]` on the rust struct fails the build if the two drift.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any

import tomllib

from dimos.cli.bake import BakeError

# Directories that never hold a source manifest, and that are expensive to walk.
_PRUNE = {
    ".direnv",
    ".git",
    ".venv",
    "__pycache__",
    "node_modules",
    "result",
    "target",
}

# Where in the repo module crates live. Everything else is out of scope.
_SEARCH_ROOTS = ("dimos", "native")


def repo_root() -> Path:
    """The checkout root, i.e. the directory holding the `dimos` package."""
    return Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class ModuleInfo:
    """One registry entry: everything bake needs about a module before compiling."""

    id: str
    crate_dir: Path
    crate_name: str
    # Full rust path of the `#[derive(Module)]` struct, for the generated host.
    rust_path: str
    # `pkg.mod:Class` of the python NativeModule wrapper, for `--emit-config`.
    python_ref: str
    threads: int
    nice: int | None
    inputs: Mapping[str, str]
    outputs: Mapping[str, str]

    @property
    def ports(self) -> Mapping[str, str]:
        return {**self.inputs, **self.outputs}


def normalize_id(name: str) -> str:
    """Module ids are underscore-spelled; `--` and `_` are interchangeable on the CLI."""
    return name.replace("-", "_")


def _iter_manifests(root: Path) -> Iterator[Path]:
    for search in _SEARCH_ROOTS:
        for dirpath, dirnames, filenames in os.walk(root / search):
            dirnames[:] = [d for d in dirnames if d not in _PRUNE]
            if "Cargo.toml" in filenames:
                yield Path(dirpath) / "Cargo.toml"


def _str_table(entry: Mapping[str, Any], key: str, where: str) -> dict[str, str]:
    table = entry.get(key, {})
    if not isinstance(table, dict):
        raise BakeError(f'{where}: `{key}` must be a table of port = "pkg.MsgType"')
    for port, msg in table.items():
        if not isinstance(msg, str):
            raise BakeError(f"{where}: `{key}.{port}` must be a message-type string")
    return dict(table)


def parse_manifest(path: Path) -> list[ModuleInfo]:
    """Registry entries declared by one Cargo.toml. Empty for crates that opt out."""
    manifest = tomllib.loads(path.read_text())
    package = manifest.get("package", {})
    modules = package.get("metadata", {}).get("dimos", {}).get("module", {})
    if not modules:
        return []

    crate_name = package.get("name")
    if not isinstance(crate_name, str):
        raise BakeError(f"{path}: a module crate needs a [package] name")

    found: list[ModuleInfo] = []
    for raw_id, entry in modules.items():
        where = f"{path}: [package.metadata.dimos.module.{raw_id}]"
        if not isinstance(entry, dict):
            raise BakeError(f"{where} must be a table")
        for key in ("path", "python"):
            if not isinstance(entry.get(key), str):
                raise BakeError(f"{where}: missing required string key `{key}`")
        threads = entry.get("threads", 1)
        if not isinstance(threads, int) or threads < 1:
            raise BakeError(f"{where}: `threads` must be a positive integer")
        nice = entry.get("nice")
        if nice is not None and not isinstance(nice, int):
            raise BakeError(f"{where}: `nice` must be an integer")
        found.append(
            ModuleInfo(
                id=normalize_id(raw_id),
                crate_dir=path.parent,
                crate_name=crate_name,
                rust_path=entry["path"],
                python_ref=entry["python"],
                threads=threads,
                nice=nice,
                inputs=_str_table(entry, "inputs", where),
                outputs=_str_table(entry, "outputs", where),
            )
        )
    return found


def discover_modules(root: Path | None = None) -> dict[str, ModuleInfo]:
    """Every registered module in the checkout, keyed by id."""
    root = root or repo_root()
    registry: dict[str, ModuleInfo] = {}
    for manifest in sorted(_iter_manifests(root)):
        for info in parse_manifest(manifest):
            if info.id in registry:
                raise BakeError(
                    f"module id `{info.id}` is declared twice: "
                    f"{registry[info.id].crate_dir} and {info.crate_dir}"
                )
            registry[info.id] = info
    return registry


def select_modules(registry: Mapping[str, ModuleInfo], names: Sequence[str]) -> list[ModuleInfo]:
    """Resolve CLI module names against the registry, preserving the given order."""
    if not names:
        raise BakeError("no modules given; `dimos bake --list` shows what is registered")
    selected: list[ModuleInfo] = []
    seen: set[str] = set()
    for name in names:
        module_id = normalize_id(name)
        if module_id in seen:
            raise BakeError(
                f"module `{module_id}` is listed twice; one instance per host for now "
                "(per-instance namespacing is not implemented)"
            )
        info = registry.get(module_id)
        if info is None:
            known = ", ".join(sorted(registry)) or "<none>"
            raise BakeError(f"unknown module `{module_id}`; registered modules: {known}")
        seen.add(module_id)
        selected.append(info)
    return selected


def render_registry(registry: Mapping[str, ModuleInfo]) -> str:
    """`dimos bake --list` output."""
    if not registry:
        return "No registered modules found."
    lines = ["Registered native modules:"]
    for module_id, info in sorted(registry.items()):
        rel = info.crate_dir.relative_to(repo_root())
        lines.append(f"  {module_id}  ({info.crate_name} at {rel}, threads={info.threads})")
        for port, msg in sorted(info.inputs.items()):
            lines.append(f"      in   {port}: {msg}")
        for port, msg in sorted(info.outputs.items()):
            lines.append(f"      out  {port}: {msg}")
    return "\n".join(lines)
