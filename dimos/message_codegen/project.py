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

"""Project configuration and deterministic generation shared by every build frontend."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from importlib.util import find_spec
import json
from pathlib import Path
import re
import shutil
import sys
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from .definitions import Definitions, parse_message
from .generate import generate
from .ownership import ABI, Dependency

BUILD_VERSION = "0.1.0"
LANGUAGES = frozenset(("python", "cpp", "rust"))


@dataclass(frozen=True)
class Project:
    root: Path
    name: str
    version: str
    module: str
    source: Path
    languages: tuple[str, ...]
    dependency_specs: dict[str, dict[str, str]]
    internal: bool = False

    @classmethod
    def load(cls, root: Path) -> Project:
        root = root.resolve()
        data = tomllib.loads((root / "pyproject.toml").read_text())
        project = data.get("project", {})
        config = data.get("tool", {}).get("dimos", {}).get("messages", {})
        name, version = project.get("name"), project.get("version")
        if not isinstance(name, str) or not re.fullmatch(r"[a-z][a-z0-9_-]*", name):
            raise ValueError("[project].name must be a lowercase package name")
        if not isinstance(version, str) or not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
            raise ValueError("[project].version must be an explicit major.minor.patch version")
        allowed = {"source-dir", "python-module", "languages", "dependencies", "internal"}
        unknown = config.keys() - allowed
        if unknown:
            raise ValueError(f"Unknown message configuration: {sorted(unknown)}")
        module = config.get("python-module", name.replace("-", "_"))
        if not isinstance(module, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", module):
            raise ValueError("Invalid python-module")
        if module != name.replace("-", "_"):
            raise ValueError("python-module must match the normalized project name")
        source = (root / config.get("source-dir", "interfaces")).resolve()
        if not source.is_relative_to(root):
            raise ValueError("Message source-dir must be inside the project")
        languages = config.get("languages", ["python"])
        if (
            not isinstance(languages, list)
            or not languages
            or any(value not in LANGUAGES for value in languages)
        ):
            raise ValueError("languages must select python, cpp and/or rust")
        internal = config.get("internal", False)
        if not isinstance(internal, bool):
            raise ValueError("internal must be a boolean")
        specs = config.get(
            "dependencies", {} if internal else {"dimos_generated": {"version": BUILD_VERSION}}
        )
        if not isinstance(specs, dict):
            raise ValueError("dependencies must map Python modules to exact versions")
        for dep_module, spec in specs.items():
            if not re.fullmatch(r"[a-z][a-z0-9_]*", dep_module) or not isinstance(spec, dict):
                raise ValueError("Invalid message dependency")
            if spec.keys() - {"version", "path"} or not re.fullmatch(
                r"[0-9]+\.[0-9]+\.[0-9]+", str(spec.get("version", ""))
            ):
                raise ValueError(f"Dependency {dep_module} requires an exact version")
            if "path" in spec and not isinstance(spec["path"], str):
                raise ValueError(f"Dependency path must be a string: {dep_module}")
        return cls(
            root, name, version, module, source, tuple(dict.fromkeys(languages)), specs, internal
        )

    @property
    def output(self) -> Path:
        return self.root / "build" / "dimos"

    def requirements(self) -> list[str]:
        return [
            f"{module.replace('_', '-')}=={spec['version']}"
            for module, spec in sorted(self.dependency_specs.items())
            if "path" not in spec
        ]

    def dependencies(self) -> tuple[Dependency, ...]:
        result: dict[str, Dependency] = {}
        pending = dict(self.dependency_specs)
        while pending:
            module = sorted(pending)[0]
            spec = pending.pop(module)
            if module in result:
                if result[module].version != spec["version"]:
                    raise ValueError(f"Conflicting package versions for {module}")
                continue
            if "path" in spec:
                root = (self.root / spec["path"]).resolve()
            else:
                found = find_spec(module + "_schemas")
                if found is None or found.origin is None:
                    raise ValueError(
                        f"Install message dependency {module.replace('_', '-')}=={spec['version']} before building (or supply it to pip's build environment)"
                    )
                root = Path(found.origin).parent / "package"
            dep = Dependency.load(root)
            if dep.module != module or dep.version != spec["version"]:
                raise ValueError(
                    f"Dependency identity/version mismatch: {module}=={spec['version']}"
                )
            if module == self.module:
                raise ValueError("A message package cannot depend on itself")
            result[module] = dep
            metadata = json.loads((root / "message-package.json").read_text())
            for child, version in sorted(metadata.get("dependencies", {}).items()):
                expected = {"version": version}
                existing = pending.get(child, self.dependency_specs.get(child, expected))
                if existing["version"] != version:
                    raise ValueError(f"Conflicting package versions for {child}")
                pending[child] = existing
        return tuple(result[module] for module in sorted(result))


def prepare(project: Project) -> Path:
    """Generate a complete owned package atomically; never invoke compilers or networks."""
    dependencies = project.dependencies()
    paths = sorted(project.source.rglob("*.msg")) if project.source.is_dir() else []
    if not paths and not project.internal:
        raise ValueError(f"No messages found under {project.source}; expected package/msg/Type.msg")
    names = [parse_message(path).name for path in paths]
    roots = [project.source] if paths else []
    definitions = Definitions(roots + [dep.root / "schemas" for dep in dependencies])
    resolved = definitions.resolve(names if not project.internal else None)
    imported = {name for dep in dependencies for name in dep.owned}
    unexpected = {message.name for message in resolved} - set(names) - imported
    if unexpected and not project.internal:
        raise ValueError(f"Missing dependency owners: {sorted(unexpected)}")
    if set(names) & imported:
        raise ValueError(
            f"Local definitions duplicate dependency ownership: {sorted(set(names) & imported)}"
        )
    toolkit = Path(__file__).parent
    toolkit_hash = sha256()
    toolkit_paths = (
        list(toolkit.glob("*.py"))
        + list((toolkit / "_vendor").glob("*.py"))
        + list((toolkit / "templates").glob("*"))
    )
    for path in sorted(toolkit_paths):
        if (
            path.is_file()
            and (path.suffix in {".py", ".hpp", ".rs"})
            and "__pycache__" not in path.parts
        ):
            toolkit_hash.update(path.relative_to(toolkit).as_posix().encode())
            toolkit_hash.update(path.read_bytes())
    inputs: dict[str, Any] = {
        "abi": ABI,
        "toolkit": toolkit_hash.hexdigest(),
        "module": project.module,
        "version": project.version,
        "names": names,
        "internal": project.internal,
        "schemas": {message.name: definitions.schema(message.name) for message in resolved},
        "dependencies": [
            {
                "module": dep.module,
                "version": dep.version,
                "schemas": dep.schemas,
                "root": str(dep.root),
            }
            for dep in dependencies
        ],
    }
    digest = sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()
    output = project.output
    state = output / "generation.json"
    if state.is_file():
        previous = json.loads(state.read_text())
        if previous.get("digest") == digest and all(
            (output / path).is_file()
            and sha256((output / path).read_bytes()).hexdigest() == expected
            for path, expected in previous.get("files", {}).items()
        ):
            return output
    staging = output.with_name("dimos-generating")
    if staging.exists():
        shutil.rmtree(staging)
    generate(
        roots,
        staging,
        names if not project.internal else None,
        project.module,
        version=project.version,
        dependencies=dependencies,
        shared=True,
    )
    (staging / "toolchain.cmake").write_text(
        'list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}"'
        + "".join(" " + json.dumps(str(dep.root)) for dep in dependencies)
        + ")\n"
    )
    files = {
        path.relative_to(staging).as_posix(): sha256(path.read_bytes()).hexdigest()
        for path in sorted(staging.rglob("*"))
        if path.is_file()
    }
    (staging / "generation.json").write_text(
        json.dumps({"digest": digest, "files": files}, indent=2, sort_keys=True) + "\n"
    )
    if output.exists():
        shutil.rmtree(output)
    staging.rename(output)
    return output
