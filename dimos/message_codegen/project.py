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
from importlib.metadata import version
from importlib.util import find_spec
import json
import keyword
import os
from pathlib import Path
import re
import sys
from tempfile import TemporaryDirectory
from typing import Any

from filelock import FileLock
import tomli_w

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos_cdrgen.definitions import Definitions, Message
from dimos_cdrgen.generate import generate
from dimos_cdrgen.ownership import ABI, Dependency

BUILD_VERSION = "0.1.0"
LANGUAGES = frozenset(("python", "cpp", "rust"))


@dataclass(frozen=True)
class Project:
    root: Path
    name: str
    version: str
    module: str
    source: Path
    python_module: str
    languages: tuple[str, ...]
    dependency_specs: dict[str, dict[str, str]]

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
        allowed = {"source", "module", "python-module", "languages", "dependencies"}
        unknown = config.keys() - allowed
        if unknown:
            raise ValueError(f"Unknown message configuration: {sorted(unknown)}")
        module = config.get("module", name.replace("-", "_"))
        if (
            not isinstance(module, str)
            or not re.fullmatch(r"[a-z][a-z0-9_]*", module)
            or keyword.iskeyword(module)
        ):
            raise ValueError("Invalid native module")
        python_module = config.get("python-module", module)
        if not isinstance(python_module, str) or not all(
            part.isidentifier() and not keyword.iskeyword(part) for part in python_module.split(".")
        ):
            raise ValueError("Invalid python-module")
        source_value = config.get("source", "interfaces")
        if not isinstance(source_value, str):
            raise TypeError("Message source must be a project-relative directory")
        source = (root / source_value).resolve()
        if not source.is_relative_to(root):
            raise ValueError("Message source must be inside the project")
        languages = config.get("languages", ["python"])
        if (
            not isinstance(languages, list)
            or not languages
            or any(not isinstance(value, str) or value not in LANGUAGES for value in languages)
        ):
            raise ValueError("languages must select python, cpp and/or rust")
        specs = config.get("dependencies", {})
        if not isinstance(specs, dict):
            raise TypeError("dependencies must map Python modules to exact versions")
        normalized = {}
        for dep_module, spec in specs.items():
            if not all(
                part.isidentifier() and not keyword.iskeyword(part)
                for part in dep_module.split(".")
            ):
                raise ValueError("Invalid message dependency")
            spec = {"version": spec} if isinstance(spec, str) else spec
            if not isinstance(spec, dict) or spec.keys() - {"version", "path", "distribution"}:
                raise ValueError(f"Invalid message dependency: {dep_module}")
            if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", str(spec.get("version", ""))):
                raise ValueError(f"Dependency {dep_module} requires an exact version")
            for field in ("path", "distribution"):
                if field in spec and not isinstance(spec[field], str):
                    raise ValueError(f"Dependency {field} must be a string: {dep_module}")
            normalized[dep_module] = spec
        return cls(
            root,
            name,
            version,
            module,
            source,
            python_module,
            tuple(dict.fromkeys(languages)),
            normalized,
        )

    @property
    def output(self) -> Path:
        return self.output_for(self.languages)

    def output_for(self, languages: tuple[str, ...]) -> Path:
        return self.root / "build/messages" / "-".join(sorted(set(languages)))

    def requirements(self) -> list[str]:
        # Built-in ownership is a platform contract. Other nonstandard package
        # names can be declared explicitly rather than inferred from imports.
        return [
            f"{spec.get('distribution', 'dimos' if module == 'dimos.messages' else module.replace('_', '-'))}=={spec['version']}"
            for module, spec in sorted(self.dependency_specs.items())
            if "path" not in spec
        ]

    def dependencies(self) -> tuple[Dependency, ...]:
        result: dict[str, Dependency] = {}
        aliases: dict[str, str] = {}
        pending = dict(self.dependency_specs)
        expected_modules: dict[str, str] = {}
        while pending:
            module = min(pending)
            spec = pending.pop(module)
            identity = aliases.get(module, module)
            if identity in result:
                existing_dep = result[identity]
                if existing_dep.version != spec["version"]:
                    raise ValueError(f"Conflicting package versions for {module}")
                if expected_modules.get(module, existing_dep.module) != existing_dep.module:
                    raise ValueError(f"Conflicting Python namespace: {module}")
                if "path" in spec and (self.root / spec["path"]).resolve() != existing_dep.root:
                    raise ValueError(f"Conflicting package identity for {module}")
                if spec.get("distribution", existing_dep.distribution) != existing_dep.distribution:
                    raise ValueError(f"Conflicting package distribution for {module}")
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
            if expected_modules.get(module, dep.module) != dep.module:
                raise ValueError(f"Conflicting Python namespace: {module}")
            if spec.get("distribution", dep.distribution) != dep.distribution:
                raise ValueError(f"Dependency distribution mismatch: {module}")
            if module not in {dep.module, dep.python_module} or dep.version != spec["version"]:
                raise ValueError(
                    f"Dependency identity/version mismatch: {module}=={spec['version']}"
                )
            if dep.module == self.module or dep.python_module == self.python_module:
                raise ValueError("A message package cannot depend on itself")
            if dep.module in result and result[dep.module] != dep:
                raise ValueError(f"Conflicting package identity for {dep.module}")
            for alias in (dep.module, dep.python_module or dep.module):
                if alias in aliases and aliases[alias] != dep.module:
                    raise ValueError(f"Conflicting Python namespace: {alias}")
            result[dep.module] = dep
            aliases[dep.module] = dep.module
            aliases[dep.python_module or dep.module] = dep.module
            metadata = json.loads((root / "message-package.json").read_text())
            for child, child_version in sorted(metadata.get("dependencies", {}).items()):
                python_child = metadata.get("python_dependencies", {}).get(child, child)
                expected = {"version": child_version}
                existing = pending.get(
                    python_child, self.dependency_specs.get(python_child, expected)
                )
                if existing["version"] != child_version:
                    raise ValueError(f"Conflicting package versions for {child}")
                previous_owner = expected_modules.setdefault(python_child, child)
                if previous_owner != child:
                    raise ValueError(f"Conflicting Python namespace: {python_child}")
                pending[python_child] = existing
        return tuple(result[module] for module in sorted(result))


def prepare(
    project: Project, languages: tuple[str, ...] | None = None, *, cache: Path | None = None
) -> Path:
    """Generate a complete owned package atomically; never invoke compilers or networks."""
    selected = tuple(sorted(set(languages or project.languages)))
    if not selected or any(language not in LANGUAGES for language in selected):
        raise ValueError("Select python, cpp and/or rust")
    dependencies = project.dependencies()
    paths = sorted(project.source.rglob("*.msg")) if project.source.is_dir() else []
    if not paths:
        raise ValueError(f"No messages found under {project.source}; expected package/msg/Type.msg")
    if any(not path.resolve().is_relative_to(project.root) for path in paths):
        raise ValueError("Message source files must be inside the project")
    names = [Message.from_file(path).name for path in paths]
    roots = [project.source] if paths else []
    definitions = Definitions(roots + [dep.root / "schemas" for dep in dependencies])
    resolved = definitions.resolve(names)
    imported = {name for dep in dependencies for name in dep.owned}
    unexpected = {message.name for message in resolved} - set(names) - imported
    if unexpected:
        raise ValueError(f"Missing dependency owners: {sorted(unexpected)}")
    if set(names) & imported:
        raise ValueError(
            f"Local definitions duplicate dependency ownership: {sorted(set(names) & imported)}"
        )
    toolkit_hash = sha256(Path(__file__).read_bytes())
    toolkit_hash.update(version("dimos-cdrgen").encode())
    inputs: dict[str, Any] = {
        "abi": ABI,
        "languages": selected,
        "toolkit": toolkit_hash.hexdigest(),
        "module": project.module,
        "version": project.version,
        "names": names,
        "python_module": project.python_module,
        "distribution": project.name,
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
    output = cache / "-".join(selected) if cache is not None else project.output_for(selected)
    # Each language selection owns a separate output. One lock serializes edits
    # of a given project, while source snapshots keep generation consistent.
    output.parent.mkdir(parents=True, exist_ok=True)
    with FileLock(str(output.parent / ".generation.lock")):
        state = output / "generation.json"
        if state.is_file():
            previous = json.loads(state.read_text())
            expected = previous.get("files", {})
            actual = {
                str(path.relative_to(output))
                for path in output.rglob("*")
                if path.is_file() and path != state
            }
            if (
                previous.get("digest") == digest
                and actual == expected.keys()
                and all(
                    (output / path).is_file()
                    and sha256((output / path).read_bytes()).hexdigest() == checksum
                    for path, checksum in expected.items()
                )
            ):
                return output
        with TemporaryDirectory(prefix=".generate-", dir=output.parent) as directory:
            staging = Path(directory) / "output"
            source = Path(directory) / "interfaces"
            for message in resolved:
                if message.name in names:
                    path = source / (message.name + ".msg")
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(message.text)
            generate(
                [source],
                staging,
                names,
                project.module,
                python_module=project.python_module,
                distribution=project.name,
                version=project.version,
                dependencies=dependencies,
                shared=True,
                languages=selected,
            )
            cargo = staging / "rust/Cargo.toml"
            if cargo.is_file():
                config = tomllib.loads(cargo.read_text())
                for dependency in dependencies:
                    name = dependency.module.replace("_", "-") + "-messages"
                    config["dependencies"][name]["path"] = os.path.relpath(
                        dependency.root / "rust", output / "rust"
                    )
                cargo.write_text(tomli_w.dumps(config))
            files = {
                path.relative_to(staging).as_posix(): sha256(path.read_bytes()).hexdigest()
                for path in sorted(staging.rglob("*"))
                if path.is_file()
            }
            (staging / "generation.json").write_text(
                json.dumps({"digest": digest, "files": files}, indent=2, sort_keys=True) + "\n"
            )
            backup = Path(directory) / "previous"
            if output.exists():
                output.rename(backup)
            try:
                staging.rename(output)
            except OSError:
                if backup.exists():
                    backup.rename(output)
                raise
    return output
