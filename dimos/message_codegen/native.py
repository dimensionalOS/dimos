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

"""Internal CMake/Cargo hooks over the same external-message project plan."""

from __future__ import annotations

import argparse
from importlib.metadata import distribution
import json
import os
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
from typing import Any

from dimos_cdrgen.native_build import prepare_cpp, write_cmake_toolchain
import tomli_w

from dimos.message_codegen.project import Project, prepare

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def sdk_root() -> Path:
    package = Path(__file__).resolve().parents[1]
    for native in (package / "_native", package.parent / "native"):
        if (native / "cpp/CMakeLists.txt").is_file():
            return native
    raise ValueError("Install a matching dimOS wheel containing the native SDK sources")


def cmake_prefix() -> Path:
    return Path(__file__).with_name("cmake")


def prepare_cmake(project: Project) -> dict[str, Any]:
    if "cpp" not in project.languages:
        raise ValueError("Enable cpp in tool.dimos.messages.languages before configuring CMake")
    if shutil.which("cmake") is None:
        raise ValueError("CMake is required for native message builds")
    output = prepare(project, ("cpp",))
    prefix = prepare_cpp(
        output,
        cache=Path(
            os.environ.get("DIMOS_MESSAGE_NATIVE_CACHE", project.root / "build/native-cache")
        ),
        offline=os.environ.get("DIMOS_OFFLINE") == "1",
    )
    toolchain_path = project.root / "build/cmake-messages/toolchain.cmake"
    toolchain_path.parent.mkdir(parents=True, exist_ok=True)
    toolchain = write_cmake_toolchain(prefix, toolchain_path)
    dependencies = project.dependencies()
    schemas = json.loads((output / "schemas.json").read_text())
    ros_packages = {name.split("/")[0] for name in schemas}
    ros_packages.update(name.split("/")[0] for dep in dependencies for name in dep.schemas)
    return {
        "module": project.module,
        "prefix": str(prefix),
        "toolchain": str(toolchain),
        "modules": sorted({project.module, *[dep.module for dep in dependencies], *ros_packages}),
    }


def prepare_rust(project: Project, manifest: Path, *, cache: Path | None = None) -> dict[str, Any]:
    if "rust" not in project.languages:
        raise ValueError("Enable rust in tool.dimos.messages.languages before running Cargo")
    cargo = tomllib.loads(manifest.read_text())
    expected_name = project.module.replace("_", "-") + "-messages"
    package = cargo.get("package", {})
    if package.get("name") != expected_name or package.get("version") != project.version:
        raise ValueError(f"Cargo package must be {expected_name} version {project.version}")
    dependencies = project.dependencies()
    for dependency in dependencies:
        name = dependency.module.replace("_", "-") + "-messages"
        spec = cargo.get("dependencies", {}).get(name)
        version = spec.get("version") if isinstance(spec, dict) else spec
        if version != "=" + dependency.version:
            raise ValueError(f"Cargo must declare {name} with exact version ={dependency.version}")
    output = prepare(project, ("rust",), cache=cache)
    toolkit = distribution("dimos-cdrgen").locate_file("dimos_cdrgen")
    return {
        "interfaces": [str(path) for path in sorted((output / "rust/interfaces").rglob("*.msg"))],
        "library": str(output / "rust/src/lib.rs"),
        "schemas": str(output / "rust/schemas.json"),
        "generation": str(output / "generation.json"),
        "inputs": [
            str(project.root / "pyproject.toml"),
            str(manifest),
            str(project.source),
            str(Path(__file__).resolve()),
            str(Path(__file__).with_name("project.py")),
            str(toolkit),
            *[str(dependency.root) for dependency in dependencies],
        ],
    }


def configure_cargo(project: Project) -> Path:
    """Export only installed SDK/dependency source locations; generate no types."""
    patches: dict[str, dict[str, str]] = {
        "dimos-message-build": {"path": str(Path(__file__).with_name("rust-build").resolve())}
    }
    native_sdk = sdk_root()
    for crate in ("dimos-module", "dimos-module-macros"):
        patches[crate] = {"path": str(native_sdk / "rust" / crate)}
    for dependency in project.dependencies():
        native = dependency.root / "rust"
        source = native / "interfaces"
        if not (native / "Cargo.toml").is_file():
            native = dependency.root / "native-project"
            if not (native / "Cargo.toml").is_file():
                raise ValueError(
                    f"Install the matching Rust source artifact for {dependency.module}"
                )
            source = Project.load(native).source
        data = tomllib.loads((native / "Cargo.toml").read_text())
        name = dependency.module.replace("_", "-") + "-messages"
        if data["package"]["name"] != name or data["package"]["version"] != dependency.version:
            raise ValueError(f"Rust dependency identity/version mismatch: {name}")
        for message in dependency.owned:
            artifact = source / (message + ".msg")
            original = dependency.root / "schemas" / (message + ".msg")
            if not artifact.is_file() or artifact.read_bytes() != original.read_bytes():
                raise ValueError(f"Rust dependency schema mismatch: {message}")
        patches[name] = {"path": str(native.resolve())}
    path = project.root / ".cargo/config.toml"
    if path.is_symlink() or not path.resolve().is_relative_to(project.root):
        raise ValueError("Cargo configuration must be inside the project")
    data = tomllib.loads(path.read_text()) if path.is_file() else {}
    existing = data.setdefault("patch", {}).setdefault("crates-io", {})
    for name, patch in patches.items():
        if name in existing and existing[name] != patch:
            raise ValueError(
                f"Existing Cargo patch for {name} differs; review it before changing SDK environments"
            )
    existing.update(patches)
    path.parent.mkdir(exist_ok=True)
    with TemporaryDirectory(prefix=".sdk-config-", dir=path.parent) as directory:
        stage = Path(directory) / "config.toml"
        stage.write_text(tomli_w.dumps(data))
        stage.replace(path)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("cmake-prefix")
    commands.add_parser("sdk-root")
    cargo = commands.add_parser("configure-cargo")
    cargo.add_argument("project", nargs="?", type=Path, default=Path.cwd())
    for name in ("cpp", "rust"):
        command = commands.add_parser(name)
        command.add_argument("project", type=Path)
        command.add_argument("output", type=Path)
        if name == "rust":
            command.add_argument("manifest", type=Path)
    args = parser.parse_args()
    if args.command == "cmake-prefix":
        print(cmake_prefix())
        return
    if args.command == "sdk-root":
        print(sdk_root())
        return
    project = Project.load(args.project)
    if args.command == "configure-cargo":
        print(configure_cargo(project))
        return
    metadata = (
        prepare_cmake(project)
        if args.command == "cpp"
        else prepare_rust(project, args.manifest, cache=args.output.parent / "dimos-project")
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
