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

"""One offline generation entry point for Python, C++, Rust, and viewer schemas."""

from __future__ import annotations

import argparse
import json
import keyword
import os
from pathlib import Path
import re
import shutil

from . import cpp, python, rust, stubs
from .definitions import Definitions
from .distribution import write_distribution
from .ownership import ABI, Dependency, resolve_owners, schema_hash

TEMPLATES = Path(__file__).with_name("templates")


def generate(
    roots: list[Path],
    output: Path,
    names: list[str] | None = None,
    module: str = "dimos_generated",
    *,
    installed: bool = False,
    version: str = "0.1.0",
    dependencies: tuple[Dependency, ...] = (),
    shared: bool = False,
) -> tuple[str, ...]:
    """Emit all three native language outputs, resolving dependencies first."""
    if not re.fullmatch(r"[a-z][a-z0-9_]*", module) or keyword.iskeyword(module):
        raise ValueError(f"Invalid Python package module name: {module}")
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
        raise ValueError("Package version must have the form major.minor.patch")
    definitions = Definitions(
        roots + [dep.root / "schemas" for dep in dependencies], installed=installed
    )
    messages = definitions.resolve(names)
    for message in messages:
        for field in message.fields:
            if field.type.name == "wstring":
                raise ValueError(
                    f"{message.source}: wstring is not yet supported by the Rust CDR backend"
                )
    owners = resolve_owners(messages, definitions, dependencies)
    owned = tuple(message for message in messages if message.name not in owners)
    imports = tuple(sorted({owner.module for owner in owners.values()}))
    output.mkdir(parents=True, exist_ok=True)
    typing_root = output / "typing" / module
    if typing_root.exists():
        shutil.rmtree(typing_root)
    stub_files = stubs.generate(owned, module)
    for name, content in stub_files.items():
        for imported_name, owner in owners.items():
            content = content.replace(
                f"{module}.{imported_name.replace(chr(47), chr(46))}",
                f"{owner.module}.{imported_name.replace(chr(47), chr(46))}",
            )
        if name.endswith(".pyi") and content:
            content = "".join(f"import {dep}\n" for dep in imports) + content
        path = typing_root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    native = output / "cpp"
    native.mkdir(exist_ok=True)
    (native / "messages.hpp").write_text(cpp.generate(owned, imports))
    shutil.copyfile(TEMPLATES / "dimos_cdr.hpp", native / "dimos_cdr.hpp")
    sources = python.generate(
        owned,
        definitions,
        module,
        imports=imports,
        shared=shared,
        version=version,
        imported={name: owner.module for name, owner in owners.items()},
        dependency_versions={dep.module: dep.version for dep in dependencies},
    )
    package = output / "python" / module
    if package.exists():
        shutil.rmtree(package)
    for name, content in sources.items():
        path = package / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    dependency_cmake = "".join(
        f"find_package({dep.module} {dep.version} EXACT CONFIG REQUIRED)\n" for dep in dependencies
    )
    dependency_targets = " ".join(f"{name}::messages" for name in imports)
    (native / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.20)\n"
        f"project({module} VERSION {version} LANGUAGES CXX)\n"
        "find_package(fastcdr 2.4.0 EXACT REQUIRED)\n"
        "include(GNUInstallDirs)\n"
        + dependency_cmake
        + f"add_library({module}_messages INTERFACE)\n"
        f"set_target_properties({module}_messages PROPERTIES EXPORT_NAME messages)\n"
        f"target_compile_features({module}_messages INTERFACE cxx_std_17)\n"
        f"target_include_directories({module}_messages INTERFACE $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}> $<INSTALL_INTERFACE:include>)\n"
        f"target_link_libraries({module}_messages INTERFACE fastcdr {dependency_targets})\n"
        f"install(TARGETS {module}_messages EXPORT {module}Targets)\n"
        f"install(FILES messages.hpp dimos_cdr.hpp DESTINATION include/{module})\n"
        f"install(EXPORT {module}Targets NAMESPACE {module}:: DESTINATION lib/cmake/{module})\n"
        "include(CMakePackageConfigHelpers)\n"
        f"write_basic_package_version_file(${{CMAKE_CURRENT_BINARY_DIR}}/{module}ConfigVersion.cmake VERSION {version} COMPATIBILITY ExactVersion)\n"
        f"install(FILES {module}Config.cmake ${{CMAKE_CURRENT_BINARY_DIR}}/{module}ConfigVersion.cmake DESTINATION lib/cmake/{module})\n"
        f"install(DIRECTORY ../schemas DESTINATION share/{module})\n"
    )
    (native / f"{module}Config.cmake").write_text(
        "include(CMakeFindDependencyMacro)\nfind_dependency(fastcdr 2.4.0 EXACT)\n"
        + "".join(
            f"find_dependency({dep.module} {dep.version} EXACT CONFIG)\n" for dep in dependencies
        )
        + f'include("${{CMAKE_CURRENT_LIST_DIR}}/{module}Targets.cmake")\n'
    )
    crate = output / "rust"
    (crate / "src").mkdir(parents=True, exist_ok=True)
    rust.write_crate(
        crate, owned, definitions, {name: owner.crate for name, owner in owners.items()}
    )
    codec_owner = (
        (dependencies[0].codec_owner or dependencies[0].module) if dependencies else module
    )
    if dependencies:
        (crate / "src" / "codec.rs").write_text(f"pub use {codec_owner}_messages::codec::*;\n")
    else:
        shutil.copyfile(TEMPLATES / "codec.rs", crate / "src" / "codec.rs")
    (crate / "Cargo.toml").write_text(
        f'[package]\nname = "{module.replace("_", "-")}-messages"\nversion = "{version}"\nedition = "2024"\n'
        'license = "Apache-2.0"\ndescription = "Native CDR messages generated from ROS2 definitions"\n'
        'include = ["src/*.rs", "build.rs", "interfaces/**", "schemas.json", "schemas/**", "Cargo.toml"]\n'
        '[workspace]\n[dependencies]\nserde = { version = "1.0", features = ["derive"] }\n'
        'serde-big-array = "=0.5.1"\nre_cdr = "=0.1.0"\n'
        + "".join(
            f'{dep.module.replace("_", "-")}-messages = {{ version = "={dep.version}", path = {json.dumps(os.path.relpath(dep.root / "rust", crate))} }}\n'
            for dep in dependencies
        )
    )
    with (crate / "Cargo.toml").open("a") as stream:
        stream.write('[build-dependencies]\nros2msg = "=0.5.3"\nheck = "=0.5.0"\n')
    metadata = {message.name: definitions.schema(message.name) for message in messages}
    (output / "schemas.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    shutil.copyfile(output / "schemas.json", crate / "schemas.json")
    schema_root = output / "schemas"
    if schema_root.exists():
        shutil.rmtree(schema_root)
    for message in messages:
        path = schema_root / (message.name + ".msg")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(message.text)
    shutil.copytree(TEMPLATES.parent / "schemas" / "licenses", schema_root / "licenses")
    for source in (TEMPLATES.parent / "schemas").glob("*/package.xml"):
        if (schema_root / source.parent.name).is_dir():
            shutil.copyfile(source, schema_root / source.parent.name / "package.xml")
    if (crate / "schemas").exists():
        shutil.rmtree(crate / "schemas")
    shutil.copytree(schema_root, crate / "schemas", dirs_exist_ok=True)
    (output / "message-package.json").write_text(
        json.dumps(
            {
                "format": 1,
                "abi": ABI,
                "module": module,
                "version": version,
                "shared": shared,
                "codec_owner": codec_owner,
                "dependencies": {dep.module: dep.version for dep in dependencies},
                "owned": [message.name for message in owned],
                "schemas": {name: schema_hash(schema) for name, schema in metadata.items()},
            },
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    # A relocatable header-only CMake package also travels inside message wheels.
    include = output / "include" / module
    include.mkdir(parents=True, exist_ok=True)
    for filename in ("messages.hpp", "dimos_cdr.hpp"):
        shutil.copyfile(native / filename, include / filename)
    config = output / "lib" / "cmake" / module
    config.mkdir(parents=True, exist_ok=True)
    (config / f"{module}Config.cmake").write_text(
        "include(CMakeFindDependencyMacro)\nfind_dependency(fastcdr 2.4.0 EXACT)\n"
        + "".join(
            f"find_dependency({dep.module} {dep.version} EXACT CONFIG)\n" for dep in dependencies
        )
        + f"if(NOT TARGET {module}::messages)\nadd_library({module}::messages INTERFACE IMPORTED)\n"
        + f'set_target_properties({module}::messages PROPERTIES INTERFACE_COMPILE_FEATURES "cxx_std_17" INTERFACE_INCLUDE_DIRECTORIES "${{CMAKE_CURRENT_LIST_DIR}}/../../../include" INTERFACE_LINK_LIBRARIES "fastcdr;{";".join(f"{name}::messages" for name in imports)}")\nendif()\n'
    )
    (config / f"{module}ConfigVersion.cmake").write_text(
        f'set(PACKAGE_VERSION "{version}")\n'
        "if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)\n"
        "set(PACKAGE_VERSION_EXACT TRUE)\nset(PACKAGE_VERSION_COMPATIBLE TRUE)\nendif()\n"
    )
    return tuple(message.name for message in messages)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, action="append", default=[])
    parser.add_argument(
        "--type", dest="names", action="append", help="Root package/msg/Type (repeatable)"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--python-module", default="dimos_generated")
    parser.add_argument("--version", default="0.1.0")
    parser.add_argument(
        "--package", action="store_true", help="Emit a reproducible Python wheel/sdist project"
    )
    args = parser.parse_args()
    try:
        names = generate(
            args.package_root,
            args.output,
            args.names,
            args.python_module,
            installed=True,
            version=args.version,
        )
        if args.package:
            write_distribution(args.output, args.python_module, names, args.version)
    except ValueError as exc:
        parser.error(str(exc))
    print(f"Generated {len(names)} message types for Python, C++, and Rust in {args.output}")


if __name__ == "__main__":
    main()
