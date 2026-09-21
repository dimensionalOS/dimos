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
from pathlib import Path
import re
import shutil

from dimos.message_codegen import cpp, python, rust
from dimos.message_codegen.definitions import Definitions
from dimos.message_codegen.distribution import write_distribution

TEMPLATES = Path(__file__).with_name("templates")


def generate(
    roots: list[Path],
    output: Path,
    names: list[str] | None = None,
    module: str = "dimos_generated",
    *,
    installed: bool = False,
    version: str = "0.1.0",
) -> tuple[str, ...]:
    """Emit all three native language outputs, resolving dependencies first."""
    if not re.fullmatch(r"[a-z][a-z0-9_]*", module) or keyword.iskeyword(module):
        raise ValueError(f"Invalid Python extension module name: {module}")
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
        raise ValueError("Package version must have the form major.minor.patch")
    definitions = Definitions(roots, installed=installed)
    messages = definitions.resolve(names)
    for message in messages:
        for field in message.fields:
            if field.type.name == "wstring":
                raise ValueError(
                    f"{message.source}: wstring is not yet supported by the Rust CDR backend"
                )
    output.mkdir(parents=True, exist_ok=True)
    native = output / "cpp"
    native.mkdir(exist_ok=True)
    (native / "messages.hpp").write_text(cpp.generate(messages))
    shutil.copyfile(TEMPLATES / "dimos_cdr.hpp", native / "dimos_cdr.hpp")
    shutil.copyfile(TEMPLATES / "dimos_python.hpp", native / "dimos_python.hpp")
    bindings = python.generate(messages, definitions, module)
    for name, content in bindings.items():
        (native / name).write_text(content)
    sources = " ".join(sorted(bindings))
    (native / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.20)\n"
        f"project({module} VERSION {version} LANGUAGES CXX)\n"
        "find_package(fastcdr 2.4.0 EXACT REQUIRED)\n"
        "include(GNUInstallDirs)\n"
        f"add_library({module}_messages INTERFACE)\n"
        f"set_target_properties({module}_messages PROPERTIES EXPORT_NAME messages)\n"
        f"target_compile_features({module}_messages INTERFACE cxx_std_17)\n"
        f"target_include_directories({module}_messages INTERFACE $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}> $<INSTALL_INTERFACE:include>)\n"
        f"target_link_libraries({module}_messages INTERFACE fastcdr)\n"
        f"install(TARGETS {module}_messages EXPORT {module}Targets)\n"
        f"install(FILES messages.hpp dimos_cdr.hpp DESTINATION include/{module})\n"
        f"install(EXPORT {module}Targets NAMESPACE {module}:: DESTINATION lib/cmake/{module})\n"
        f"install(FILES {module}Config.cmake DESTINATION lib/cmake/{module})\n"
        f"install(DIRECTORY ../schemas DESTINATION share/{module})\n"
        'option(DIMOS_BUILD_PYTHON "Build Python bindings" ON)\n'
        "if(DIMOS_BUILD_PYTHON)\n"
        "find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)\n"
        'execute_process(COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)\n'
        "find_package(pybind11 3.0.1 EXACT REQUIRED)\n"
        f"pybind11_add_module({module} NO_EXTRAS {sources})\n"
        f"target_compile_features({module} PRIVATE cxx_std_17)\n"
        f"target_link_libraries({module} PRIVATE fastcdr)\n"
        "endif()\n"
    )
    (native / f"{module}Config.cmake").write_text(
        "include(CMakeFindDependencyMacro)\nfind_dependency(fastcdr 2.4.0 EXACT)\n"
        f'include("${{CMAKE_CURRENT_LIST_DIR}}/{module}Targets.cmake")\n'
    )
    crate = output / "rust"
    (crate / "src").mkdir(parents=True, exist_ok=True)
    (crate / "src" / "lib.rs").write_text(rust.generate(messages, definitions))
    shutil.copyfile(TEMPLATES / "codec.rs", crate / "src" / "codec.rs")
    (crate / "Cargo.toml").write_text(
        f'[package]\nname = "{module.replace("_", "-")}-messages"\nversion = "{version}"\nedition = "2024"\n'
        'license = "Apache-2.0"\ndescription = "Native CDR messages generated from ROS2 definitions"\n'
        'include = ["src/*.rs", "schemas.json", "schemas/**", "Cargo.toml"]\n'
        '[workspace]\n[dependencies]\nserde = { version = "1.0", features = ["derive"] }\n'
        'serde-big-array = "=0.5.1"\nre_cdr = "=0.1.0"\n'
    )
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
