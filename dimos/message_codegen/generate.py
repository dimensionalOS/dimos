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

TEMPLATES = Path(__file__).with_name("templates")


def generate(
    roots: list[Path], output: Path, names: list[str] | None = None, module: str = "dimos_generated"
) -> tuple[str, ...]:
    """Emit all three native language outputs, resolving dependencies first."""
    if not re.fullmatch(r"[a-z][a-z0-9_]*", module) or keyword.iskeyword(module):
        raise ValueError(f"Invalid Python extension module name: {module}")
    definitions = Definitions(roots)
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
    package = output / "python" / module
    if package.exists():
        shutil.rmtree(package)
    for name, content in python.generate(messages, definitions, module).items():
        path = package / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    (native / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.20)\n"
        "project(dimos_generated_messages LANGUAGES CXX)\n"
        "find_package(fastcdr 2.4.0 EXACT REQUIRED)\n"
        f"add_library({module}_messages INTERFACE)\n"
        f"target_compile_features({module}_messages INTERFACE cxx_std_17)\n"
        f"target_include_directories({module}_messages INTERFACE ${{CMAKE_CURRENT_SOURCE_DIR}})\n"
        f"target_link_libraries({module}_messages INTERFACE fastcdr)\n"
    )
    crate = output / "rust"
    (crate / "src").mkdir(parents=True, exist_ok=True)
    rust.write_crate(crate, messages, definitions)
    shutil.copyfile(TEMPLATES / "codec.rs", crate / "src" / "codec.rs")
    (crate / "Cargo.toml").write_text(
        '[package]\nname = "dimos-generated-messages"\nversion = "0.1.0"\nedition = "2024"\n'
        '[workspace]\n[dependencies]\nserde = { version = "1.0", features = ["derive"] }\n'
        'serde-big-array = "=0.5.1"\nre_cdr = "=0.1.0"\n'
        '[build-dependencies]\nros2msg = "=0.5.3"\nheck = "=0.5.0"\n'
    )
    metadata = {message.name: definitions.schema(message.name) for message in messages}
    (output / "schemas.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    return tuple(message.name for message in messages)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, action="append", default=[])
    parser.add_argument(
        "--type", dest="names", action="append", help="Root package/msg/Type (repeatable)"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--python-module", default="dimos_generated")
    args = parser.parse_args()
    try:
        names = generate(args.package_root, args.output, args.names, args.python_module)
    except ValueError as exc:
        parser.error(str(exc))
    print(f"Generated {len(names)} message types for Python, C++, and Rust in {args.output}")


if __name__ == "__main__":
    main()
