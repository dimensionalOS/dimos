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

"""Package message definitions and native-library entry points without compiling."""

from __future__ import annotations

import argparse
import json
import keyword
import os
from pathlib import Path
import re
import shutil

from . import cpp, python, rust
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
    languages: tuple[str, ...] = ("python", "cpp", "rust"),
) -> tuple[str, ...]:
    if not re.fullmatch(r"[a-z][a-z0-9_]*", module) or keyword.iskeyword(module):
        raise ValueError(f"Invalid Python package module name: {module}")
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
        raise ValueError("Package version must have the form major.minor.patch")
    if not languages or set(languages) - {"python", "cpp", "rust"}:
        raise ValueError("Select python, cpp and/or rust")
    definitions = Definitions(
        roots + [dep.root / "schemas" for dep in dependencies], installed=installed
    )
    messages = definitions.resolve(names)
    owners = resolve_owners(messages, definitions, dependencies)
    owned = tuple(message for message in messages if message.name not in owners)
    output.mkdir(parents=True, exist_ok=True)
    # Remove outputs of languages no longer selected, including old adapters.
    for name in ["cpp", "rust", "python", "include", "lib", "typing", "schemas"]:
        if (output / name).exists():
            shutil.rmtree(output / name)
    schema_root = output / "schemas"
    for message in messages:
        path = schema_root / (message.name + ".msg")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(message.text)
    shutil.copytree(TEMPLATES.parent / "schemas/licenses", schema_root / "licenses")
    metadata = {msg.name: definitions.schema(msg.name) for msg in messages}
    (output / "schemas.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    unsupported: dict[str, dict[str, str]] = {
        language: {} for language in ["python", "cpp", "rust"]
    }
    defaults = []
    for message in messages:
        closure = definitions.resolve([message.name])
        if any(field.default is not None for msg in closure for field in msg.fields):
            defaults.append(message.name)
        if any(
            field.type.array_bounded or field.type.string_bound is not None
            for msg in closure
            for field in msg.fields
        ):
            unsupported["python"][message.name] = "rosbags does not enforce declared bounds"
        if any(field.type.string_bound is not None for msg in closure for field in msg.fields):
            unsupported["cpp"][message.name] = "upstream FastRTPS does not enforce bounded strings"
            unsupported["rust"][message.name] = "native bounded-string semantics not validated"
        if any(field.type.name == "wstring" for msg in closure for field in msg.fields):
            unsupported["rust"][message.name] = "re_cdr wstring mapping not validated"
    manifest = {
        "format": 1,
        "abi": ABI,
        "module": module,
        "version": version,
        "shared": shared,
        "dependencies": {dep.module: dep.version for dep in dependencies},
        "owned": [msg.name for msg in owned],
        "schemas": {name: schema_hash(schema) for name, schema in metadata.items()},
        "unsupported": unsupported,
        "native_construction": {
            "python": "explicit fields; no implicit defaults",
            "rust": "explicit fields; upstream Default generation unsupported",
            "cpp": "upstream default constructors",
        },
        "types_with_declared_defaults": defaults,
    }
    (output / "message-package.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    if "python" in languages:
        for name, content in python.generate(
            owned,
            module,
            version=version,
            imports={name: owner.module for name, owner in owners.items()},
        ).items():
            path = output / "python" / module / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
    if "cpp" in languages:
        if unsupported["cpp"]:
            raise NotImplementedError(f"Unsupported native C++ semantics: {unsupported['cpp']}")
        cpp.write_project(output / "cpp", owned, module, version)
    if "rust" in languages:
        if unsupported["rust"]:
            raise NotImplementedError(f"Unsupported native Rust semantics: {unsupported['rust']}")
        crate = output / "rust"
        crate.mkdir()
        shutil.copyfile(output / "schemas.json", crate / "schemas.json")
        rust.write_crate(
            crate, owned, definitions, {name: owner.crate for name, owner in owners.items()}
        )
        (crate / "Cargo.toml").write_text(
            f'[package]\nname = "{module.replace("_", "-")}-messages"\nversion = "{version}"\nedition = "2024"\n'
            'license = "Apache-2.0"\n[workspace]\n[dependencies]\n'
            'serde = {version = "1", features = ["derive"]}\nserde-big-array = "=0.5.1"\n'
            're_cdr = "=0.1.0"\nheapless = {version = "=0.8.0", features = ["serde"]}\n'
            + "".join(
                f'{dep.module.replace("_", "-")}-messages = {{version = "={dep.version}", path = {json.dumps(os.path.relpath(dep.root / "rust", crate))}}}\n'
                for dep in dependencies
            )
            + '[build-dependencies]\nros2msg = "=0.5.3"\n'
        )
    if "python" in languages:
        write_distribution(
            output, module, tuple(msg.name for msg in owned), version, dependencies=dependencies
        )
    return tuple(msg.name for msg in messages)


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
