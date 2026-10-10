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

"""Package unchanged definitions for native ros2msg generation in Cargo."""

from __future__ import annotations

from pathlib import Path
import shutil

from .definitions import Definitions, Message


def write_crate(
    crate: Path,
    messages: tuple[Message, ...],
    definitions: Definitions,
    imports: dict[str, str] | None = None,
) -> None:
    imports = imports or {}
    namespaces: dict[str, str] = {}
    for name, owner in imports.items():
        package = name.split("/")[0]
        if namespaces.setdefault(package, owner) != owner:
            raise ValueError(f"Native Rust requires one owner per ROS package: {package}")
    if {message.package for message in messages} & namespaces.keys():
        raise ValueError("Native Rust cannot split a ROS package across dependency crates")
    if (crate / "interfaces").exists():
        shutil.rmtree(crate / "interfaces")
    for message in messages:
        source = crate / "interfaces" / (message.name + ".msg")
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(message.text)
    shutil.copyfile(Path(__file__).with_name("templates") / "message_build.rs", crate / "build.rs")
    (crate / "src").mkdir(parents=True, exist_ok=True)
    (crate / "src/lib.rs").write_text(
        "".join(f"pub use {owner}::{package};\n" for package, owner in sorted(namespaces.items()))
        + 'pub const ROS2MSG_SCHEMAS: &str = include_str!("../schemas.json");\n'
        + 'include!(concat!(env!("OUT_DIR"), "/mod.rs"));\n'
    )
