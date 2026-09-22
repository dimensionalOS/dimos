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

"""Tests for the bake registry: what a Cargo.toml has to say to be bakeable."""

import importlib
from pathlib import Path
from typing import get_args, get_origin, get_type_hints

import pytest

from dimos.cli.bake.discovery import (
    discover_modules,
    parse_manifest,
    render_registry,
    select_modules,
)
from dimos.cli.bake.errors import BakeError
from dimos.core.stream import IO, In, Out
from dimos.core.transport_factory import zenoh_key_expr
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic

MANIFEST = """
[package]
name = "demo-crate"

[package.metadata.dimos.module.demo]
path = "demo::module::Demo"
python = "dimos.demo.module:Demo"
threads = 2

[package.metadata.dimos.module.demo.inputs]
lidar = "sensor_msgs.PointCloud2"

[package.metadata.dimos.module.demo.outputs]
global_map = "sensor_msgs.PointCloud2"
"""


def write_crate(root: Path, relative: str, manifest: str) -> Path:
    path = root / relative / "Cargo.toml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(manifest)
    return path


def test_parses_a_registered_module(tmp_path: Path) -> None:
    (info,) = parse_manifest(write_crate(tmp_path, "dimos/demo/rust", MANIFEST))
    assert info.id == "demo"
    assert info.crate_name == "demo-crate"
    assert info.rust_path == "demo::module::Demo"
    assert info.python_ref == "dimos.demo.module:Demo"
    assert info.threads == 2
    assert info.inputs == {"lidar": "sensor_msgs.PointCloud2"}
    assert info.outputs == {"global_map": "sensor_msgs.PointCloud2"}


def test_a_crate_without_the_metadata_table_is_not_a_module(tmp_path: Path) -> None:
    manifest = '[package]\nname = "plain"\n'
    assert parse_manifest(write_crate(tmp_path, "dimos/plain/rust", manifest)) == []


def test_threads_defaults_to_one(tmp_path: Path) -> None:
    manifest = MANIFEST.replace("threads = 2\n", "")
    (info,) = parse_manifest(write_crate(tmp_path, "dimos/demo/rust", manifest))
    assert info.threads == 1


def test_a_missing_required_key_is_an_error(tmp_path: Path) -> None:
    manifest = MANIFEST.replace('python = "dimos.demo.module:Demo"\n', "")
    with pytest.raises(BakeError, match="python"):
        parse_manifest(write_crate(tmp_path, "dimos/demo/rust", manifest))


def test_a_non_string_msg_type_is_an_error(tmp_path: Path) -> None:
    manifest = MANIFEST.replace('lidar = "sensor_msgs.PointCloud2"', "lidar = 3")
    with pytest.raises(BakeError, match="lidar"):
        parse_manifest(write_crate(tmp_path, "dimos/demo/rust", manifest))


def test_discovery_walks_dimos_and_native_and_skips_target(tmp_path: Path) -> None:
    write_crate(tmp_path, "dimos/demo/rust", MANIFEST)
    write_crate(tmp_path, "native/rust/other", MANIFEST.replace("module.demo", "module.other"))
    write_crate(
        tmp_path,
        "dimos/demo/rust/target/debug/build/junk",
        MANIFEST.replace("module.demo", "module.stale"),
    )
    assert sorted(discover_modules(tmp_path)) == ["demo", "other"]


def test_the_same_id_in_two_crates_is_an_error(tmp_path: Path) -> None:
    write_crate(tmp_path, "dimos/a/rust", MANIFEST)
    write_crate(tmp_path, "dimos/b/rust", MANIFEST)
    with pytest.raises(BakeError, match="declared twice"):
        discover_modules(tmp_path)


def test_selection_normalizes_dashes_and_keeps_order(tmp_path: Path) -> None:
    write_crate(tmp_path, "dimos/a/rust", MANIFEST)
    write_crate(
        tmp_path,
        "dimos/b/rust",
        MANIFEST.replace("module.demo", "module.other_one"),
    )
    registry = discover_modules(tmp_path)
    assert [m.id for m in select_modules(registry, ["other-one", "demo"])] == [
        "other_one",
        "demo",
    ]


def test_selecting_an_unknown_module_lists_what_exists(tmp_path: Path) -> None:
    write_crate(tmp_path, "dimos/a/rust", MANIFEST)
    with pytest.raises(BakeError, match="registered modules: demo"):
        select_modules(discover_modules(tmp_path), ["nope"])


def test_the_same_module_twice_is_rejected(tmp_path: Path) -> None:
    write_crate(tmp_path, "dimos/a/rust", MANIFEST)
    with pytest.raises(BakeError, match="namespacing"):
        select_modules(discover_modules(tmp_path), ["demo", "demo"])


def test_the_real_repo_registers_the_two_shipped_modules() -> None:
    registry = discover_modules()
    assert {"ray_tracing", "mls_planner"} <= set(registry)
    assert registry["mls_planner"].threads == 2
    assert "Registered native modules" in render_registry(registry)


def wrapper_ports(python_ref: str) -> dict[str, type]:
    """The python wrapper's ports, keyed by name, as message classes.

    `IO` counts: a port that both subscribes and publishes (tf) rides in the
    manifest's `inputs` table, because the bake graph has no io kind.
    """
    module_name, _, class_name = python_ref.partition(":")
    wrapper = getattr(importlib.import_module(module_name), class_name)
    return {
        name: get_args(hint)[0]
        for name, hint in get_type_hints(wrapper).items()
        if get_origin(hint) in (In, Out, IO)
    }


def test_every_baked_port_lands_on_the_key_its_python_wrapper_subscribes_to() -> None:
    """A manifest naming the rust payload rather than the dimos message goes dark."""
    for module in discover_modules().values():
        ports = wrapper_ports(module.python_ref)
        for port, msg_type in module.ports.items():
            assert port in ports, f"{module.id}.{port} is not a port of {module.python_ref}"
            subscribed = ZenohTopic(f"dimos/{port}", ports[port]).key_expr
            assert zenoh_key_expr(port, msg_type) == subscribed
