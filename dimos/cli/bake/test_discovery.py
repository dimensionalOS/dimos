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

from pathlib import Path

import pytest

from dimos.cli.bake import BakeError
from dimos.cli.bake.discovery import (
    discover_modules,
    parse_manifest,
    render_registry,
    select_modules,
)

MANIFEST = """
[package]
name = "demo-crate"

[package.metadata.dimos.module.demo]
path = "demo::module::Demo"
python = "dimos.demo.module:Demo"
threads = 2
nice = 5

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


def test_parses_a_registered_module(tmp_path):
    (info,) = parse_manifest(write_crate(tmp_path, "dimos/demo/rust", MANIFEST))
    assert info.id == "demo"
    assert info.crate_name == "demo-crate"
    assert info.rust_path == "demo::module::Demo"
    assert info.python_ref == "dimos.demo.module:Demo"
    assert info.threads == 2
    assert info.nice == 5
    assert info.inputs == {"lidar": "sensor_msgs.PointCloud2"}
    assert info.outputs == {"global_map": "sensor_msgs.PointCloud2"}


def test_a_crate_without_the_metadata_table_is_not_a_module(tmp_path):
    manifest = '[package]\nname = "plain"\n'
    assert parse_manifest(write_crate(tmp_path, "dimos/plain/rust", manifest)) == []


def test_threads_defaults_to_one(tmp_path):
    manifest = MANIFEST.replace("threads = 2\n", "").replace("nice = 5\n", "")
    (info,) = parse_manifest(write_crate(tmp_path, "dimos/demo/rust", manifest))
    assert info.threads == 1
    assert info.nice is None


def test_a_missing_required_key_is_an_error(tmp_path):
    manifest = MANIFEST.replace('python = "dimos.demo.module:Demo"\n', "")
    with pytest.raises(BakeError, match="python"):
        parse_manifest(write_crate(tmp_path, "dimos/demo/rust", manifest))


def test_a_non_string_msg_type_is_an_error(tmp_path):
    manifest = MANIFEST.replace('lidar = "sensor_msgs.PointCloud2"', "lidar = 3")
    with pytest.raises(BakeError, match="lidar"):
        parse_manifest(write_crate(tmp_path, "dimos/demo/rust", manifest))


def test_discovery_walks_dimos_and_native_and_skips_target(tmp_path):
    write_crate(tmp_path, "dimos/demo/rust", MANIFEST)
    write_crate(tmp_path, "native/rust/other", MANIFEST.replace("module.demo", "module.other"))
    write_crate(
        tmp_path,
        "dimos/demo/rust/target/debug/build/junk",
        MANIFEST.replace("module.demo", "module.stale"),
    )
    assert sorted(discover_modules(tmp_path)) == ["demo", "other"]


def test_the_same_id_in_two_crates_is_an_error(tmp_path):
    write_crate(tmp_path, "dimos/a/rust", MANIFEST)
    write_crate(tmp_path, "dimos/b/rust", MANIFEST)
    with pytest.raises(BakeError, match="declared twice"):
        discover_modules(tmp_path)


def test_selection_normalizes_dashes_and_keeps_order(tmp_path):
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


def test_selecting_an_unknown_module_lists_what_exists(tmp_path):
    write_crate(tmp_path, "dimos/a/rust", MANIFEST)
    with pytest.raises(BakeError, match="registered modules: demo"):
        select_modules(discover_modules(tmp_path), ["nope"])


def test_the_same_module_twice_is_rejected(tmp_path):
    write_crate(tmp_path, "dimos/a/rust", MANIFEST)
    with pytest.raises(BakeError, match="namespacing"):
        select_modules(discover_modules(tmp_path), ["demo", "demo"])


def test_the_real_repo_registers_the_two_shipped_modules():
    registry = discover_modules()
    assert {"ray_tracing", "mls_planner"} <= set(registry)
    assert registry["mls_planner"].threads == 2
    assert "Registered native modules" in render_registry(registry)
