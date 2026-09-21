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

"""Tests for the bake CLI entry point and its config emission."""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import pytest
import typer
from typer.testing import CliRunner

from dimos.cli.bake import cli
from dimos.cli.bake.cli import bake, default_config, emit_config
from dimos.cli.bake.codegen import render_main_rs
from dimos.cli.bake.discovery import discover_modules, select_modules
from dimos.cli.bake.errors import BakeError
from dimos.cli.bake.graph import build_graph


def app() -> typer.Typer:
    cli = typer.Typer(add_completion=False)
    cli.command()(bake)
    return cli


def test_list_prints_the_registry() -> None:
    result = CliRunner().invoke(app(), ["--list"])
    assert result.exit_code == 0
    assert "Registered native modules:" in result.output
    assert "ray_tracing" in result.output
    assert "mls_planner" in result.output


def test_a_bake_error_exits_nonzero() -> None:
    result = CliRunner().invoke(app(), ["ray_tracing", "-o", "bad.name"])
    assert result.exit_code == 1
    assert "cannot name the host binary" in result.output


def test_out_is_required_for_a_build() -> None:
    result = CliRunner().invoke(app(), ["ray_tracing"])
    assert result.exit_code == 1
    assert "-o/--out is required" in result.output


def test_dry_run_prints_the_graph_and_skips_the_build(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[str] = []
    monkeypatch.setattr(cli, "generate_crate", lambda *a, **k: calls.append("generate"))
    result = CliRunner().invoke(
        app(), ["ray_tracing", "-o", str(tmp_path / "hostbin"), "--dry-run"]
    )
    assert result.exit_code == 0
    assert "Host `hostbin`: ray_tracing" in result.output
    assert calls == []


def test_emit_config_writes_one_json_line_and_creates_parent_dirs(tmp_path: Path) -> None:
    dest = tmp_path / "deploy" / "configs" / "hostbin.json"
    result = CliRunner().invoke(
        app(),
        ["ray_tracing", "-o", str(tmp_path / "hostbin"), "--dry-run", "--emit-config", str(dest)],
    )
    assert result.exit_code == 0
    text = dest.read_text()
    # The host reads its config with a single read_line.
    assert text.endswith("\n")
    assert "\n" not in text[:-1]
    assert set(json.loads(text)["modules"]) == {"ray_tracing"}


def test_default_config_reports_an_unimportable_wrapper() -> None:
    entry = discover_modules()["ray_tracing"]
    missing_module = dataclasses.replace(entry, python_ref="dimos.no_such_module:Wrapper")
    with pytest.raises(BakeError, match="cannot import"):
        default_config(missing_module)
    missing_class = dataclasses.replace(entry, python_ref="dimos.cli.bake.cli:NoSuchClass")
    with pytest.raises(BakeError, match="cannot import"):
        default_config(missing_class)


def test_emit_config_leaves_the_session_to_the_deployment() -> None:
    """Only the machine the host runs on knows its interface and endpoints."""
    selected = select_modules(discover_modules(), ["ray_tracing", "mls_planner"])
    graph = build_graph("go2-nav", selected)
    assert "session" not in emit_config(graph, selected)


def test_emit_config_is_a_complete_stdin_blob() -> None:
    registry = discover_modules()
    selected = select_modules(registry, ["ray_tracing", "mls_planner"])
    graph = build_graph("go2-nav", selected, suppress=["local_map"])

    blob = emit_config(graph, selected)

    assert set(blob["modules"]) == {"ray_tracing", "mls_planner"}
    for section in blob["modules"].values():
        assert isinstance(section["config"], dict) and section["config"]
    assert blob["modules"]["ray_tracing"]["topics"]["lidar"] == (
        "dimos/lidar/sensor_msgs.PointCloud2"
    )
    assert blob["suppress"] == list(graph.suppressed_topics())
    assert blob["qos"] == graph.qos()
    # The stamp the binary checks its config against, from the same graph.
    assert f'graph_hash: "{blob["graph"]}",' in render_main_rs("go2-nav", selected, graph)
