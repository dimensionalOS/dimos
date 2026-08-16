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

import typer
from typer.testing import CliRunner

from dimos.cli.bake.cli import bake, emit_config
from dimos.cli.bake.codegen import render_main_rs
from dimos.cli.bake.discovery import discover_modules, select_modules
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
