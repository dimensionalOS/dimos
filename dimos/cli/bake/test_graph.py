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

"""Tests for the bake graph: who connects to whom, and what bake refuses."""

from pathlib import Path

import pytest

from dimos.cli.bake.discovery import RegisteredModule
from dimos.cli.bake.errors import BakeError
from dimos.cli.bake.graph import Connection, Graph, build_graph, parse_remap, render

PC2 = "sensor_msgs.PointCloud2"
POSE = "geometry_msgs.PoseStamped"


def module(
    module_id: str, inputs: dict[str, str], outputs: dict[str, str], threads: int = 1
) -> RegisteredModule:
    return RegisteredModule(
        id=module_id,
        crate_dir=Path("/crates") / module_id,
        crate_name=f"crate-{module_id}",
        rust_path=f"{module_id}::module::Thing",
        python_ref=f"dimos.{module_id}:Thing",
        threads=threads,
        inputs=inputs,
        outputs=outputs,
    )


MAPPER = module("mapper", {"lidar": PC2}, {"local_map": PC2, "region_bounds": POSE})
PLANNER = module("planner", {"local_map": PC2, "goal": POSE}, {"path": POSE})


def by_name(graph: Graph) -> dict[str, Connection]:
    return {c.name: c for c in graph.connections}


def test_matching_names_connect_and_classify() -> None:
    graph = build_graph("host", [MAPPER, PLANNER])
    conns = by_name(graph)
    assert conns["local_map"].kind == "internal"
    assert conns["lidar"].kind == "external_input"
    assert conns["goal"].kind == "external_input"
    assert conns["path"].kind == "external_output"
    # Produced but consumed by nobody in the host: still an output.
    assert conns["region_bounds"].kind == "external_output"


def test_topics_carry_the_zenoh_namespace_and_the_message_type() -> None:
    graph = build_graph("host", [MAPPER, PLANNER])
    assert by_name(graph)["local_map"].topic == f"dimos/local_map/{PC2}"
    assert graph.topics()["mapper"]["lidar"] == f"dimos/lidar/{PC2}"


def test_pointcloud_topics_get_latest_wins_qos() -> None:
    graph = build_graph("host", [MAPPER, PLANNER])
    assert graph.qos()[f"dimos/local_map/{PC2}"] == {
        "reliability": "best_effort",
        "congestion_control": "drop",
    }
    # A PoseStamped channel keeps zenoh's defaults, so bake bakes nothing.
    assert f"dimos/goal/{POSE}" not in graph.qos()


def test_never_drop_channels_get_reliable_qos() -> None:
    talker = module("talker", {}, {"command": POSE})
    graph = build_graph("host", [talker])
    assert graph.qos()[f"dimos/command/{POSE}"] == {
        "reliability": "reliable",
        "congestion_control": "block",
    }


def test_same_name_different_type_is_an_error() -> None:
    clash = module("clash", {"path": PC2}, {})
    with pytest.raises(BakeError, match="--remap"):
        build_graph("host", [PLANNER, clash])


def test_remap_resolves_a_name_clash() -> None:
    clash = module("clash", {"path": PC2}, {})
    graph = build_graph("host", [PLANNER, clash], remaps={("clash", "path"): "cloud"})
    conns = by_name(graph)
    assert conns["cloud"].kind == "external_input"
    assert conns["cloud"].remapped
    assert graph.topics()["clash"]["path"] == f"dimos/cloud/{PC2}"


def test_remap_connects_two_differently_named_ports() -> None:
    graph = build_graph("host", [MAPPER, PLANNER], remaps={("planner", "local_map"): "surface"})
    conns = by_name(graph)
    assert conns["local_map"].kind == "external_output"
    assert conns["surface"].kind == "external_input"


def test_remapping_an_unknown_port_is_an_error() -> None:
    with pytest.raises(BakeError, match="unknown port"):
        build_graph("host", [MAPPER], remaps={("mapper", "nope"): "x"})


def test_multiple_producers_warn_but_build() -> None:
    other = module("other", {}, {"local_map": PC2})
    graph = build_graph("host", [MAPPER, PLANNER, other])
    assert by_name(graph)["local_map"].kind == "internal"
    assert any("2 producers" in w for w in graph.warnings)


def test_suppression_accepts_a_name_or_a_topic() -> None:
    topic = f"dimos/local_map/{PC2}"
    graph = build_graph("host", [MAPPER, PLANNER], suppress=["local_map"])
    assert graph.suppressed_topics() == (topic,)
    same = build_graph("host", [MAPPER, PLANNER], suppress=[topic])
    assert same.suppressed_topics() == (topic,)


def test_suppressing_an_external_input_is_refused() -> None:
    with pytest.raises(BakeError, match="external input"):
        build_graph("host", [MAPPER, PLANNER], suppress=["lidar"])


def test_suppressing_an_unconsumed_output_warns() -> None:
    graph = build_graph("host", [MAPPER, PLANNER], suppress=["region_bounds"])
    assert any("no baked module consumes it" in w for w in graph.warnings)


def test_suppressing_an_unknown_topic_is_an_error() -> None:
    with pytest.raises(BakeError, match="no baked module touches"):
        build_graph("host", [MAPPER, PLANNER], suppress=["nowhere"])


def test_parse_remap_rejects_junk() -> None:
    assert parse_remap("a.b=c") == (("a", "b"), "c")
    assert parse_remap("a-b.c=d") == (("a_b", "c"), "d")
    with pytest.raises(BakeError, match="malformed"):
        parse_remap("a.b")


def test_rendered_graph_is_grouped_by_direction() -> None:
    graph = build_graph("host", [MAPPER, PLANNER], suppress=["local_map"])
    assert render(graph).splitlines() == [
        "Host `host`: mapper, planner",
        "",
        "Internal connections:",
        "  dimos/local_map/sensor_msgs.PointCloud2  sensor_msgs.PointCloud2  [SUPPRESSED]",
        "      out  mapper.local_map",
        "      in   planner.local_map",
        "",
        "External inputs (subscribed):",
        "  dimos/goal/geometry_msgs.PoseStamped  geometry_msgs.PoseStamped",
        "      in   planner.goal",
        "  dimos/lidar/sensor_msgs.PointCloud2  sensor_msgs.PointCloud2",
        "      in   mapper.lidar",
        "",
        "External outputs (published):",
        "  dimos/path/geometry_msgs.PoseStamped  geometry_msgs.PoseStamped",
        "      out  planner.path",
        "  dimos/region_bounds/geometry_msgs.PoseStamped  geometry_msgs.PoseStamped",
        "      out  mapper.region_bounds",
    ]


def test_graph_json_round_trips_the_classification() -> None:
    graph = build_graph("host", [MAPPER, PLANNER], suppress=["local_map"])
    payload = graph.to_json()
    assert payload["host"] == "host"
    assert payload["modules"] == ["mapper", "planner"]
    internal = next(c for c in payload["connections"] if c["name"] == "local_map")
    assert internal["kind"] == "internal"
    assert internal["suppressed"] is True
    assert internal["producers"] == [{"module": "mapper", "port": "local_map"}]
