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

import math
from types import SimpleNamespace

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hosted.daemon import HostDescriptor
from dimos.hosted.demo_compose_scenarios import (
    MovingMujocoSimModule,
    ScenarioName,
    _circle_pose,
    _visual_topic_to_entity,
    build_scenario,
)
from dimos.hosted.fragment_compiler import compile_fragments
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _host(host_id: str, name: str, tag: str) -> HostDescriptor:
    return HostDescriptor(
        host_id=host_id,
        epoch=f"{host_id}-epoch",
        name=name,
        tags=frozenset({tag}),
        versions={"application_revision": "test-revision"},
        state="available",
        active_run_ids=(),
    )


HOSTS = (
    _host("edge-id", "edge-a", "edge"),
    _host("compute-id", "compute-a", "compute"),
    _host("replay-id", "replay-a", "replay"),
    _host("sim-id", "sim-a", "sim"),
)


@pytest.mark.parametrize("name", ["basic", "replay", "sim", "visual"])
def test_scenario_auto_placement_matches_compose_topology(name: ScenarioName) -> None:
    scenario = build_scenario(name, f"test-{name}")
    config = BlueprintConfigParser(scenario.blueprint).parse(environ={})

    fragments = compile_fragments(
        scenario.blueprint,
        config,
        run_id=f"test-{name}",
        generation=1,
        application_name="compose-test",
        application_revision="test-revision",
        hosts=HOSTS,
        local_host_id="controller-id",
    )

    host_names = {host.host_id: host.name for host in HOSTS}
    host_names["controller-id"] = "controller"
    actual = {
        atom.name: host_names[host_id]
        for host_id, fragment in fragments.items()
        for atom in fragment.load_payload().blueprint.active_blueprints
    }
    assert actual == scenario.expected_hosts
    assert len(fragments) == 3


def test_visual_scenario_keeps_rerun_on_controller() -> None:
    scenario = build_scenario("visual", "test-visual")

    assert scenario.expected_hosts[MovingMujocoSimModule.name] == "sim-a"
    assert scenario.expected_hosts[RerunBridgeModule.name] == "controller"
    assert scenario.keep_running is True
    assert scenario.viewer_url == (
        "http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy"
    )


def test_circle_pose_moves_tangentially_around_the_scene() -> None:
    assert _circle_pose(elapsed_seconds=0.0, radius=1.0, period_seconds=8.0) == pytest.approx(
        (1.0, 0.0, math.pi / 2.0)
    )
    assert _circle_pose(elapsed_seconds=2.0, radius=1.0, period_seconds=8.0) == pytest.approx(
        (0.0, 1.0, math.pi)
    )


def test_visual_maps_hosted_run_odometry_to_the_robot_entity() -> None:
    hosted_topic = SimpleNamespace(
        topic="dimos/runs/test-visual/streams/odom",
    )

    assert _visual_topic_to_entity(hosted_topic) == "world/odom"
    assert _visual_topic_to_entity(SimpleNamespace(topic="dimos/debug")) == "world/debug"


@pytest.mark.parametrize("name", ["basic", "replay", "sim"])
def test_validation_scenarios_exit_after_the_result(name: ScenarioName) -> None:
    scenario = build_scenario(name, f"test-{name}")

    assert scenario.keep_running is False
    assert scenario.viewer_url is None
