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

from typing import Any

import pytest

from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport_factory import transport_topic


class Cloud:
    pass


class Status:
    pass


class Command:
    pass


class SensorConfig(ModuleConfig):
    sensitivity: float = 1.0


class Sensor(Module):
    config: SensorConfig

    pointcloud: Out[Cloud]
    local_status: Out[Status]
    cmd: In[Command]

    @rpc
    def whoami(self) -> str:
        return self.config.instance_name or "default"

    @rpc
    def get_sensitivity(self) -> float:
        return self.config.sensitivity

    @rpc
    def get_frame_id(self) -> str:
        return self.frame_id


class LocalMapper(Module):
    local_status: In[Status]

    sensor: Sensor

    @rpc
    def sensor_name(self) -> str:
        return self.sensor.whoami()


class Aggregator(Module):
    pointcloud: In[Cloud]


class FleetCommander(Module):
    cmd: Out[Command]


def _fleet_blueprint():
    return autoconnect(
        Aggregator.blueprint(),
        FleetCommander.blueprint(),
        *[
            autoconnect(
                Sensor.blueprint(),
                LocalMapper.blueprint(),
            ).namespace(f"robot{i}", expose={"pointcloud"})
            for i in range(2)
        ],
    ).remappings([(FleetCommander, "cmd", "robot0/cmd")])


def _parsed_config(
    blueprint: Blueprint,
    overrides: dict[str, Any] | None = None,
) -> ParsedBlueprintConfig:
    all_overrides: dict[str, Any] = {"g": {"viewer": "none"}}
    all_overrides.update(overrides or {})
    return BlueprintConfigParser(blueprint).parse(environ={}, overrides=all_overrides)


@pytest.fixture
def fleet_coordinator():
    blueprint = _fleet_blueprint()
    parsed = _parsed_config(blueprint, {"robot0_sensor": {"sensitivity": 2.0}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        yield coordinator
    finally:
        coordinator.stop()


def test_fleet_blueprint(fleet_coordinator: ModuleCoordinator):
    coordinator = fleet_coordinator
    sensor0 = coordinator.get_instance("robot0/sensor")
    sensor1 = coordinator.get_instance("robot1/sensor")
    mapper0 = coordinator.get_instance("robot0/localmapper")
    mapper1 = coordinator.get_instance("robot1/localmapper")
    aggregator = coordinator.get_instance(Aggregator)
    commander = coordinator.get_instance(FleetCommander)

    # A class lookup with two instances is ambiguous.
    with pytest.raises(ValueError, match="Multiple instances"):
        coordinator.get_instance(Sensor)

    # RPC is served per instance, on the instance-name topic.
    assert sensor0.whoami() == "robot0/sensor"
    assert sensor1.whoami() == "robot1/sensor"

    # Namespaced streams get separate topics and exposed streams share one.
    assert (
        sensor0.local_status.transport.topic
        == mapper0.local_status.transport.topic
        == transport_topic("/robot0/local_status")
    )
    assert (
        sensor1.local_status.transport.topic
        == mapper1.local_status.transport.topic
        == transport_topic("/robot1/local_status")
    )
    assert (
        sensor0.pointcloud.transport.topic
        == sensor1.pointcloud.transport.topic
        == aggregator.pointcloud.transport.topic
        == transport_topic("/pointcloud")
    )

    # Direct-class module refs resolve namespace-locally.
    assert mapper0.sensor_name() == "robot0/sensor"
    assert mapper1.sensor_name() == "robot1/sensor"

    # Per-instance config args reach only their instance.
    assert sensor0.get_sensitivity() == 2.0
    assert sensor1.get_sensitivity() == 1.0

    # TF frames carry the namespace.
    assert sensor0.get_frame_id() == "robot0/Sensor"

    # Directed wiring: the shared commander drives only robot0.
    assert (
        commander.cmd.transport.topic
        == sensor0.cmd.transport.topic
        == transport_topic("/robot0/cmd")
    )
    assert sensor1.cmd.transport.topic != sensor0.cmd.transport.topic


def test_restart_preserves_parsed_instance_config(fleet_coordinator: ModuleCoordinator):
    fleet_coordinator.restart_module("robot0/sensor", reload_source=False)

    restarted_sensor = fleet_coordinator.get_instance("robot0/sensor")
    assert restarted_sensor.get_sensitivity() == 2.0


def test_load_blueprint_accepts_parsed_module_config() -> None:
    blueprint = Sensor.blueprint()
    parsed = _parsed_config(blueprint, {"sensor": {"sensitivity": 3.0}})
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    coordinator.start()
    try:
        coordinator.load_blueprint(blueprint, parsed)

        sensor = coordinator.get_instance(Sensor)
        assert sensor.get_sensitivity() == 3.0
    finally:
        coordinator.stop()


def test_load_blueprint_preserves_unrelated_global_config() -> None:
    blueprint = Sensor.blueprint()
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    coordinator = ModuleCoordinator(g=GlobalConfig(viewer="none", robot_ip="10.9.8.7", n_workers=0))
    coordinator.start()
    try:
        coordinator.load_blueprint(blueprint, parsed)

        config = coordinator._global_config
        assert config.viewer == "none"
        assert config.robot_ip == "10.9.8.7"
        assert config.n_workers == 0
    finally:
        coordinator.stop()


def test_fleet_blueprint_config_keys() -> None:
    parsed = BlueprintConfigParser(_fleet_blueprint()).parse(
        environ={}, overrides={"robot0_sensor": {"sensitivity": 5.0}}
    )
    assert set(parsed.module_configs) == {
        "robot0/sensor",
        "robot1/sensor",
        "robot0/localmapper",
        "robot1/localmapper",
        "aggregator",
        "fleetcommander",
    }
    # Escaped roots resolve back to the namespaced instances.
    assert parsed.module_kwargs("robot0/sensor")["sensitivity"] == 5.0


def test_load_blueprint_resolves_existing_provider_in_same_namespace():
    blueprint = autoconnect(
        Sensor.blueprint().namespace("robot0"),
        Sensor.blueprint().namespace("robot1"),
    )
    coordinator = ModuleCoordinator.build(blueprint, _parsed_config(blueprint))
    try:
        coordinator.load_blueprint(LocalMapper.blueprint().namespace("robot0"))

        mapper0 = coordinator.get_instance("robot0/localmapper")
        assert mapper0.sensor_name() == "robot0/sensor"
    finally:
        coordinator.stop()
