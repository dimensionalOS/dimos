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

from types import MappingProxyType

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out

_BUILD_WITHOUT_RERUN = MappingProxyType(
    {
        "g": {"viewer": "none"},
    }
)


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


@pytest.fixture
def fleet_coordinator():
    args = dict(_BUILD_WITHOUT_RERUN)
    args["robot0_sensor"] = {"sensitivity": 2.0}

    coordinator = ModuleCoordinator.build(_fleet_blueprint(), args)
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
        == "/robot0/local_status"
    )
    assert (
        sensor1.local_status.transport.topic
        == mapper1.local_status.transport.topic
        == "/robot1/local_status"
    )
    assert (
        sensor0.pointcloud.transport.topic
        == sensor1.pointcloud.transport.topic
        == aggregator.pointcloud.transport.topic
        == "/pointcloud"
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
    assert commander.cmd.transport.topic == sensor0.cmd.transport.topic == "/robot0/cmd"
    assert sensor1.cmd.transport.topic != sensor0.cmd.transport.topic


def test_fleet_blueprint_config_keys():
    config = _fleet_blueprint().config()
    assert {
        "robot0_sensor",
        "robot1_sensor",
        "robot0_localmapper",
        "robot1_localmapper",
        "aggregator",
        "fleetcommander",
        "g",
    } == set(config.model_fields.keys())


def test_load_blueprint_resolves_existing_provider_in_same_namespace():
    coordinator = ModuleCoordinator.build(
        autoconnect(
            Sensor.blueprint().namespace("robot0"),
            Sensor.blueprint().namespace("robot1"),
        ),
        dict(_BUILD_WITHOUT_RERUN),
    )
    try:
        coordinator.load_blueprint(LocalMapper.blueprint().namespace("robot0"))

        mapper0 = coordinator.get_instance("robot0/localmapper")
        assert mapper0.sensor_name() == "robot0/sensor"
    finally:
        coordinator.stop()
