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

"""Simulation retains robot connections and replaces only standalone sensors."""

from dataclasses import replace
from pathlib import Path

import pytest

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.composition import attach_simulation, has_simulation_binding
from dimos.sim2.device import SimulatedDeviceFactory
from dimos.sim2.module import SimulationModule
from dimos.sim2.physics import WorldState
from dimos.sim2.sensors.camera.module import SimRGBDCameraModule
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.spec import RobotInstanceSpec


class _ArmConfig(ConnectionConfig):
    source: str = "left"
    pd: bool = True
    robot: RobotInstanceSpec
    frame_id: str = "arm/base"


class _Arm(ConnectionModule):
    """A hardware connection: it knows how it stands in a world."""

    config: _ArmConfig
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[ImuMessage]

    @staticmethod
    def describe_simulated_robot(config: _ArmConfig) -> RobotInstanceSpec:
        return replace(
            config.robot,
            robot_id=config.source,
            frame_id=config.frame_id,
            default_scene="workbench.xml",
        )


class _ScannerConfig(ModuleConfig):
    frame_id: str = "scan"


class _Scanner(Module):
    """A hardware lidar driver: it knows which device the world should attach."""

    config: _ScannerConfig
    lidar: Out[PointCloud2]
    imu: Out[ImuMessage]

    @staticmethod
    def simulated_sensor(config: _ScannerConfig) -> Lidar:
        return Lidar("device", config.frame_id, Fibonacci, model_kwargs={"ray_count": 64})


class _Mapper(Module):
    lidar: In[PointCloud2]


class _Skill(Module):
    _arm: _Arm


class _SensorSkill(Module):
    _scanner: _Scanner


class _StateConsumerConfig(ModuleConfig):
    robot_id: str | None = None


class _StateConsumer(Module):
    config: _StateConsumerConfig
    state: In[WorldState]


class _WorldStateConsumer(Module):
    world_state: In[WorldState]


@pytest.fixture
def hardware(world_config):
    return autoconnect(
        _Arm.blueprint(robot=world_config["robots"]["left"], instance_name="arm"),
        _Scanner.blueprint(instance_name="scanner"),
        _Mapper.blueprint(),
    ).remappings([("arm", "motor_states", "arm/motor_states"), ("scanner", "lidar", "lidar_raw")])


@pytest.fixture
def mujoco(scene_path):
    return GlobalConfig(simulation="mujoco", scene_package=str(scene_path))


def test_declaring_modules_are_recognized():
    assert has_simulation_binding(_Arm) and has_simulation_binding(_Scanner)
    assert not has_simulation_binding(_Mapper)


def test_blueprint_without_twins_is_returned_unchanged(mujoco):
    blueprint = _Mapper.blueprint()
    kwargs = {"_mapper": {}}
    assert attach_simulation(blueprint, kwargs, mujoco) == (blueprint, kwargs)


def test_connection_is_retained_with_private_world_io_and_public_sensor_streams(hardware, mujoco):
    result, kwargs = attach_simulation(
        hardware, {"scanner": {"frame_id": "scan"}, "_mapper": {}}, mujoco
    )

    assert set(kwargs) == {"arm", "_mapper"}
    device = kwargs["arm"]["simulation_device"]
    assert isinstance(device, SimulatedDeviceFactory)
    names = {atom.name: atom.module for atom in result.blueprints}
    world = next(atom for atom in result.blueprints if atom.module is SimulationModule)
    assert names["arm"] is _Arm
    assert names["scanner"] is LidarModule
    assert names["_mapper"] is _Mapper or _Mapper in names.values()
    remaps = result.remapping_map
    assert remaps["arm", "motor_states"] == "arm/motor_states"
    assert device.topics == {
        "motor_command": "_sim2/arm/motor_command",
        "motor_states": "_sim2/arm/motor_states",
        "imu": "_sim2/arm/imu",
    }
    assert world.kwargs["robot_topics"]["left"] == {
        **device.topics,
        "odom": "odom",
        "tf": "tf",
    }
    # The sensor keeps the driver's names; its truth extras are robot-qualified.
    assert remaps["scanner", "lidar"] == "lidar_raw"
    assert remaps["scanner", "imu"] == "imu"
    assert remaps["scanner", "pointcloud"] == "left/scanner/pointcloud"
    # The upstream model remains lazy; its attached sensor uses the driver's frame.
    robot = world.kwargs["robots"]["left"]
    assert robot.frame_id == "arm/base"
    attached = next(s for owner, s in world.kwargs["sensors"] if s.name == "scanner")
    assert isinstance(attached, Lidar) and attached.site == "scan"
    assert world.kwargs["backend"].keywords["scene"] == Path(mujoco.scene_package)


def test_cli_overrides_reach_the_twin(hardware, mujoco):
    result, _ = attach_simulation(hardware, {"arm": {"frame_id": "robot/pelvis"}}, mujoco)
    world = next(atom for atom in result.blueprints if atom.module is SimulationModule)
    assert world.kwargs["robots"]["left"].frame_id == "robot/pelvis"


def test_two_namespaced_connections_share_one_world_and_keep_distinct_clients(world_config, mujoco):
    robot = world_config["robots"]["left"]
    blueprint = autoconnect(
        _Arm.blueprint(robot=robot).namespace("a"),
        _Arm.blueprint(robot=robot).namespace("b"),
    )
    result, kwargs = attach_simulation(blueprint, {}, mujoco)
    worlds = [a for a in result.blueprints if a.module is SimulationModule]
    assert len(worlds) == 1
    assert set(worlds[0].kwargs["robots"]) == {"a/left", "b/left"}
    assert (
        kwargs["a/_arm"]["simulation_device"].topics["motor_command"]
        == "_sim2/a/_arm/motor_command"
    )
    assert (
        kwargs["b/_arm"]["simulation_device"].topics["motor_command"]
        == "_sim2/b/_arm/motor_command"
    )


def test_duplicate_robot_ids_are_rejected_before_world_start(world_config, mujoco):
    blueprint = autoconnect(
        _Arm.blueprint(robot=world_config["robots"]["left"], instance_name="a"),
        _Arm.blueprint(robot=world_config["robots"]["left"], instance_name="b"),
    )
    with pytest.raises(ValueError, match="duplicate simulated robot id"):
        attach_simulation(blueprint, {}, mujoco)


def test_explicit_empty_world_is_populated_instead_of_duplicated(hardware, mujoco):
    blueprint = autoconnect(hardware, SimulationModule.blueprint(snapshot_hz=17))
    result, kwargs = attach_simulation(blueprint, {}, mujoco)
    worlds = [a for a in result.blueprints if a.module is SimulationModule]
    assert len(worlds) == 1
    assert worlds[0].kwargs["snapshot_hz"] == 17
    assert set(kwargs[worlds[0].name]["robots"]) == {"left"}


def test_manual_world_snapshot_cadence_matches_sensor_history(hardware, mujoco, worlds):
    blueprint = autoconnect(hardware, SimulationModule.blueprint(snapshot_hz=17))
    result, kwargs = attach_simulation(blueprint, {}, mujoco)
    atom = next(a for a in result.blueprints if a.module is SimulationModule)
    resolved = {**atom.kwargs, **kwargs[atom.name]}
    world = worlds(
        config={
            "backend": resolved["backend"](),
            **{key: resolved[key] for key in ("robots", "sensors", "snapshot_hz")},
        }
    )
    reader = WorldReader(world.describe())
    reader.push(world.last_snapshot)
    world.publish_snapshot = reader.push
    for _ in range(40):
        world.step()
    assert reader.update()
    end = reader.metadata.sim_time
    assert reader.period == pytest.approx(0.055)
    assert reader.history(end - 2 * reader.period, end)


@pytest.mark.parametrize("manual_world", [False, True])
def test_neutral_state_consumers_resolve_owners_and_share_the_world_without_physics_sensors(
    world_config, mujoco, manual_world
):
    stack = autoconnect(
        _Arm.blueprint(robot=world_config["robots"]["left"]),
        _StateConsumer.blueprint(instance_name="camera", robot_id="left"),
    )
    blueprint = autoconnect(
        stack.namespace("a"),
        stack.namespace("b"),
        _WorldStateConsumer.blueprint(instance_name="viewer"),
        _StateConsumer.blueprint(instance_name="world_camera", robot_id="world"),
        _StateConsumer.blueprint(instance_name="default_world_camera"),
    )
    if manual_world:
        blueprint = autoconnect(
            blueprint,
            SimulationModule.blueprint(instance_name="world").remappings(
                [("world", "world_state", "shared_state")]
            ),
        )
    result, kwargs = attach_simulation(
        blueprint, {}, mujoco.model_copy(update={"simulation": "genesis"})
    )
    world = next(a for a in result.blueprints if a.module is SimulationModule)
    resolved = {**world.kwargs, **kwargs.get(world.name, {})}

    assert resolved["sensors"] == ()
    assert resolved["backend"].keywords["sensors"] == ()
    assert kwargs["a/camera"]["robot_id"] == "a/left"
    assert kwargs["b/camera"]["robot_id"] == "b/left"
    assert kwargs["world_camera"]["robot_id"] is None
    assert kwargs["default_world_camera"]["robot_id"] is None
    topic = "shared_state" if manual_world else "world_state"
    for name, port in (
        ("a/camera", "state"),
        ("b/camera", "state"),
        ("viewer", "world_state"),
        ("world_camera", "state"),
        ("default_world_camera", "state"),
    ):
        assert result.remapping_map[name, port] == topic


def test_namespaced_copies_compile_distinct_models_in_the_collected_world(
    world_config, mujoco, worlds
):
    robot = world_config["robots"]["left"]
    blueprint = autoconnect(
        _Arm.blueprint(robot=replace(robot, position=(1, 0, 0))).namespace("a"),
        _Arm.blueprint(robot=replace(robot, position=(3, 0, 0))).namespace("b"),
    )
    result, _ = attach_simulation(blueprint, {}, mujoco)
    module = next(a for a in result.blueprints if a.module is SimulationModule)
    world = worlds(
        config={
            "backend": module.kwargs["backend"](),
            **{key: module.kwargs[key] for key in ("robots", "sensors")},
        }
    )

    left, right = world.last_state.robots["a/left"], world.last_state.robots["b/left"]
    assert left.base.position[0] == 1
    assert right.base.position[0] == 3
    assert set(world.backend.description.robot_joints["a/left"]).isdisjoint(
        world.backend.description.robot_joints["b/left"]
    )
    assert module.kwargs["robots"]["a/left"].frame_id == "a/arm/base"


def test_explicit_sensor_workers_keep_public_remaps_and_share_the_global_world(
    world_config, mujoco
):
    stack = autoconnect(
        _Arm.blueprint(robot=world_config["robots"]["left"]),
        LidarModule.blueprint(
            sensor=Lidar("scan", "scan", Fibonacci), robot_id="left", instance_name="scanner"
        ),
        SimRGBDCameraModule.blueprint(sensor=Camera("camera", "view"), instance_name="camera"),
    )
    result, kwargs = attach_simulation(
        autoconnect(stack.namespace("a"), stack.namespace("b")), {}, mujoco
    )
    world = next(a for a in result.blueprints if a.module is SimulationModule)
    assert len(result.blueprints) == 7
    assert {(owner, sensor.name) for owner, sensor in world.kwargs["sensors"]} == {
        ("a/left", "scan"),
        ("a/left", "camera"),
        ("b/left", "scan"),
        ("b/left", "camera"),
    }
    assert kwargs["a/scanner"]["robot_id"] == "a/left"
    assert kwargs["b/camera"]["robot_id"] == "b/left"
    assert result.remapping_map["a/scanner", "snapshots"] == "snapshots"
    assert result.remapping_map["b/camera", "observations"] == "observations"
    assert result.remapping_map["a/scanner", "lidar"] == "a/lidar"
    assert result.remapping_map["b/camera", "color_image"] == "b/color_image"


def test_direct_dependency_on_the_retained_connection_is_preserved(world_config, mujoco):
    blueprint = autoconnect(
        _Arm.blueprint(robot=world_config["robots"]["left"], instance_name="arm"),
        _Skill.blueprint(),
    )
    result, _ = attach_simulation(blueprint, {}, mujoco)
    assert next(a for a in result.blueprints if a.name == "arm").module is _Arm
    assert next(a for a in result.blueprints if a.module is _Skill) == blueprint.blueprints[1]


def test_direct_dependency_on_a_replaced_sensor_is_an_error(hardware, mujoco):
    blueprint = autoconnect(hardware, _SensorSkill.blueprint())
    with pytest.raises(ValueError, match="no simulated counterpart"):
        attach_simulation(blueprint, {}, mujoco)


def test_robot_id_must_be_usable_as_a_port_prefix(world_config):
    with pytest.raises(ValueError, match="identifier"):
        replace(world_config["robots"]["left"], robot_id="left//arm")
