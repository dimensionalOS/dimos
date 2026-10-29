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

"""Hardware modules that declare a twin are swapped for simulation on the same streams."""

from dataclasses import replace
from pathlib import Path
from typing import ClassVar

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.lidar.models.mid360.model import Mid360
from dimos.sim2.sensors.lidar.models.mid360.pattern import Mid360Pattern
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.spec import RobotConfig, RobotInstance, SimulatedRobot
from dimos.sim2.swap import declares_twin, simulate


class _LeftWorld(SimulationModule):
    left_motor_command: In[MotorCommandArray]
    left_motor_states: Out[JointState]
    left_imu: Out[ImuMessage]
    left_odom: Out[PoseStamped]
    left_tf: Out[TFMessage]


class _ArmConfig(ModuleConfig):
    robot: RobotConfig
    frame_id: str = "arm/base"


class _Arm(Module):
    """A hardware connection: it knows how it stands in a world."""

    config: _ArmConfig
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[ImuMessage]
    default_sim_scene: ClassVar[str] = "workbench.xml"

    @staticmethod
    def simulated_robot(config: _ArmConfig, scene: Path) -> SimulatedRobot:
        assert scene.is_file()
        instance = RobotInstance(
            config.robot,
            xyz=(0, 0, 0.3),
            initial_positions={"shoulder": 0.1, "gripper": 850},
            control_imu="imu",
            frame_id=config.frame_id,
        )
        return SimulatedRobot("left", _LeftWorld, instance)


class _ScannerConfig(ModuleConfig):
    frame_id: str = "scan"
    rolling: bool = False


class _Scanner(Module):
    """A hardware lidar driver: it knows which device the world should attach."""

    config: _ScannerConfig
    lidar: Out[PointCloud2]
    imu: Out[ImuMessage]

    @staticmethod
    def simulated_sensor(config: _ScannerConfig) -> Lidar:
        if config.rolling:
            return Lidar(
                "device",
                config.frame_id,
                Mid360,
                model_kwargs={"downsample": 1000, "noise": False, "dropout": False},
                imu=Imu("device_imu", Mount(config.frame_id, xyz=(0.01, 0.02, -0.04))),
                truth_outputs=False,
            )
        return Lidar("device", config.frame_id, Fibonacci, model_kwargs={"ray_count": 64})


class _Mapper(Module):
    lidar: In[PointCloud2]


class _Skill(Module):
    _arm: _Arm


@pytest.fixture
def hardware(world_config):
    return autoconnect(
        _Arm.blueprint(robot=world_config.robots["left"].config, instance_name="arm"),
        _Scanner.blueprint(instance_name="scanner"),
        _Mapper.blueprint(),
    ).remappings([("arm", "motor_states", "arm/motor_states"), ("scanner", "lidar", "lidar_raw")])


@pytest.fixture
def mujoco(world_config):
    return GlobalConfig(simulation="mujoco", scene_package=str(world_config.scene))


def test_declaring_modules_are_recognized():
    assert declares_twin(_Arm) and declares_twin(_Scanner)
    assert not declares_twin(_Mapper)


def test_blueprint_without_twins_is_returned_unchanged(world_config, mujoco):
    blueprint = _Mapper.blueprint()
    assert simulate(blueprint, {}, mujoco) is blueprint


def test_twins_replace_hardware_modules_on_the_same_streams(hardware, mujoco):
    result = simulate(hardware, {}, mujoco)

    names = {atom.name: atom.module for atom in result.blueprints}
    world = next(atom for atom in result.blueprints if atom.module is _LeftWorld)
    assert "arm" not in names
    assert names["scanner"] is LidarModule
    assert names["_mapper"] is _Mapper or _Mapper in names.values()
    remaps = result.remapping_map
    # The robot's ports take the connection's stream names, remapped or not.
    assert remaps[world.name, "left_motor_states"] == "arm/motor_states"
    assert remaps[world.name, "left_motor_command"] == "motor_command"
    assert remaps[world.name, "left_imu"] == "imu"
    # Truth the hardware never published keeps the plain kind as its stream.
    assert remaps[world.name, "left_odom"] == "odom"
    assert remaps[world.name, "left_tf"] == "tf"
    # The sensor keeps the driver's names; its truth extras are robot-qualified.
    assert remaps["scanner", "lidar"] == "lidar_raw"
    assert remaps["scanner", "imu"] == "imu"
    assert remaps["scanner", "pointcloud"] == "left/scanner/pointcloud"
    # The world holds the robot with the sensor attached at the driver's frame.
    config = world.kwargs["world"]
    instance = config.robots["left"]
    assert instance.frame_id == "arm/base"
    attached = next(s for s in instance.config.sensors if s.name == "scanner")
    assert isinstance(attached, Lidar) and attached.site == "scan"
    assert config.scene == Path(mujoco.scene_package)


def test_cli_overrides_reach_the_twin(hardware, mujoco):
    result = simulate(hardware, {"arm": {"frame_id": "robot/pelvis"}}, mujoco)
    world = next(atom for atom in result.blueprints if atom.module is _LeftWorld)
    assert world.kwargs["world"].robots["left"].frame_id == "robot/pelvis"


def test_a_sensor_with_its_own_imu_gets_both_named_after_the_driver(world_config, mujoco, mocker):
    coefs = [[[1, 0, 0]] * 1] * 4
    mocker.patch(
        "dimos.sim2.sensors.lidar.models.mid360.model._pattern",
        return_value=Mid360Pattern(0, 0, 0, 0, __import__("numpy").asarray(coefs, dtype=float)),
    )
    blueprint = autoconnect(
        _Arm.blueprint(robot=world_config.robots["left"].config, instance_name="arm"),
        _Scanner.blueprint(instance_name="mid360", rolling=True),
    )
    result = simulate(blueprint, {}, mujoco)
    world = next(atom for atom in result.blueprints if atom.module is _LeftWorld)
    attached = next(
        s for s in world.kwargs["world"].robots["left"].config.sensors if s.name == "mid360"
    )
    assert attached.imu.name == "mid360_imu"
    assert attached.imu.site == Mount("scan", xyz=(0.01, 0.02, -0.04))
    assert remaps_for(result, "mid360") == {"lidar": "lidar", "imu": "imu"}


def remaps_for(blueprint, name):
    return {
        port: stream
        for (atom, port), stream in blueprint.remapping_map.items()
        if atom == name and port in ("lidar", "imu")
    }


def test_two_robot_connections_are_refused_for_now(world_config, mujoco):
    robot = world_config.robots["left"].config
    blueprint = autoconnect(
        _Arm.blueprint(robot=robot, instance_name="a"),
        _Arm.blueprint(robot=robot, instance_name="b"),
    )
    with pytest.raises(ValueError, match="exactly one robot connection"):
        simulate(blueprint, {}, mujoco)


def test_direct_dependency_on_a_swapped_module_is_an_error(world_config, mujoco):
    blueprint = autoconnect(
        _Arm.blueprint(robot=world_config.robots["left"].config, instance_name="arm"),
        _Skill.blueprint(),
    )
    with pytest.raises(ValueError, match="no simulated counterpart"):
        simulate(blueprint, {}, mujoco)


def test_robot_id_must_be_usable_as_a_port_prefix(world_config):
    with pytest.raises(ValueError, match="identifier"):
        SimulatedRobot("left/arm", _LeftWorld, replace(world_config.robots["left"]))
