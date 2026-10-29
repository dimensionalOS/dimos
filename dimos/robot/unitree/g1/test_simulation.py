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

"""The G1 connection and Mid360 driver declare simulated twins; the GR00T blueprint swaps cleanly."""

from dataclasses import replace

import mujoco
import pytest

from dimos.control.components import make_humanoid_joints
from dimos.core.global_config import GlobalConfig
from dimos.hardware.sensors.lidar.livox.module import MID360_IMU_OFFSET, Mid360, Mid360Config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.robot.unitree.g1.groot_config import G1_GROOT_HOME, G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.simulation import G1_SIM, G1World, g1_groot_instance
from dimos.robot.unitree.g1.wholebody_connection import (
    G1WholeBodyConnection,
    G1WholeBodyConnectionConfig,
)
from dimos.sim2.module import robot_ports
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.spec import WorldConfig
from dimos.sim2.swap import simulate
from dimos.sim2.world.scene import load_scene


@pytest.fixture
def scene(tmp_path):
    path = tmp_path / "scene.xml"
    path.write_text(
        '<mujoco><worldbody><geom name="floor" type="plane" size="5 5 .1"/></worldbody></mujoco>'
    )
    return path


def test_definition_binds_every_coordinator_joint_and_only_the_control_imu():
    assert [joint.name for joint in G1_SIM.joints] == make_humanoid_joints("g1")
    for joint in G1_SIM.joints:
        assert joint.model_name == joint.actuator == joint.name.split("/", 1)[1] + "_joint"
        assert joint.mode == "effort"
    assert [sensor.name for sensor in G1_SIM.sensors] == ["imu"]
    assert len(G1_GROOT_KP) == len(G1_GROOT_KD) == len(G1_GROOT_HOME) == 29


def test_groot_instance_holds_the_home_pose_until_the_policy_commands(scene):
    instance = g1_groot_instance(scene, frame_id="pelvis")
    names = [joint.name for joint in G1_SIM.joints]
    assert instance.initial_positions == dict(zip(names, G1_GROOT_HOME, strict=True))
    assert [instance.hold_gains[name] for name in names] == list(
        zip(G1_GROOT_KP, G1_GROOT_KD, strict=True)
    )
    assert instance.control_imu == "imu"
    assert instance.wait_for_first_command
    assert instance.frame_id == "pelvis"
    assert instance.xyz == pytest.approx((0, 0, 0.793))


def test_connection_declares_its_twin_with_the_hardware_connection_ports(scene):
    twin = G1WholeBodyConnection.simulated_robot(G1WholeBodyConnectionConfig(), scene)
    assert twin.robot_id == "g1"
    assert twin.world is G1World
    assert twin.instance.frame_id == G1WholeBodyConnectionConfig().frame_id == "g1_pelvis"
    assert robot_ports(G1World, ["g1"])["g1"] == {
        "motor_command": "g1_motor_command",
        "motor_states": "g1_motor_states",
        "imu": "g1_imu",
        "odom": "g1_odom",
        "tf": "g1_tf",
    }
    assert G1WholeBodyConnection.default_sim_scene == "logistics.xml"


def test_mid360_declares_a_rolling_device_with_its_imu_at_the_factory_offset():
    lidar = Mid360.simulated_sensor(Mid360Config(frame_id="mid360_link", frequency=10.0))
    assert lidar.site == "mid360_link"
    assert lidar.rate_hz == 10.0
    assert lidar.imu is not None
    assert lidar.imu.site == Mount("mid360_link", xyz=MID360_IMU_OFFSET)
    assert not lidar.truth_outputs
    assert Mid360.simulated_sensor(Mid360Config(enable_imu=False)).imu is None


def test_groot_blueprint_swaps_robot_and_lidar_and_keeps_pointlio(scene):
    from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import unitree_g1_groot_wbc

    hardware = {atom.module for atom in unitree_g1_groot_wbc.active_blueprints}
    assert {G1WholeBodyConnection, Mid360, PointLio} <= hardware

    result, _ = simulate(
        unitree_g1_groot_wbc, {}, GlobalConfig(simulation="mujoco", scene_package=str(scene))
    )

    modules = {atom.name: atom.module for atom in result.active_blueprints}
    assert G1WholeBodyConnection not in modules.values()
    assert modules["mid360"] is LidarModule
    assert PointLio in modules.values()
    world = next(atom for atom in result.blueprints if atom.module is G1World)
    remaps = result.remapping_map
    assert remaps[world.name, "g1_motor_states"] == "g1/motor_states"
    assert remaps[world.name, "g1_motor_command"] == "g1/motor_command"
    assert remaps[world.name, "g1_imu"] == "g1/imu"
    assert remaps[world.name, "g1_odom"] == "odom"
    # The Mid360 driver's wiring into Point-LIO survives untouched.
    assert remaps["mid360", "lidar"] == "lidar_raw"
    assert remaps["mid360", "imu"] == "imu_raw"
    instance = world.kwargs["world"].robots["g1"]
    assert [s.name for s in instance.config.sensors] == ["imu", "mid360"]


@pytest.mark.self_hosted
def test_asset_carries_the_mounts_and_actuators_the_twins_name(scene):
    instance = g1_groot_instance(scene)
    lidar = Mid360.simulated_sensor(Mid360Config(frame_id="mid360_link"))
    # The swap names the device after its driver and the device IMU after it.
    lidar = replace(lidar, name="mid360", imu=replace(lidar.imu, name="mid360_imu"))
    instance = replace(instance, config=instance.config.with_sensor(lidar))
    model = load_scene(WorldConfig(scene, {"g1": instance}))
    assert model.nu == 29
    for kind, name in (
        (mujoco.mjtObj.mjOBJ_SITE, "g1/control_imu"),
        (mujoco.mjtObj.mjOBJ_SITE, "g1/mid360_link"),
        (mujoco.mjtObj.mjOBJ_SITE, "g1/sensor/mid360_imu"),
        (mujoco.mjtObj.mjOBJ_CAMERA, "g1/front_camera"),
        (mujoco.mjtObj.mjOBJ_ACTUATOR, "g1/left_hip_pitch_joint"),
    ):
        assert mujoco.mj_name2id(model, kind, name) >= 0, name
