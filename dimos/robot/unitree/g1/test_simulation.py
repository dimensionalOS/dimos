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

"""G1 model composition preserves the hardware motor order and original dynamics."""

from contextlib import closing
from functools import partial
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
from robosuite.models.robots.robot_model import RobotModel

from dimos.control.components import make_humanoid_joints
from dimos.core.global_config import GlobalConfig
from dimos.hardware.sensors.lidar.livox.module import MID360_IMU_OFFSET, Mid360, Mid360Config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import unitree_g1_groot_wbc
from dimos.robot.unitree.g1.groot_config import G1_GROOT_HOME, G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.simulation import G1Model, g1_model
from dimos.robot.unitree.g1.wholebody_connection import (
    G1WholeBodyConnection,
    G1WholeBodyConnectionConfig,
)
from dimos.sim2.composition import attach_simulation
from dimos.sim2.environment import MotorScene
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.camera.unreal.assets import export_scene
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.world.runtime import SimulationRuntime
from dimos.sim2.world.scene import scene_path
from dimos.sim2.world.scene_types import SceneDescription
from dimos.utils.data import LfsPath


@pytest.fixture
def model(tmp_path, mocker):
    # Model parsing is local; these tests never fetch robot mesh assets.
    mocker.patch("dimos.robot.unitree.g1.simulation.LfsPath", return_value=tmp_path)
    return G1Model(idn="g1")


@pytest.fixture
def scene(tmp_path):
    path = tmp_path / "scene.xml"
    path.write_text(
        '<mujoco><worldbody><geom name="floor" type="plane" size="5 5 .1"/></worldbody></mujoco>'
    )
    return path


def test_upstream_model_actuator_order_matches_every_hardware_motor(model):
    assert isinstance(model, RobotModel)
    assert model.public_motor_names == tuple(make_humanoid_joints("g1"))
    expected = model.correct_naming(
        [name.split("/", 1)[1] + "_joint" for name in model.public_motor_names]
    )
    assert model.actuators == expected
    assert [motor.attrib["joint"] for motor in model.actuator] == expected


def test_control_imu_is_a_native_sensor_at_the_authored_site(model):
    assert model.sensors == model.correct_naming(
        [
            "imu-angular-velocity",
            "imu-linear-acceleration",
            "control_imu_gyro",
            "control_imu_accel",
            "control_imu_quat",
        ]
    )
    gyro = model.sensor.find(f"gyro[@name='{model.correct_naming('control_imu_gyro')}']")
    accel = model.sensor.find(f"accelerometer[@name='{model.correct_naming('control_imu_accel')}']")
    assert gyro.attrib["site"] == model.correct_naming("control_imu")
    assert accel.attrib["site"] == model.correct_naming("control_imu")
    assert model.sensor.find("framequat").attrib["objname"] == model.correct_naming("control_imu")


def test_robosuite_loading_preserves_anonymous_joint_defaults(model):
    source = ET.parse(Path(__file__).parent / "assets" / "g1_29dof.xml").getroot()
    defaults = source.find("./default/joint").attrib
    originals = {
        model.correct_naming(joint.attrib["name"]): {**defaults, **joint.attrib}
        for joint in source.findall(".//worldbody//joint")
    }
    for joint in model.worldbody.iter("joint"):
        expected = originals[joint.attrib["name"]]
        for field in ("damping", "armature", "frictionloss"):
            assert float(joint.attrib[field]) == float(expected[field])


def test_upstream_model_preserves_visual_and_contact_geom_colors(model):
    source = ET.parse(Path(__file__).parent / "assets" / "g1_29dof.xml").getroot()
    for body in source.findall(".//worldbody//body"):
        imported = model.worldbody.find(
            f".//body[@name='{model.correct_naming(body.attrib['name'])}']"
        )
        assert [geom.get("rgba") for geom in imported.findall("geom")] == [
            geom.get("rgba") for geom in body.findall("geom")
        ]


def test_contact_geom_material_reference_survives_robot_prefixing(model, mocker):
    original_parse = ET.parse

    def painted_source(path):
        tree = original_parse(path)
        source = tree.getroot()
        ET.SubElement(source.find("asset"), "material", name="paint", rgba=".2 .4 .6 1")
        # Give both the visual and contact instances an authored material.
        for geom in source.findall(".//worldbody//geom[@mesh='pelvis_contour_link']"):
            geom.set("material", "paint")
            geom.attrib.pop("rgba")
        return tree

    mocker.patch.object(ET, "parse", side_effect=painted_source)
    painted = G1Model(idn="painted")
    copies = painted.worldbody.findall(
        f".//geom[@mesh='{painted.correct_naming('pelvis_contour_link')}']"
    )

    assert len(copies) == 2
    assert [geom.get("material") for geom in copies] == ["robotpainted_paint", "robotpainted_paint"]
    assert [geom.get("rgba") for geom in copies] == [None, None]
    assert painted.asset.find("material[@name='robotpainted_paint']") is not None


def test_groot_startup_uses_existing_home_and_motor_gains(model):
    assert model.init_qpos.tolist() == list(G1_GROOT_HOME)
    assert model.startup_kp == tuple(G1_GROOT_KP)
    assert model.startup_kd == tuple(G1_GROOT_KD)
    assert model.wait_for_first_command
    # Each caller owns its initial joint array.
    model.init_qpos[:] = 99
    assert model.init_qpos.tolist() == list(G1_GROOT_HOME)


def test_factory_places_the_robot_above_an_authored_support(model, scene):
    scene.with_suffix(".json").write_text(
        SceneDescription(id="platform", spawns={"default": Pose(1, 2, 0.4)}).model_dump_json()
    )

    placed = g1_model(scene)

    body = placed.worldbody.find("body")
    assert np.fromstring(body.attrib["pos"], sep=" ") == pytest.approx([1, 2, 1.193])


def test_connection_declares_model_lazily_and_preserves_public_frame(scene, mocker):
    factory = mocker.patch("dimos.robot.unitree.g1.simulation.g1_model")

    twin = G1WholeBodyConnection.simulated_robot(
        G1WholeBodyConnectionConfig(frame_id="pelvis"), scene
    )

    factory.assert_not_called()
    assert twin.joint_names == tuple(make_humanoid_joints("g1"))
    assert twin.frame_id == "pelvis"
    assert twin.control_imu == "control_imu"
    assert twin.startup_kp == tuple(G1_GROOT_KP)
    assert twin.startup_kd == tuple(G1_GROOT_KD)


def test_mid360_declares_a_rolling_device_with_its_imu_at_the_factory_offset():
    lidar = Mid360.simulated_sensor(Mid360Config(frame_id="mid360_link", frequency=10.0))
    assert lidar.site == "mid360_link"
    assert lidar.rate_hz == 10.0
    assert lidar.imu is not None
    assert lidar.imu.site == Mount("mid360_link", xyz=MID360_IMU_OFFSET)
    assert not lidar.truth_outputs
    assert Mid360.simulated_sensor(Mid360Config(enable_imu=False)).imu is None


def test_groot_blueprint_keeps_connection_and_pointlio_wiring(scene):
    result, module_kwargs = attach_simulation(
        unitree_g1_groot_wbc, {}, GlobalConfig(simulation="mujoco", scene_package=str(scene))
    )
    modules = {atom.name: atom.module for atom in result.active_blueprints}
    connection = next(a for a in result.active_blueprints if a.module is G1WholeBodyConnection)
    original = next(
        a for a in unitree_g1_groot_wbc.active_blueprints if a.module is G1WholeBodyConnection
    )
    world = next(a for a in result.blueprints if a.module is SimulationModule)

    assert connection.name == original.name
    assert module_kwargs[connection.name]["simulation_device"] is not None
    assert modules["mid360"] is LidarModule
    assert PointLio in modules.values()
    for port in ("motor_states", "motor_command", "imu"):
        assert result.remapping_map[connection.name, port] == f"g1/{port}"
        assert world.kwargs["robot_topics"]["g1"][port] == f"_sim2/{connection.name}/{port}"
    assert result.remapping_map["mid360", "lidar"] == "lidar_raw"
    assert result.remapping_map["mid360", "imu"] == "imu_raw"
    assert [(owner, sensor.name) for owner, sensor in world.kwargs["sensors"]] == [("g1", "mid360")]


@pytest.mark.self_hosted
def test_compiled_robot_preserves_the_native_mjcf_physics_and_control_imu():
    path = Path(__file__).parent / "assets" / "g1_29dof.xml"
    native_spec = mujoco.MjSpec.from_file(str(path))
    native_spec.meshdir = str(LfsPath("g1_urdf/meshes"))
    native = native_spec.compile()
    upstream = G1Model()
    compiled = upstream.get_model()

    for field in (
        "body_mass",
        "body_inertia",
        "dof_damping",
        "dof_armature",
        "dof_frictionloss",
        "jnt_range",
        "geom_type",
        "geom_size",
        "geom_pos",
        "geom_quat",
        "geom_contype",
        "geom_conaffinity",
        "geom_group",
        "geom_rgba",
        "geom_matid",
        "geom_friction",
        "actuator_gainprm",
        "actuator_biasprm",
        "actuator_gear",
        "actuator_ctrlrange",
    ):
        np.testing.assert_allclose(getattr(compiled, field), getattr(native, field), err_msg=field)
    assert compiled.nu == 29
    assert compiled.sensor(upstream.correct_naming("control_imu_gyro")).dim == 3
    assert compiled.sensor(upstream.correct_naming("control_imu_accel")).dim == 3
    assert compiled.sensor(upstream.correct_naming("control_imu_quat")).dim == 4


@pytest.mark.self_hosted
def test_g1_commands_and_coherent_feedback_through_the_upstream_environment(scene, mocker):
    twin = G1WholeBodyConnection.simulated_robot(G1WholeBodyConnectionConfig(), scene)
    factory = partial(MotorScene, scene=scene, models={"g1": twin.model})
    with SimulationRuntime(factory, robots={"g1": twin}) as world:
        feedback = mocker.patch.object(world, "publish_feedback")
        world.step()
        assert world.tick == 0
        command = MotorCommandArray(
            q=list(G1_GROOT_HOME), kp=list(G1_GROOT_KP), kd=list(G1_GROOT_KD)
        )

        assert world.accept_command("g1", command)
        world.step()

        _, joints, imu, odom = feedback.call_args.args
        assert world.tick == 1
        assert joints.name == list(twin.joint_names)
        assert joints.frame_id == imu.frame_id == "g1_pelvis"
        assert joints.ts == imu.ts == odom.ts
        assert np.isfinite(joints.position).all()
        assert world.data.time == pytest.approx(0.005)


@pytest.mark.self_hosted
def test_g1_hard_reset_reloads_sensor_model_and_preserves_mid360_imu(scene):
    twin = G1WholeBodyConnection.simulated_robot(G1WholeBodyConnectionConfig(), scene)
    lidar = Mid360.simulated_sensor(Mid360Config(frame_id="mid360_link"))
    sensors = (("g1", lidar),)
    factory = partial(
        MotorScene, scene=scene, models={"g1": twin.model}, sensors=sensors, hard_reset=True
    )
    with SimulationRuntime(factory, robots={"g1": twin}, sensors=sensors) as world:
        reader = WorldReader(world.describe())
        reader.push(world.last_snapshot)
        assert reader.update()
        original_model = reader.model
        original_episode = reader.episode
        prefix = reader.model_prefix("g1")
        imu_site = reader.model.site(prefix + lidar.imu.model_name).id
        lidar_site = reader.model.site(prefix + lidar.model_name).id
        assert reader.model.site_bodyid[imu_site] == reader.model.site_bodyid[lidar_site]
        offset = reader.data.site_xmat[lidar_site].reshape(3, 3).T @ (
            reader.data.site_xpos[imu_site] - reader.data.site_xpos[lidar_site]
        )
        assert offset == pytest.approx(MID360_IMU_OFFSET)

        world.reset()
        reader.push(world.last_snapshot)
        assert reader.update()

        assert reader.model is not original_model
        assert reader.episode == original_episode + 1
        assert world.waiting_for_control == {"g1"}
        assert reader.model.sensor(prefix + "sensor/imu/gyro").dim == 3
        assert reader.model.sensor(prefix + "sensor/imu/accel").dim == 3
        assert reader.model.sensor(prefix + "control_imu_quat").dim == 4
        assert np.isfinite(reader.data.sensordata).all()


@pytest.mark.self_hosted
def test_full_g1_logistics_world_exports_after_reset_without_injected_ui_targets():
    path = scene_path(None, "logistics.xml")
    with closing(MotorScene(path, {"g1": partial(g1_model, path)})) as environment:
        environment.reset()
        model, data = environment.sim.model._model, environment.sim.data._data
        manifest = export_scene(model, data)

        assert model.nu == 29
        assert {geom["id"] for geom in manifest["geoms"]} == set(range(model.ngeom))
        assert any(model.geom(geom["id"]).name.startswith("robotg1_") for geom in manifest["geoms"])
        assert {model.geom(i).name for i in range(model.ngeom)}.isdisjoint(
            {
                "left_eef_target_box",
                "left_eef_target_sphere",
                "right_eef_target_box",
                "right_eef_target_sphere",
            }
        )
