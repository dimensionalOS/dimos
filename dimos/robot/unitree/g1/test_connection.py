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

"""Robot-boundary contracts, plus the actual GR00T sim2 control loop."""

from contextlib import ExitStack
import json
from pathlib import Path
import runpy
import sys
from threading import Event
import time

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.transport import pZenohTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.costmapper import Config as CostMapperConfig, CostMapper
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.connection import G1Config, G1Connection, G1LocomotionSpec
from dimos.robot.unitree.g1.controllers.groot.coordinator import G1GrootCoordinator
from dimos.robot.unitree.g1.model.simulation import G1
from dimos.sim2.client import SimRobotClient
from dimos.sim2.discovery import collect_simulation
from dimos.sim2.messages import StreamDescriptor
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.camera.module import SimRGBDCameraModule
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.world.scene import scene_path
from dimos.spec.utils import spec_annotation_compliance
from dimos.visualization.rerun.bridge import RerunBridgeModule


@pytest.fixture
def load_g1_blueprint(monkeypatch, mocker):
    mocker.patch("dimos.utils.data.get_data", side_effect=AssertionError("unexpected asset load"))

    def load(name, *, simulation="mujoco", viewer="none"):
        for key, value in {
            "simulation": simulation,
            "viewer": viewer,
            "transport": "zenoh",
            "scene_package": None,
        }.items():
            monkeypatch.setattr(global_config, key, value)
        path = Path(__file__).parent / "blueprints/basic" / f"{name}.py"
        return runpy.run_path(str(path))[name]

    return load


def test_g1_does_not_restamp_a_queued_motor_command(mocker):
    connection = G1Connection(controller="groot")
    client = mocker.create_autospec(SimRobotClient, instance=True)
    mocker.patch.object(connection, "_sim", client)
    values = {
        f"{name}/{field}": 0.0
        for name in connection._names
        for field in ("position", "velocity", "kp", "kd", "effort")
    }
    stamp = time.time() - 0.1
    try:
        connection.write(values, source_ts=stamp)
        command = client.write_motors.call_args.args[0]
        assert command.timestamp == stamp
        assert len(command.q) == len(connection._names)
    finally:
        connection.stop()


def _stream_key(blueprint, module, port):
    atom = next(a for a in blueprint.active_blueprints if a.module is module)
    stream = next(s for s in atom.streams if s.name == port)
    return blueprint.remapping_map.get((atom.name, port), port), stream.type


@pytest.mark.parametrize(
    "name",
    ["unitree_g1_groot_wbc", "unitree_g1_basic_sim"],
)
def test_sim_blueprints_wire_lidar_through_mapping_and_navigation(
    load_g1_blueprint, tmp_path, name
):
    blueprint = load_g1_blueprint(name)
    scene = tmp_path / "floor.xml"
    scene.write_text("<mujoco/>")
    parsed = BlueprintConfigParser(blueprint).parse([], environ={})
    costmap_config = CostMapperConfig(**parsed.module_kwargs(CostMapper.name))
    assert costmap_config.initial_safe_radius_meters == 0.0
    expanded, _ = collect_simulation(
        blueprint, {}, GlobalConfig(simulation="mujoco", scene_package=str(scene))
    )

    assert _stream_key(expanded, LidarModule, "pointcloud") == _stream_key(
        expanded, VoxelGridMapper, "lidar"
    )
    assert _stream_key(expanded, VoxelGridMapper, "global_map") == _stream_key(
        expanded, CostMapper, "global_map"
    )
    assert _stream_key(expanded, CostMapper, "global_costmap") == _stream_key(
        expanded, ReplanningAStarPlanner, "global_costmap"
    )
    assert _stream_key(expanded, ReplanningAStarPlanner, "nav_cmd_vel") == _stream_key(
        expanded, MovementManager, "nav_cmd_vel"
    )
    assert _stream_key(expanded, MovementManager, "cmd_vel") == _stream_key(
        expanded, G1Connection, "cmd_vel"
    )
    assert _stream_key(expanded, G1Connection, "odom") == _stream_key(
        expanded, ReplanningAStarPlanner, "odom"
    )
    assert _stream_key(expanded, SimRGBDCameraModule, "color_image")[0] == "color_image"
    assert blueprint.transport_map == {}
    assert sum(a.module is SimulationModule for a in expanded.active_blueprints) == 1


def test_hardware_blueprint_keeps_pointlio_navigation(load_g1_blueprint):
    blueprint = load_g1_blueprint("unitree_g1_groot_wbc", simulation="")
    parsed = BlueprintConfigParser(blueprint).parse([], environ={})
    assert CostMapperConfig(**parsed.module_kwargs(CostMapper.name)).initial_safe_radius_meters > 0

    assert _stream_key(blueprint, PointLio, "lidar") == _stream_key(
        blueprint, RayTracingVoxelMap, "lidar"
    )
    assert _stream_key(blueprint, RayTracingVoxelMap, "global_map") == _stream_key(
        blueprint, CostMapper, "global_map"
    )
    assert _stream_key(blueprint, PointLio, "odometry") == _stream_key(
        blueprint, ReplanningAStarPlanner, "odometry"
    )
    assert VoxelGridMapper not in {a.module for a in blueprint.active_blueprints}


@pytest.mark.parametrize("simulation", ["", "mujoco"])
@pytest.mark.parametrize(
    "name, joint_topic",
    [
        ("unitree_g1_groot_wbc", "g1_joints"),
        ("unitree_g1_basic_sim", "g1_joints"),
    ],
)
def test_viewer_uses_a_joint_stream_published_by_its_stack(
    load_g1_blueprint, name, joint_topic, simulation
):
    blueprint = load_g1_blueprint(name, simulation=simulation, viewer="rerun")
    viewer = next(a for a in blueprint.active_blueprints if a.module is RerunBridgeModule)
    outputs = {
        blueprint.remapping_map.get((a.name, s.name), s.name)
        for a in blueprint.active_blueprints
        for s in a.streams
        if s.direction == "out"
    }

    assert joint_topic in outputs
    assert f"world/{joint_topic}" in viewer.kwargs["visual_override"]
    assert "world/g1/joints" not in viewer.kwargs["visual_override"]


def test_basic_sim_does_not_import_the_groot_application(load_g1_blueprint, monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc",
        None,
    )
    blueprint = load_g1_blueprint("unitree_g1_basic_sim")
    connections = [a for a in blueprint.active_blueprints if a.module is G1Connection]

    assert len(connections) == 1
    assert connections[0].kwargs["controller"] == "groot"


@pytest.mark.parametrize("simulation", ["", "mujoco"])
def test_groot_composition_preserves_hardware_activation_safety(load_g1_blueprint, simulation):
    blueprint = load_g1_blueprint("unitree_g1_groot_wbc", simulation=simulation)
    coordinator = next(a for a in blueprint.active_blueprints if a.module is G1GrootCoordinator)
    task = next(t for t in coordinator.kwargs["tasks"] if t.name == "groot_wbc")

    assert task.params["auto_arm"] is bool(simulation)
    assert task.params["auto_dry_run"] is not bool(simulation)
    assert task.params["default_ramp_seconds"] == (0.0 if simulation else 10.0)
    assert coordinator.kwargs["tick_rate"] / task.params["decimation"] == 50.0


@pytest.fixture
def robot(mocker):
    robot = G1Connection(controller="groot")
    mocker.patch.object(robot, "_locomotion", mocker.Mock(spec=G1LocomotionSpec))
    yield robot
    robot.stop()


def test_custom_controller_owns_motors_not_a_second_twist_controller(robot):
    contract = robot.describe_control()
    assert list(contract.command_ports) == ["motor_command"]
    assert list(contract.description.joint_names()) == [j.name for j in G1.joints]
    assert "odom" not in contract.reading_ports.values()
    assert {r.name for r in contract.description.resources} == {
        *(j.name.split("/", 1)[1] for j in G1.joints),
        "imu",
    }


def test_stock_connection_describes_only_available_stock_control():
    with ExitStack() as stack:
        robot = G1Connection(controller="stock")
        stack.callback(robot.stop)
        contract = robot.describe_control()
        assert list(contract.command_ports) == ["cmd_vel"]
        assert contract.reading_ports == {}
        assert [r.name for r in contract.description.resources] == ["base"]


def test_public_move_routes_intent_without_exposing_motor_io(robot):
    robot.move(Twist(Vector3(0.4, -0.1, 0), Vector3(0, 0, 0.2)))
    robot._locomotion.set_velocity_command.assert_called_once_with(0.4, -0.1, 0.2)


def test_stock_actions_are_explicitly_unavailable_under_custom_policy(robot):
    with pytest.raises(NotImplementedError, match="stock SDK actions"):
        robot.publish_request("sport", {"api_id": 1004})


def test_policy_pd_targets_are_not_kinematic_position_limits(robot):
    limit = robot.describe_control().description.limits["g1/left_ankle_pitch/position"]
    assert limit.lo is None and limit.hi is None
    assert robot.describe_control().description.limits["g1/left_ankle_pitch/effort"].hi > 0


def test_estop_is_latched_at_robot_boundary(robot):
    robot.set_estop(True)
    with pytest.raises(RuntimeError, match="latched"):
        robot.move(Twist())
    with pytest.raises(RuntimeError, match="latched"):
        robot.set_estop(False)
    robot._locomotion.set_estop.assert_called_once_with(True)


def test_groot_host_implements_intent_contract():
    assert spec_annotation_compliance(G1GrootCoordinator, G1LocomotionSpec)


def test_spawn_support_is_not_applied_twice(tmp_path):
    scene = tmp_path / "floor.xml"
    scene.write_text("<mujoco/>")
    config = G1Config(controller="groot", g=GlobalConfig(simulation="mujoco"))
    instance = G1Connection.describe_simulated_robot(config, scene)
    assert instance.xyz == (0.0, 0.0, 0.793)


def test_groot_startup_settings_do_not_change_physical_robot(tmp_path):
    scene = tmp_path / "floor.xml"
    scene.write_text("<mujoco/>")
    groot = G1Connection.describe_simulated_robot(G1Config(controller="groot"), scene)
    assert groot.config == G1
    assert groot.hold_gains
    assert groot.control_imu == "imu"
    assert all(j.kp == j.kd == 0 for j in groot.config.joints)


@pytest.fixture
def walking_stack(monkeypatch):
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(global_config, "viewer", "none")
    monkeypatch.setattr(global_config, "scene_package", None)
    module = Path(__file__).parent / "blueprints/basic/unitree_g1_groot_wbc.py"
    blueprint = runpy.run_path(str(module))["unitree_g1_groot_wbc"]
    parsed = BlueprintConfigParser(blueprint).parse(
        [],
        environ={},
        global_overrides={
            "simulation": "mujoco",
            "scene_package": str(scene_path(None, "logistics.xml")),
            "transport": "zenoh",
            "viewer": "none",
            "n_workers": 2,
        },
    )
    with ExitStack() as stack:
        app = ModuleCoordinator.build(blueprint, parsed)
        stack.callback(app.stop)
        yield app


@pytest.mark.self_hosted
def test_real_policy_moves_robot_through_public_connection(walking_stack, record_property):
    app = walking_stack
    world = app.get_instance(SimulationModule)
    robot = app.get_instance(G1Connection)
    mapped = Event()
    commands, latency, image_ages, scans = {}, [], [], []
    last_applied = 0
    descriptor = StreamDescriptor.from_dict(world.describe()["robots"][G1Connection.name])

    def on_command(frame):
        commands[frame.metadata.sequence] = time.monotonic() - (time.time() - frame.source_ts)

    def on_feedback(frame):
        nonlocal last_applied
        sequence = frame.metadata.applied_action_sequence
        if sequence != last_applied and sequence in commands:
            latency.append(float(frame.values["monotonic_time"][0]) - commands.pop(sequence))
            last_applied = sequence

    def on_costmap(grid):
        if grid.info.width > 0 and grid.info.height > 0:
            mapped.set()

    with ExitStack() as stack:
        for suffix, callback in (("command", on_command), ("state", on_feedback)):
            transport = pZenohTransport(f"{descriptor.topic}/{suffix}")
            stack.callback(transport.stop)
            stack.callback(transport.subscribe(callback))
        stack.callback(
            app.transports[("color_image", Image)].subscribe(
                lambda image: image_ages.append(time.time() - image.ts)
            )
        )
        stack.callback(
            app.transports[("pointcloud", PointCloud2)].subscribe(
                lambda cloud: scans.append(cloud.ts)
            )
        )
        stack.callback(app.transports[("global_costmap", OccupancyGrid)].subscribe(on_costmap))
        stack.callback(robot.move, Twist())
        assert mapped.wait(15), "simulated lidar never produced a navigation costmap"
        start = world.status()
        image_ages.clear()
        scans.clear()
        latency.clear()
        started = time.monotonic()
        positions = []
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            robot.move(Twist(Vector3(0.35, 0, 0)))
            positions.append(world.status()["robots"][G1Connection.name])
            time.sleep(0.1)  # The commanded input rate, not a readiness wait.
    end = world.status()
    elapsed = time.monotonic() - started
    samples = np.array(positions)
    evidence = {
        "elapsed_s": elapsed,
        "realtime_factor": (end["sim_time"] - start["sim_time"]) / elapsed,
        "physics_hz": (end["steps"] - start["steps"]) / elapsed,
        "command_to_poststep_p99_ms": float(np.percentile(latency, 99) * 1000) if latency else None,
        "command_samples": len(latency),
        "camera_hz": len(image_ages) / elapsed,
        "camera_age_p99_ms": float(np.percentile(image_ages, 99) * 1000) if image_ages else None,
        "lidar_hz": len(scans) / elapsed,
        "forward_m": float(samples[-1, 0] - samples[0, 0]),
    }
    record_property("sim2_zenoh_loaded", json.dumps(evidence))
    assert len(latency) > 100 and evidence["command_to_poststep_p99_ms"] < 25, evidence
    assert end["error"] is None
    assert end["steps"] - start["steps"] > 1200
    assert np.isfinite(samples).all()
    assert samples[:, 2].min() > 0.5
    assert samples[-1, 0] - samples[0, 0] > 0.8
    assert robot.status().last_error is None
