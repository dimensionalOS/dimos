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

from contextlib import ExitStack
from dataclasses import replace
import json
import multiprocessing
import subprocess
import sys
import threading
import time
from typing import ClassVar

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.client import SimRobotClient
from dimos.sim2.discovery import collect_simulation
from dimos.sim2.messages import StreamDescriptor
from dimos.sim2.module import SimulationModule, SimulationSpec
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.spec import RobotConfig, RobotInstance


class _RobotConfig(ModuleConfig):
    robot: RobotConfig
    x: float = 0.0


class _RobotConnection(Module):
    config: _RobotConfig
    _simulation: SimulationSpec
    default_sim_scene: ClassVar[str] = "workbench.xml"

    @staticmethod
    def describe_simulated_robot(config, scene):
        assert scene.is_file()
        return RobotInstance(config.robot, xyz=(config.x, 0, 0.3))

    @rpc
    def start(self):
        super().start()
        descriptor = self._simulation.describe()["robots"][self.config.instance_name]
        self._client = SimRobotClient(StreamDescriptor.from_dict(descriptor), writable=True)
        self._client.connect()

    @rpc
    def read_position(self):
        return self._client.read().values["root_position"].tolist()

    @rpc
    def stop(self):
        client = getattr(self, "_client", None)
        if client is not None:
            client.close()
        super().stop()


def test_collection_uses_resolved_configuration_without_instantiating_driver(world_config, mocker):
    blueprint = _RobotConnection.blueprint(
        robot=world_config.robots["left"].config, instance_name="robot"
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        ["--robot.x=2.5"],
        environ={},
        global_overrides={"simulation": "mujoco", "scene_package": str(world_config.scene)},
    )
    constructor = mocker.patch.object(_RobotConnection, "__init__", side_effect=AssertionError)
    kwargs = {a.name: parsed.module_kwargs(a.name) for a in blueprint.active_blueprints}
    expanded, values = collect_simulation(
        blueprint, kwargs, GlobalConfig(**parsed.global_config_values())
    )
    assert len([a for a in expanded.blueprints if a.module is SimulationModule]) == 1
    assert values[SimulationModule.name]["world"].robots["robot"].xyz == (2.5, 0, 0.3)
    assert expanded.remapping_map["robot_camera", "color_image"] == "robot/camera/color_image"
    assert expanded.remapping_map["robot_lidar", "pointcloud"] == "robot/lidar/pointcloud"
    constructor.assert_not_called()
    parsed.assert_matches(blueprint)
    assert parsed.module_kwargs("robot")["x"] == 2.5
    assert not any(a.module is SimulationModule for a in blueprint.blueprints)


@pytest.mark.parametrize("simulation_mode", ["none", "dimsim"])
def test_collection_does_not_modify_other_backends(world_config, simulation_mode, mocker):
    blueprint = _RobotConnection.blueprint(robot=world_config.robots["left"].config)
    describe = mocker.patch.object(_RobotConnection, "describe_simulated_robot")
    expanded, values = collect_simulation(blueprint, {}, GlobalConfig(simulation=simulation_mode))
    assert expanded is blueprint
    assert values == {}
    describe.assert_not_called()


def test_collection_rejects_two_physics_owners_before_deployment(world_config):
    blueprint = autoconnect(
        _RobotConnection.blueprint(robot=world_config.robots["left"].config),
        SimulationModule.blueprint(world=world_config).namespace("a"),
        SimulationModule.blueprint(world=world_config).namespace("b"),
    )
    with pytest.raises(ValueError, match="one shared SimulationModule"):
        collect_simulation(blueprint, {}, GlobalConfig(simulation="mujoco"))


def test_collection_preserves_explicit_world_and_rejects_duplicate_robot(world_config):
    settings = replace(world_config, robots={}, timestep=0.002, snapshot_hz=200)
    connection = _RobotConnection.blueprint(
        robot=world_config.robots["left"].config, instance_name="left"
    )
    blueprint = autoconnect(connection, SimulationModule.blueprint(world=settings))
    expanded, values = collect_simulation(
        blueprint, {}, GlobalConfig(simulation="mujoco", scene_package="not-used.xml")
    )
    world = values[SimulationModule.name]["world"]
    assert world.scene == world_config.scene
    assert world.timestep == 0.002
    assert world.snapshot_hz == 200
    assert set(world.robots) == {"left"}
    assert len([a for a in expanded.blueprints if a.module is SimulationModule]) == 1
    assert settings.robots == {}
    with pytest.raises(ValueError, match="duplicate simulated robot definition"):
        collect_simulation(
            autoconnect(connection, SimulationModule.blueprint(world=world_config)),
            {},
            GlobalConfig(simulation="mujoco"),
        )


@pytest.mark.native_e2e
def test_two_namespaced_connections_deploy_into_one_world(world_config):
    robot = world_config.robots["left"].config
    robot = replace(robot, sensors=robot.sensors[:1])
    connection = _RobotConnection.blueprint(robot=robot, instance_name="robot")
    blueprint = autoconnect(connection.namespace("left"), connection.namespace("right"))
    parsed = BlueprintConfigParser(blueprint).parse(
        ["--left/robot.x=-1.0", "--right/robot.x=1.0"],
        environ={},
        global_overrides={
            "simulation": "mujoco",
            "scene_package": str(world_config.scene),
            "n_workers": 1,
            "transport": "zenoh",
            "viewer": "none",
        },
    )
    with ExitStack() as stack:
        coordinator = ModuleCoordinator.build(blueprint, parsed)
        stack.callback(coordinator.stop)
        world = coordinator.get_instance(SimulationModule)
        assert set(world.describe()["robots"]) == {"left/robot", "right/robot"}
        assert coordinator.get_instance("left/robot").read_position() == pytest.approx([-1, 0, 0.3])
        assert coordinator.get_instance("right/robot").read_position() == pytest.approx([1, 0, 0.3])
        with pytest.raises(ValueError, match="before build"):
            coordinator.load_blueprint(connection.namespace("late"))


def command():
    return MotorCommandArray(q=[0.3, 425], dq=[0.2, 0], kp=[2, 0], kd=[3, 0], tau=[0.4, 0])


def test_complete_command_and_coherent_feedback(worlds, clients):
    world = worlds()
    client = clients(world, writable=True)
    observed = client.read()
    assert observed.values["position"] == pytest.approx([0.1, 850])
    assert observed.values["imu_quaternion"] == pytest.approx([1, 0, 0, 0])
    sequence = client.write_motors(command(), observed=observed)

    world.step()

    binding = world.robots["left"]
    assert world.data.ctrl[binding.actuators] == pytest.approx([1.4, 0.02])
    feedback = client.read()
    assert feedback.metadata.applied_action_sequence == sequence
    assert feedback.metadata.physics_tick == 1
    assert feedback.metadata.sim_time == pytest.approx(world.config.timestep)
    assert observed.metadata.physics_tick == 0
    assert observed.values["position"] == pytest.approx([0.1, 850])


def test_two_robots_use_independent_channels(world_config, worlds, clients):
    robot = world_config.robots["left"]
    config = replace(
        world_config,
        robots={
            "left": robot,
            "right": replace(robot, xyz=(0, 2, 0.3)),
        },
    )
    world = worlds(config=config)
    left, right = clients(world, "left", writable=True), clients(world, "right", writable=True)
    left.write_motors(command(), observed=left.read())
    world.step()

    assert world.data.ctrl[world.robots["left"].actuators[0]] == pytest.approx(1.4)
    assert world.data.ctrl[world.robots["right"].actuators[0]] == pytest.approx(0)
    assert right.read().values["root_position"] == pytest.approx([0, 2, 0.3])


def test_position_device_moves_and_preserves_gripper_units(worlds, clients):
    world = worlds()
    client = clients(world, writable=True)
    for _ in range(300):
        client.write_positions([0.25, 0], observed=client.read())
        world.step()

    assert client.read().values["position"] == pytest.approx([0.25, 0], abs=0.03)


def test_control_imu_selection_does_not_redefine_physical_robot(world_config, worlds, clients):
    instance = world_config.robots["left"]
    robot = replace(
        instance.config,
        sensors=(
            Imu("torso", Mount("base")),
            Imu("wrist", Mount("base", rpy=(0, 0, 1.5707963267948966))),
        ),
    )
    left = replace(instance, config=robot, control_imu="torso")
    right = replace(instance, config=robot, control_imu="wrist", xyz=(0, 2, 0.3))
    world = worlds(config=replace(world_config, robots={"left": left, "right": right}))

    torso = clients(world, "left").read()
    wrist = clients(world, "right").read()

    assert left.config is right.config
    assert torso.values["imu_quaternion"] == pytest.approx([1, 0, 0, 0])
    assert wrist.values["imu_quaternion"] == pytest.approx([2**-0.5, 0, 0, 2**-0.5])
    assert torso.metadata.physics_tick == wrist.metadata.physics_tick
    assert world.robots["right"].channel.descriptor.control_imu == "wrist"


def test_pd_motor_commands_do_not_require_an_imu(world_config, worlds, clients):
    instance = replace(world_config.robots["left"], control_imu=None)
    world = worlds(config=replace(world_config, robots={"left": instance}))
    client = clients(world, writable=True)
    observed = client.read()
    client.write_motors(command(), observed=observed)
    world.step()

    assert "imu_quaternion" not in observed.values
    assert world.data.ctrl[world.robots["left"].actuators] == pytest.approx([1.4, 0.02])


def test_native_servo_rejects_unsupported_pd_fields(worlds, clients):
    world = worlds()
    client = clients(world, writable=True)
    invalid = command()
    invalid.kp[1] = 1
    with pytest.raises(ValueError, match="native position servos"):
        client.write_motors(invalid, observed=client.read())
    world.step()
    assert client.read().metadata.applied_action_sequence == 0


def test_initial_state_and_watchdog_gains_are_instance_settings(world_config, worlds, clients):
    instance = world_config.robots["left"]
    changed = replace(
        instance,
        initial_positions={"shoulder": 0.6, "gripper": 425},
        hold_gains={"shoulder": (7, 2)},
    )
    world = worlds(config=replace(world_config, robots={"left": instance, "right": changed}))
    reader = clients(world, "right")
    assert reader.read().values["position"] == pytest.approx([0.6, 425])
    assert instance.config is changed.config
    assert world.robots["right"].hold_kp == pytest.approx([7, 0])
    assert world.robots["right"].hold_kd == pytest.approx([2, 0])
    world.reset()
    assert reader.read().values["position"] == pytest.approx([0.6, 425])


@pytest.mark.parametrize(
    "settings",
    [
        {"control_imu": "missing"},
        {"initial_positions": {"missing": 0}},
        {"initial_positions": {"gripper": 851}},
        {"hold_gains": {"shoulder": (-1, 1)}},
    ],
)
def test_invalid_controller_binding_is_rejected_before_world_load(world_config, settings):
    with pytest.raises(ValueError):
        replace(world_config.robots["left"], **settings)


def test_read_only_clients_and_single_command_owner(worlds, clients):
    world = worlds()
    owner = clients(world, writable=True)
    reader = clients(world)
    with pytest.raises(PermissionError, match="read-only"):
        reader.write_motors(command(), observed=reader.read())
    with pytest.raises(RuntimeError, match="already has a command owner"):
        clients(world, writable=True)
    owner.close()
    replacement = clients(world, writable=True)
    assert replacement.write_motors(command(), observed=replacement.read()) > 0


def test_old_observation_cannot_command_after_reset(worlds, clients):
    world = worlds()
    client = clients(world, writable=True)
    old = client.read()
    client.write_motors(command(), observed=old)

    world.reset()

    with pytest.raises(RuntimeError, match="reset"):
        client.write_motors(command(), observed=old)
    world.step()
    assert world.data.ctrl[world.robots["left"].actuators] == pytest.approx([0, 0])
    fresh = client.read()
    assert fresh.metadata.episode_id == old.metadata.episode_id + 1
    assert fresh.metadata.applied_action_sequence == 0
    assert client.write_motors(command(), observed=fresh) > 0


def test_reset_between_client_validation_and_publication_rejects_late_command(
    worlds, clients, mocker
):
    world = worlds()
    client = clients(world, writable=True)
    observed = client.read()
    publish = client._channel.publish_action

    def reset_then_publish(values, metadata, **kwargs):
        world.reset()
        return publish(values, metadata, **kwargs)

    mocker.patch.object(client._channel, "publish_action", side_effect=reset_then_publish)
    client.write_motors(command(), observed=observed)
    world.step()

    assert world.data.ctrl[world.robots["left"].actuators] == pytest.approx([0, 0])
    assert client.read().metadata.applied_action_sequence == 0


def test_stale_feedback_is_not_accepted(worlds, clients, mocker):
    world = worlds()
    client = clients(world, writable=True)
    observed = client.read()
    mocker.patch("dimos.sim2.client.time.monotonic", return_value=time.monotonic() + 1)

    with pytest.raises(TimeoutError, match="stale"):
        client.read()
    with pytest.raises(TimeoutError, match="stale"):
        client.write_motors(command(), observed=observed)


def test_foreign_observation_is_not_a_command_basis(worlds, clients):
    world = worlds()
    writer, reader = clients(world, writable=True), clients(world)
    with pytest.raises(ValueError, match="this client's latest observation"):
        writer.write_motors(command(), observed=reader.read())


@pytest.mark.parametrize("field,value", [("q", [0]), ("tau", [float("nan"), 0]), ("kp", [-1, 0])])
def test_invalid_command_does_not_replace_held_action(worlds, clients, field, value):
    world = worlds()
    client = clients(world, writable=True)
    observed = client.read()
    expected = client.write_motors(command(), observed=observed)
    invalid = command()
    setattr(invalid, field, value)

    with pytest.raises(ValueError):
        client.write_motors(invalid, observed=observed)

    world.step()
    assert client.read().metadata.applied_action_sequence == expected


def test_mismatched_joint_order_is_rejected_at_attach(worlds):
    world = worlds()
    descriptor = world.robots["left"].channel.descriptor
    wrong = replace(descriptor, joint_names=tuple(reversed(descriptor.joint_names)))
    with pytest.raises(ValueError, match="descriptor mismatch"):
        with SimRobotClient(wrong):
            pytest.fail("mismatched joint order attached")


def test_faulted_world_cannot_be_reactivated_by_spawn(worlds, clients):
    world = worlds()
    client = clients(world)
    world.snapshots.set_lifecycle("faulted")
    world.robots["left"].channel.set_lifecycle("faulted")
    with pytest.raises(RuntimeError, match="faulted"):
        client.read()
    with pytest.raises(RuntimeError, match="faulted"):
        world.set_spawn("left", (1, 2, 3), (0, 0, 0))
    assert world.snapshots.lifecycle == "faulted"


def test_close_invalidates_clients_and_restart_has_a_new_generation(worlds, clients):
    world = worlds(sim_id="restart")
    client = clients(world)
    old = world.robots["left"].channel.descriptor
    world.close()
    with pytest.raises(RuntimeError, match="closed"):
        client.read()
    with pytest.raises((RuntimeError, TimeoutError)):
        closed = SimRobotClient(old)
        closed.connect(timeout=0.1)
    replacement = worlds(sim_id="restart")
    new = replacement.robots["left"].channel.descriptor
    assert new.generation != old.generation
    assert new.topic != old.topic
    assert clients(replacement).read().metadata.physics_tick == 0


def _command_in_spawned_process(descriptor, pipe):
    pipe.send("ready")
    pipe.recv()
    with SimRobotClient(StreamDescriptor.from_dict(descriptor), writable=True) as client:
        observation = client.read()
        sequence = client.write_motors(command(), observed=observation)
        pipe.send((sequence, observation.values["position"].tolist()))
        pipe.recv()  # Keep ownership until the parent has applied the command.
    pipe.close()


@pytest.mark.native_e2e
def test_client_controls_world_across_spawned_process(worlds):
    world = worlds()
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(
        target=_command_in_spawned_process,
        args=(world.robots["left"].channel.descriptor.to_dict(), child),
    )
    process.start()
    child.close()
    try:
        assert parent.poll(15), "child did not start"
        assert parent.recv() == "ready"
        world.step()
        parent.send("sample now")
        deadline = time.monotonic() + 15
        while not parent.poll(0.005) and time.monotonic() < deadline:
            world.step()
        assert parent.poll(), "child did not return a command"
        sequence, position = parent.recv()
        deadline = time.monotonic() + 1
        while world.robots["left"].channel.read_action() is None and time.monotonic() < deadline:
            threading.Event().wait(0.001)
        world.step()
        assert position == pytest.approx([0.1, 850])
        assert world.robots["left"].applied_action_sequence == sequence
        assert world.data.ctrl[world.robots["left"].actuators] == pytest.approx(
            [1.4, 0.02], abs=0.05
        )
        parent.send("applied")
        process.join(10)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(5)
        parent.close()
        process.close()


@pytest.mark.native_e2e
def test_independent_reader_exit_does_not_stop_the_world(worlds, clients):
    world = worlds()
    descriptor = world.robots["left"].channel.descriptor
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import json,sys; from dimos.sim2.client import SimRobotClient; "
            "from dimos.sim2.messages import StreamDescriptor; "
            "client=SimRobotClient(StreamDescriptor.from_dict(json.loads(sys.argv[1])), "
            "feedback_timeout=10); client.connect(); "
            "print(client.read().metadata.episode_id); client.close()",
            json.dumps(descriptor.to_dict()),
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    )
    assert result.stdout.strip() == str(world.generation)
    assert "resource_tracker" not in result.stderr
    world.step()
    assert clients(world).read().metadata.physics_tick == 1


def test_pre_reset_command_with_fresh_feedback_is_rejected(worlds, clients):
    world = worlds()
    client = clients(world, writable=True)
    queued = command()
    world.reset()
    with pytest.raises(RuntimeError, match="predates reset"):
        client.write_motors(queued, observed=client.read())
    world.step()
    assert world.robots["left"].applied_action_sequence == 0


def test_expired_command_is_not_retimed_when_forwarded(worlds, clients):
    world = worlds()
    client = clients(world, writable=True)
    queued = command()
    queued.timestamp -= 1
    # It may predate the reset as well, but neither case may reach the actuator.
    with pytest.raises((RuntimeError, TimeoutError)):
        client.write_motors(queued, observed=client.read())
    assert world.robots["left"].channel.read_action() is None
