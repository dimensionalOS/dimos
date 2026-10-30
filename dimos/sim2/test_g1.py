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

"""Actual G1 URDF, GR00T balance/walk weights and normal module deployment."""

from dataclasses import replace
import time
import uuid

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import dimos.control.coordinator as coordinator_module
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2 import G1_DESK_WORLD, unitree_g1_sim2
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.sim2.client import SimRobotClient
from dimos.sim2.module import SimModule
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.runtime import Runtime
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.types import JointTargets
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def walking_robot(mocker):
    world = replace(G1_DESK_WORLD, instances=G1_DESK_WORLD.instances[:1])
    backend = MujocoBackend(world)
    runtime = Runtime(backend, backend.layout.drives, control_steps=10)
    atom = next(a for a in unitree_g1_sim2.blueprints if a.module is ControlCoordinator)
    control = ControlCoordinator(**atom.kwargs)
    client = SimRobotClient(tuple(control.config.motor_gains), runtime.submit, imu_frame="g1/imu")
    client.bind(backend.layout)
    mocker.patch.object(Module, "start")  # no transports; worker acceptance below uses real modules
    mocker.patch.object(control.motor_control, "publish", side_effect=client.write_motors)
    control.start()
    try:
        yield runtime, control, client
    finally:
        control.stop()


def cycle(runtime, control, client, count, velocity=None):
    minimum_height = 10.0
    for _ in range(count):
        if velocity is not None:
            control.set_velocity_command(velocity)
        control.receive_feedback(client.feedback(runtime.capture()))
        runtime.advance(10)
        minimum_height = min(minimum_height, runtime.snapshot().poses[0, 2])
    assert minimum_height > 0.68
    assert runtime.rejected == 0
    return runtime.snapshot()


def test_g1_balances_for_ten_minutes_and_resets(walking_robot):
    runtime, control, client = walking_robot
    initial = runtime.snapshot()
    idle = cycle(runtime, control, client, 30000)  # old policy drifted 2.5 m in ten minutes
    assert np.linalg.norm(idle.poses[0, :2] - initial.poses[0, :2]) < 0.3
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "balance"
    reset = runtime.reset()
    assert reset.epoch != initial.epoch
    assert reset.poses == pytest.approx(initial.poses)
    cycle(runtime, control, client, 500)


@pytest.mark.parametrize("axis,speed", [(0, 0.5), (0, -0.5), (1, 0.3), (1, -0.3)])
def test_g1_walks_in_commanded_direction_and_stops(walking_robot, axis, speed):
    runtime, control, client = walking_robot
    before = cycle(runtime, control, client, 100)
    heading = Rotation.from_quat(before.poses[0, 3:].copy()).as_matrix()[:, axis]
    velocity = np.zeros(3)
    velocity[axis] = speed
    walking = cycle(runtime, control, client, 750, Twist(linear=Vector3(*velocity)))
    distance = np.dot(walking.poses[0, :3] - before.poses[0, :3], heading)
    assert distance * np.sign(speed) > abs(speed) * 15 * 0.5
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "walk"
    settled = cycle(runtime, control, client, 100, Twist())
    stopped = cycle(runtime, control, client, 500, Twist())
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "balance"
    assert np.linalg.norm(stopped.poses[0, :2] - settled.poses[0, :2]) < 0.15


@pytest.mark.parametrize("speed", [0.6, -0.6])
def test_g1_turns_in_commanded_direction(walking_robot, speed):
    runtime, control, client = walking_robot
    before = cycle(runtime, control, client, 100)
    after = cycle(runtime, control, client, 200, Twist(angular=Vector3(0, 0, speed)))
    yaw_before = Rotation.from_quat(before.poses[0, 3:].copy()).as_euler("xyz")[2]
    yaw_after = Rotation.from_quat(after.poses[0, 3:].copy()).as_euler("xyz")[2]
    delta = (yaw_after - yaw_before + np.pi) % (2 * np.pi) - np.pi
    assert delta * np.sign(speed) > 1.0
    cycle(runtime, control, client, 100, Twist())
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "balance"


def test_g1_duplicate_feedback_deadman_and_reset(walking_robot, monkeypatch):
    runtime, control, client = walking_robot
    monkeypatch.setattr(coordinator_module.time, "monotonic", lambda: 100.0)
    cycle(runtime, control, client, 100)
    cycle(runtime, control, client, 50, Twist(linear=Vector3(0.2, 0, 0)))
    frame = runtime.capture()
    assert control.receive_feedback(client.feedback(frame))
    count = control.get_tick_count()
    assert not control.receive_feedback(client.feedback(frame))
    assert control.get_tick_count() == count
    runtime.prepare()
    assert runtime.controlled(client.joints)  # retransmission still releases the gate
    runtime.advance(10)
    monkeypatch.setattr(coordinator_module.time, "monotonic", lambda: 100.31)
    cycle(runtime, control, client, 1)
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "balance"
    cycle(runtime, control, client, 10, Twist(linear=Vector3(0.2, 0, 0)))
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "walk"
    runtime.reset()
    cycle(runtime, control, client, 1)
    assert (
        control.task_invoke("groot_wbc", "state_snapshot")["model"] == "balance"
    )  # reset clears a still-live command


def test_control_barrier_requires_current_epoch_tick_and_valid_targets(walking_robot):
    runtime, control, client = walking_robot
    assert not runtime.controlled(client.joints)
    runtime.submit(JointTargets(runtime.epoch, client.joints, (0.0,) * 29))
    runtime.prepare()
    assert runtime.last_error == "paced joints require a feedback tick"
    control.receive_feedback(client.feedback(runtime.capture()))
    runtime.prepare()
    assert runtime.controlled(client.joints)
    runtime.advance(10)
    assert not runtime.controlled(client.joints)
    runtime.submit(JointTargets(runtime.epoch, client.joints, (0.0,) * 29, tick=0))
    runtime.prepare()
    assert not runtime.controlled(client.joints)
    assert runtime.last_error == "stale control tick"
    runtime.submit(JointTargets(runtime.epoch, client.joints, (100.0,) * 29, tick=runtime.tick))
    runtime.prepare()
    assert not runtime.controlled(client.joints)
    assert runtime.last_error.startswith("target outside joint limits")
    before = runtime.snapshot()
    for _ in range(10):
        runtime.prepare()  # waiting for inference must not advance or reset physics
    assert runtime.snapshot().poses == pytest.approx(before.poses)
    assert runtime.tick == before.tick
    runtime.reset()
    runtime.submit(JointTargets(before.epoch, client.joints, (0.0,) * 29, tick=0))
    runtime.prepare()
    assert runtime.last_error == "stale simulation epoch"


@pytest.fixture
def deployed_g1(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = unitree_g1_sim2.global_config(
        zenoh_domain="g1-test-" + uuid.uuid4().hex, zenoh_scout_addr="224.0.0.227:7446"
    )
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        yield coordinator
    finally:
        coordinator.stop()


def test_g1_blueprint_drives_with_sensors_then_resets(deployed_g1, record_property):
    sim = deployed_g1.get_instance(SimModule)
    connection = deployed_g1.get_instance(G1Connection)
    control = deployed_g1.get_instance(ControlCoordinator)
    sensors = deployed_g1.get_instance(SimSensorModule)
    wait_until(lambda: control.get_tick_count() >= 10, timeout=10)
    result = control.execute_trajectory(
        JointTrajectory(
            joint_names=["g1/left_elbow_joint"],
            points=[TrajectoryPoint(positions=[0.8])],
        )
    )
    assert result.status is TrajectoryExecutionStatus.ACCEPTED
    initial = sim.state()
    initial_stats = sensors.statistics()
    wall_start = time.monotonic()
    deadline = wall_start + 30
    while time.monotonic() < deadline:
        state = sim.state()
        stats = sensors.statistics()
        assert not stats["error"]
        assert state.poses[0, 2] > 0.68
        elapsed = state.sim_time - initial.sim_time
        if elapsed < 3:
            connection.move(Twist(linear=Vector3(0.2, 0, 0)))
        else:
            connection.move(Twist())
        if elapsed > 5 and stats["images"] >= 10 and stats["scans"] >= 20:
            break
        time.sleep(0.05)  # poll multiple independent workers with a bounded deadline
    else:
        pytest.fail(
            f"G1 did not progress: {sim.timing()}, sensors={stats}, control={connection.control_status()}"
        )
    assert state.poses[0, 0] - initial.poses[0, 0] > 0.25
    elbow = sim.describe().joints.index("g1/left_elbow_joint")
    assert state.position[elbow] == pytest.approx(0.8, abs=0.08)
    record_property("camera_sim_hz", (stats["images"] - initial_stats["images"]) / elapsed)
    record_property("real_time_factor", elapsed / (time.monotonic() - wall_start))
    record_property("max_compute_ms", control.control_status()["max_compute_ms"])
    assert not sim.timing()["error"]
    assert sim.timing()["rejected_commands"] == 0
    assert connection.control_status()["controller"] == "external motor controller"
    assert control.control_status()["computations"] > 100
    reset = sim.reset()
    assert reset.epoch != state.epoch
    assert reset.poses[0, :3] == pytest.approx([0, 0, 0.793])
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        state = sim.state()
        if state.epoch == reset.epoch and state.sim_time > 1:
            break
        time.sleep(0.05)
    assert state.epoch == reset.epoch and state.sim_time > 1
    assert state.poses[0, 2] > 0.68


def test_g1_arm_trajectory_while_walking_and_reset(walking_robot):
    runtime, control, client = walking_robot
    before = cycle(runtime, control, client, 100)
    joint = "g1/left_elbow_joint"
    index = runtime.backend.layout.joints.index(joint)
    start = control.get_joint_positions()[joint]
    result = control.execute_trajectory(
        JointTrajectory(
            joint_names=[joint],
            points=[
                TrajectoryPoint(positions=[start]),
                TrajectoryPoint(time_from_start=2.0, positions=[0.8]),
            ],
        )
    )
    assert result.status is TrajectoryExecutionStatus.ACCEPTED
    after = cycle(runtime, control, client, 250, Twist(linear=Vector3(0.2, 0, 0)))
    assert after.position[index] == pytest.approx(0.8, abs=0.08)
    assert after.poses[0, 0] > before.poses[0, 0] + 0.4
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "walk"
    assert control.task_invoke("joint_trajectory", "get_state") is TrajectoryState.COMPLETED

    # An unfinished arm trajectory and a live velocity request cannot survive reset.
    start = control.get_joint_positions()[joint]
    result = control.execute_trajectory(
        JointTrajectory(
            joint_names=[joint],
            points=[
                TrajectoryPoint(positions=[start]),
                TrajectoryPoint(time_from_start=10.0, positions=[1.0]),
            ],
        )
    )
    assert result.status is TrajectoryExecutionStatus.ACCEPTED
    cycle(runtime, control, client, 5, Twist(linear=Vector3(0.2, 0, 0)))
    runtime.reset()
    cycle(runtime, control, client, 100)
    assert control.task_invoke("joint_trajectory", "get_state") is TrajectoryState.IDLE
    assert control.task_invoke("groot_wbc", "state_snapshot")["model"] == "balance"
    assert runtime.snapshot().position[index] == pytest.approx(
        next(d.initial for d in G1_DESK_WORLD.instances[0].drives if d.joint == joint), abs=0.08
    )


def test_old_motor_reply_cannot_be_relabelled_after_reset(walking_robot, mocker):
    runtime, control, client = walking_robot
    cycle(runtime, control, client, 1)
    feedback = client.feedback(runtime.capture())
    assert feedback is not None
    sent = mocker.patch.object(control.motor_control, "publish")
    control.receive_feedback(feedback)
    old = sent.call_args.args[0]
    runtime.reset()
    client.feedback(runtime.capture())
    assert not client.write_motors(old)
    runtime.prepare()
    assert not runtime.controlled(client.joints)
    assert runtime.rejected == 0  # rejected before it could reach the physics inbox


def test_g1_resumes_after_estop_without_resetting_physics(walking_robot):
    runtime, control, client = walking_robot
    stopped = cycle(runtime, control, client, 100)
    control.set_estop(True)
    feedback = client.feedback(runtime.capture())
    assert feedback is not None
    assert not control.receive_feedback(feedback)
    runtime.prepare()
    assert not runtime.controlled(client.joints)
    assert runtime.tick == stopped.tick
    control.set_estop(False)
    assert control.receive_feedback(feedback)
    runtime.prepare()
    assert runtime.controlled(client.joints)
    runtime.advance(10)
    after = cycle(runtime, control, client, 100)
    assert after.epoch == stopped.epoch
    assert after.sim_time > stopped.sim_time
