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

"""MujocoArmConnection over the sim's shared memory, with the test standing in for the sim."""

import threading
from types import SimpleNamespace
import uuid

import pytest

from dimos.control._control_test_helpers import fake_arm_model
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import LocalTransport
import dimos.simulation.engines.mujoco_arm_connection as connection_module
from dimos.simulation.engines.mujoco_arm_connection import MujocoArmConnection
from dimos.simulation.engines.mujoco_shm import ManipShmWriter
from dimos.utils.testing.waiting import wait_until

# The xArm Gripper's scene joint: 0 rad open, 0.85 rad closed.
GRIPPER_RANGE = (0.0, 0.85)


@pytest.fixture
def sim(monkeypatch):
    """The sim's side of the shared memory: two arm joints and a gripper, updating."""
    key = f"test_{uuid.uuid4().hex[:10]}"
    monkeypatch.setattr(connection_module, "shm_key_from_path", lambda _: key)
    writer = ManipShmWriter(key)
    writer.write_gripper_range(*GRIPPER_RANGE)
    writer.signal_ready(num_joints=3, arm_joints=2)
    stop = threading.Event()

    def step() -> None:
        while not stop.wait(0.01):
            writer._mark_joint_state()

    thread = threading.Thread(target=step, daemon=True)
    thread.start()
    yield writer
    stop.set()
    thread.join()
    writer.cleanup()


@pytest.fixture
def rig(sim, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = MujocoArmConnection(
        model=fake_arm_model(("joint1", "joint2", "drive_joint")),
        gripper_joint="drive_joint",
        deadman_timeout_s=10,
    )
    module._mujoco_sim = SimpleNamespace(scene_path=lambda: "/fake/scene.xml")
    commands, readings = LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("velocity_command", LocalTransport())
    module.set_transport("joint_state", readings)
    try:
        module.start()
        yield module, commands, readings
    finally:
        module.stop()


def test_a_position_command_reaches_the_sim_and_its_joints_come_back(sim, rig):
    module, commands, readings = rig
    commands.publish(
        JointState(name=["arm/joint1", "arm/joint2", "arm/drive_joint"], position=[0.3, -0.2, 0.85])
    )
    arm = []
    wait_until(lambda: arm.append(sim.read_position_command(2)) or arm[-1] is not None, timeout=2)
    assert arm[-1].tolist() == pytest.approx([0.3, -0.2])
    # The sim's gripper command runs the other way to its joint: 0 closes it.
    assert sim.read_gripper_command() == pytest.approx(0.0)

    sim.write_joint_state(positions=[0.3, -0.2, 0.0], velocities=[0.1, 0.2, 0.0], efforts=[0.0] * 3)
    sim.write_gripper_state(0.6)
    wait_until(lambda: bool(readings.sent) and readings.sent[-1].position[0] == 0.3, timeout=2)
    reading = readings.sent[-1]
    assert reading.name == ["arm/joint1", "arm/joint2", "arm/drive_joint"]
    assert reading.position == pytest.approx([0.3, -0.2, 0.6])
    assert reading.velocity == pytest.approx([0.1, 0.2, 0.0])
