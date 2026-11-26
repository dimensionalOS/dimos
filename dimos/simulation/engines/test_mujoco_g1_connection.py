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

"""MujocoG1Connection over the sim's shared memory, with the test standing in for the sim."""

import threading
from types import SimpleNamespace
import uuid

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import LocalTransport
from dimos.robot.unitree.g1.config import G1_JOINT_NAMES
import dimos.simulation.engines.mujoco_g1_connection as connection_module
from dimos.simulation.engines.mujoco_g1_connection import MujocoG1Connection
from dimos.simulation.engines.mujoco_shm import ManipShmWriter
from dimos.utils.testing.waiting import wait_until

KP = tuple(float(10 + i) for i in range(29))
KD = tuple(float(i) / 10 for i in range(29))


@pytest.fixture
def sim(monkeypatch):
    """The sim's side of the shared memory: 29 joints, updating."""
    key = f"test_{uuid.uuid4().hex[:10]}"
    monkeypatch.setattr(connection_module, "shm_key_from_path", lambda _: key)
    writer = ManipShmWriter(key)
    writer.signal_ready(num_joints=29)
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


def test_commands_reach_the_sim_with_the_gains_and_readings_come_back_as_g1(sim, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = MujocoG1Connection(kp=KP, kd=KD, deadman_timeout_s=10)
    module._mujoco_sim = SimpleNamespace(scene_path=lambda: "/fake/scene.xml")
    commands, readings, imu = LocalTransport(), LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("joint_state", readings)
    module.set_transport("imu", imu)
    targets = [0.01 * i for i in range(29)]
    try:
        module.start()
        commands.publish(JointState(name=list(G1_JOINT_NAMES), position=targets))
        sent = []
        wait_until(
            lambda: sent.append(sim.read_position_command(29)) or sent[-1] is not None, timeout=2
        )
        assert sent[-1].tolist() == pytest.approx(targets)
        assert sim.read_kp_command(29).tolist() == pytest.approx(KP)
        assert sim.read_kd_command(29).tolist() == pytest.approx(KD)
        assert sim.read_tau_command(29).tolist() == pytest.approx([0.0] * 29)

        sim.write_imu(quaternion=(0.5, 0.1, 0.2, 0.3), gyroscope=(1, 2, 3), accelerometer=(4, 5, 6))
        sim.write_joint_state(positions=targets, velocities=[0.5] * 29, efforts=[0.0] * 29)
        wait_until(lambda: bool(readings.sent) and readings.sent[-1].position == targets, timeout=2)
        assert readings.sent[-1].name == list(G1_JOINT_NAMES)
        assert readings.sent[-1].velocity == pytest.approx([0.5] * 29)
        wait_until(lambda: bool(imu.sent) and imu.sent[-1].orientation.w == 0.5, timeout=2)
        orientation = imu.sent[-1].orientation
        assert (orientation.x, orientation.y, orientation.z) == pytest.approx((0.1, 0.2, 0.3))
    finally:
        module.stop()
