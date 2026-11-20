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

from collections.abc import Iterator
import json
import math
import socket
import threading
import time
from typing import Any

import msgpack  # type: ignore[import-untyped]
import numpy as np
import pytest
import zenoh

from dimos.hardware.manipulators.robolab.adapter import RoboLabAdapter
from dimos.hardware.manipulators.spec import ControlMode
from dimos.simulation.robolab.protocol import CameraFrame, RobotState

HOME = [0.0, -0.628, 0.0, -2.513, 0.0, 1.885, 0.0]


def _free_endpoint() -> str:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return f"tcp/127.0.0.1:{s.getsockname()[1]}"


class FakeRunner:
    """RoboLab's runner as far as the bridge sees it: it sends state and records commands."""

    def __init__(self, endpoint: str) -> None:
        config = zenoh.Config()
        config.insert_json5("scouting/multicast/enabled", "false")
        config.insert_json5("listen/endpoints", json.dumps([endpoint]))
        self.session = zenoh.open(config)
        self.commands: list[dict[str, Any]] = []
        self.session.declare_subscriber("robolab/cmd", self._on_cmd)
        self._state = self.session.declare_publisher("robolab/state")
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._send_state, daemon=True)
        self._thread.start()

    def _on_cmd(self, sample: zenoh.Sample) -> None:
        self.commands.append(msgpack.unpackb(sample.payload.to_bytes()))

    def _send_state(self) -> None:
        step = 0
        while not self._stop.is_set():
            self._state.put(
                msgpack.packb(
                    {
                        "step": step,
                        "sim_time": step / 15,
                        "paused": False,
                        "seq": -1,
                        "q": HOME,
                        "ee_pos": [0.36, 0.0, 0.47],
                        "ee_quat": [0.0, 1.0, 0.0, 0.0],
                        "gripper": 0.5,
                    }
                )
            )
            step += 1
            time.sleep(0.05)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.session.close()


@pytest.fixture
def bridge() -> Iterator[tuple[FakeRunner, RoboLabAdapter]]:
    endpoint = _free_endpoint()
    runner = FakeRunner(endpoint)
    adapter = RoboLabAdapter(address=endpoint)
    assert adapter.connect()
    try:
        yield runner, adapter
    finally:
        adapter.disconnect()
        runner.close()


def _wait_for(predicate: Any, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.02)


def test_adapter_reads_arm_then_finger_joint_in_radians(
    bridge: tuple[FakeRunner, RoboLabAdapter],
) -> None:
    _, adapter = bridge

    positions = adapter.read_joint_positions()

    assert positions[:7] == pytest.approx(HOME)
    # RoboLab measures the gripper 0 open .. 1 closed; finger_joint closes at pi/4.
    assert positions[7] == pytest.approx(0.5 * math.pi / 4)
    assert adapter.get_limits().position_upper[7] == pytest.approx(math.pi / 4)


def test_adapter_sends_changed_targets_once(bridge: tuple[FakeRunner, RoboLabAdapter]) -> None:
    runner, adapter = bridge
    assert adapter.set_control_mode(ControlMode.SERVO_POSITION)
    target = [*HOME[:6], 0.3, math.pi / 4]

    assert adapter.write_joint_positions(target)
    assert adapter.write_joint_positions(target)
    _wait_for(lambda: len(runner.commands) >= 1)
    time.sleep(0.2)

    assert len(runner.commands) == 1
    assert runner.commands[0]["seq"] == 1
    assert runner.commands[0]["q"] == pytest.approx(target[:7])
    assert runner.commands[0]["gripper"] == pytest.approx(1.0)


def test_adapter_refuses_velocity_and_torque_modes() -> None:
    adapter = RoboLabAdapter()

    assert not adapter.set_control_mode(ControlMode.VELOCITY)
    assert not adapter.set_control_mode(ControlMode.TORQUE)


def test_state_and_camera_frame_decode() -> None:
    state = RobotState.decode(
        {"step": 3, "sim_time": 0.2, "paused": True, "seq": 7, "q": HOME, "gripper": 1.0}
    )
    rgb = np.zeros((4, 6, 3), np.uint8)
    depth = np.full((4, 6, 1), np.inf, np.float32)
    frame = CameraFrame.decode(
        {
            "step": 3,
            "sim_time": 0.2,
            "camera": "over_shoulder_left_camera",
            "rgb": {"shape": rgb.shape, "data": rgb.tobytes()},
            "depth": {"shape": depth.shape, "data": depth.tobytes()},
            "K": [500.0, 0.0, 3.0, 0.0, 500.0, 2.0, 0.0, 0.0, 1.0],
            "cam_pos": [0.05, 0.57, 0.66],
            "cam_quat": [0.805, -0.393, -0.195, 0.399],
        }
    )

    assert (state.paused, state.seq, state.q.tolist()) == (True, 7, HOME)
    assert frame.rgb.shape == (4, 6, 3)
    assert frame.depth.shape == (4, 6)
    assert frame.K[0, 2] == 3.0
    # RoboLab sends w, x, y, z; the frame keeps x, y, z, w.
    assert frame.quat_xyzw.tolist() == [-0.393, -0.195, 0.399, 0.805]
