# Copyright 2025-2026 Dimensional Inc.
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

import sys
from types import ModuleType
from typing import Any, cast

import numpy as np
from numpy.testing import assert_array_equal
from pytest import MonkeyPatch

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree import mujoco_connection
from dimos.robot.unitree.mujoco_connection import MujocoConnection


def _bare_connection(monkeypatch: MonkeyPatch) -> MujocoConnection:
    mjx_env = ModuleType("mujoco_playground._src.mjx_env")
    mjx_env.ensure_menagerie_exists = lambda: None  # type: ignore[attr-defined]
    playground_src = ModuleType("mujoco_playground._src")
    playground_src.mjx_env = mjx_env  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "mujoco_playground._src", playground_src)
    monkeypatch.setattr(mujoco_connection, "get_data", lambda _name: None)
    return MujocoConnection(GlobalConfig())


def test_start_and_stop_own_the_simulation_thread(monkeypatch: MonkeyPatch) -> None:
    connection = _bare_connection(monkeypatch)
    from dimos.simulation.mujoco import locomotion_sim

    calls: list[tuple[GlobalConfig, MujocoConnection]] = []

    def run(config: GlobalConfig, io: MujocoConnection) -> None:
        calls.append((config, io))
        io.signal_ready()
        assert io._simulation_stop_event.wait(timeout=1.0)

    monkeypatch.setattr(locomotion_sim, "run_locomotion_sim", run)

    connection.start()
    thread = connection._simulation_thread
    assert thread is not None
    assert thread.is_alive()

    connection.stop()

    assert calls == [(connection.global_config, connection)]
    assert not thread.is_alive()
    assert connection._simulation_thread is None


def test_move_holds_latest_policy_command(monkeypatch: MonkeyPatch) -> None:
    connection = _bare_connection(monkeypatch)

    connection.move(Twist(linear=[1.0, -0.5, 0.0], angular=[0.0, 0.0, 0.25]))
    first = connection.get_command()
    connection.stop_movement()

    assert_array_equal(first, np.array([1.0, -0.5, 0.25], dtype=np.float32))
    assert_array_equal(connection.get_command(), np.zeros(3, dtype=np.float32))


def test_observations_are_latest_only(monkeypatch: MonkeyPatch) -> None:
    connection = _bare_connection(monkeypatch)
    first_frame = np.zeros((2, 2, 3), dtype=np.uint8)
    latest_frame = np.ones((2, 2, 3), dtype=np.uint8)
    lidar = cast("Any", object())

    connection.publish_video(first_frame)
    connection.publish_video(latest_frame)
    connection.publish_odom(
        np.array([1.0, 2.0, 3.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
        4.0,
    )
    connection.publish_lidar(lidar)

    assert_array_equal(connection.get_video_frame(), latest_frame)
    assert connection.get_video_frame() is None
    odom = connection.get_odom_message()
    assert odom is not None
    assert (odom.position.x, odom.position.y, odom.position.z) == (1.0, 2.0, 3.0)
    assert odom.ts == 4.0
    assert connection.get_odom_message() is None
    assert connection.get_lidar_message() is lidar
    assert connection.get_lidar_message() is None
