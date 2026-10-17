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

from collections.abc import Iterator
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pytest
from pytest import MonkeyPatch

from dimos.simulation.mujoco.shared_memory import ShmReader, ShmWriter

_frame_1 = np.array([[0.0, 1.0, 2.0], [3.5, -4.25, 5.125]], dtype=np.float32)
_frame_2 = np.array([[6.0, 7.0, 8.0]], dtype=np.float32)


@pytest.fixture
def shm_pair() -> Iterator[tuple[ShmWriter, ShmReader]]:
    writer = ShmWriter()
    reader = ShmReader(writer.shm.to_names())
    yield writer, reader
    reader.cleanup()
    writer.cleanup()


def test_lidar_round_trip(shm_pair: tuple[ShmWriter, ShmReader]) -> None:
    writer, reader = shm_pair
    assert writer.read_lidar() == (None, 0)

    reader.write_lidar(_frame_1, 12.5)

    lidar, seq = writer.read_lidar()
    assert seq == 1
    assert lidar is not None
    got, ts = lidar
    assert ts == 12.5
    assert got.dtype == np.float32
    np.testing.assert_array_equal(got, _frame_1)


def test_command_round_trip(shm_pair: tuple[ShmWriter, ShmReader]) -> None:
    writer, reader = shm_pair
    assert reader.read_command() is None

    writer.write_command(np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0]))

    command = reader.read_command()
    assert command is not None
    linear, angular = command
    np.testing.assert_array_equal(linear, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(angular, [4.0, 5.0, 6.0])
    assert reader.read_command() is None  # the same command again


def test_torn_lidar_read_is_retried(
    shm_pair: tuple[ShmWriter, ShmReader], monkeypatch: MonkeyPatch
) -> None:
    """A frame written while the reader copies must not leak into the result."""
    writer, reader = shm_pair
    reader.write_lidar(_frame_1, 1.0)
    copy = writer._copy_lidar
    copies = 0

    def racing_copy() -> tuple[NDArray[Any], float] | None:
        nonlocal copies
        copies += 1
        if copies == 1:
            reader.write_lidar(_frame_2, 2.0)  # the simulator overtakes the copy
        return copy()

    monkeypatch.setattr(writer, "_copy_lidar", racing_copy)

    lidar, seq = writer.read_lidar()
    assert seq == 2
    assert lidar is not None
    got, ts = lidar
    assert ts == 2.0
    np.testing.assert_array_equal(got, _frame_2)
    assert copies == 2


def test_frame_being_written_is_not_delivered(shm_pair: tuple[ShmWriter, ShmReader]) -> None:
    writer, reader = shm_pair
    reader.write_lidar(_frame_1, 1.0)

    with reader._writing(4):  # the simulator is half-way through frame 2
        assert writer.read_lidar() == (None, 0)

    assert writer.read_lidar()[1] == 2


def test_run_flag_round_trip(shm_pair: tuple[ShmWriter, ShmReader]) -> None:
    writer, reader = shm_pair
    assert not writer.is_ready()
    assert not reader.should_run()
    assert not reader.should_stop()

    reader.signal_ready()
    writer.signal_run()

    assert writer.is_ready()
    assert reader.should_run()
    assert not reader.should_stop()

    writer.signal_stop()
    assert reader.should_stop()
