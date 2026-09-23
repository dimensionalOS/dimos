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

import numpy as np
import pytest

from dimos.simulation.mujoco.shared_memory import ShmReader, ShmWriter


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

    points = np.array([[0.0, 1.0, 2.0], [3.5, -4.25, 5.125]], dtype=np.float32)
    reader.write_lidar(points, 12.5)

    lidar, seq = writer.read_lidar()
    assert seq == 1
    assert lidar is not None
    got, ts = lidar
    assert ts == 12.5
    assert got.dtype == np.float32
    np.testing.assert_array_equal(got, points)
