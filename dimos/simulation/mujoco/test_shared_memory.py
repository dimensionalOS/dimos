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

from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from dimos.simulation.mujoco.shared_memory import ShmWriter


@pytest.fixture
def consumer():
    value = ShmWriter()
    try:
        yield value
    finally:
        value.cleanup()


def test_lidar_cross_process_generated_cdr():
    demo = Path(__file__).resolve().parents[3] / "examples/message-codegen/demo_mujoco_lidar_shm.py"
    result = subprocess.run([sys.executable, str(demo)], capture_output=True, text=True, timeout=45)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS:" in result.stdout
    assert "resource_tracker" not in result.stderr


def test_lidar_ignores_uncommitted_write(consumer):
    sequence = np.ndarray((8,), dtype=np.int64, buffer=consumer.shm.seq.buf)
    sequence[4] = 1
    assert consumer.read_lidar() == (None, 0)


@pytest.mark.parametrize("length", [0, 4, 2**32 - 1])
def test_lidar_invalid_payload_is_not_delivered(consumer, length):
    sequence = np.ndarray((8,), dtype=np.int64, buffer=consumer.shm.seq.buf)
    sequence[4] = 2
    size = np.ndarray((1,), dtype=np.uint32, buffer=consumer.shm.lidar_len.buf)
    size[0] = length
    consumer.shm.lidar.buf[:4] = b"bad!"
    assert consumer.read_lidar() == (None, 0)
