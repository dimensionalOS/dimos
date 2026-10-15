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

import os
import threading
import uuid

import numpy as np
import pytest

from dimos.protocol.pubsub.shm.ipc_factory import CpuShmChannel


@pytest.fixture
def channels():
    opened = []
    yield opened
    for channel in opened:
        channel.close()


def test_attach_reads_stable_snapshot(channels):
    writer = CpuShmChannel((4,), np.uint8)
    channels.append(writer)
    reader = CpuShmChannel.attach(writer.descriptor())
    channels.append(reader)
    writer.publish(np.array([1, 2, 3, 4], dtype=np.uint8))
    sequence, _, pixels = reader.read()
    assert sequence == 1
    np.testing.assert_array_equal(pixels, [1, 2, 3, 4])
    writer.publish(np.array([5, 6, 7, 8], dtype=np.uint8))
    writer.publish(np.array([9, 10, 11, 12], dtype=np.uint8))
    np.testing.assert_array_equal(pixels, [1, 2, 3, 4])
    _, _, newest = reader.read(last_seq=sequence)
    np.testing.assert_array_equal(newest, [9, 10, 11, 12])


def test_concurrent_creation_waits_for_sized_segments(monkeypatch, channels):
    original = os.ftruncate
    creation_started = threading.Event()
    allow_sizing = threading.Event()

    def delayed(fd, size):
        creation_started.set()
        assert allow_sizing.wait(2)
        return original(fd, size)

    monkeypatch.setattr(os, "ftruncate", delayed)
    tag = uuid.uuid4().hex
    errors = []

    def open_channel():
        try:
            channels.append(CpuShmChannel((4,), data_name=f"{tag}_data", ctrl_name=f"{tag}_ctrl"))
        except Exception as error:
            errors.append(error)

    first = threading.Thread(target=open_channel)
    second = threading.Thread(target=open_channel)
    first.start()
    assert creation_started.wait(2)
    second.start()
    allow_sizing.set()
    first.join(3)
    second.join(3)
    assert not first.is_alive() and not second.is_alive()
    assert errors == []
    assert len(channels) == 2
    channels[0].publish(np.array([4, 3, 2, 1], dtype=np.uint8))
    _, _, pixels = channels[1].read()
    np.testing.assert_array_equal(pixels, [4, 3, 2, 1])


def test_named_segment_layout_mismatch_is_rejected(channels):
    tag = uuid.uuid4().hex
    owner = CpuShmChannel((4,), data_name=f"{tag}_data", ctrl_name=f"{tag}_ctrl")
    channels.append(owner)
    with pytest.raises(ValueError, match="layout mismatch"):
        CpuShmChannel((8,), data_name=f"{tag}_data", ctrl_name=f"{tag}_ctrl")
