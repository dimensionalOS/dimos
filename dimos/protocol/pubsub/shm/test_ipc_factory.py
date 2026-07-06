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

"""Unit tests for the ring-buffer SHM channel (``CpuShmQueue``)."""

import threading
import uuid

import numpy as np
import pytest

from dimos.protocol.pubsub.shm.ipc_factory import CpuShmQueue

CAP = 64


def _frame(payload: bytes) -> np.ndarray:
    frame = np.zeros((CAP,), dtype=np.uint8)
    frame[: len(payload)] = np.frombuffer(payload, dtype=np.uint8)
    return frame


def _publish(ch: CpuShmQueue, payload: bytes) -> None:
    ch.publish(_frame(payload), length=len(payload))


def _drain(ch: CpuShmQueue, last_seq: int = 0) -> tuple[list[tuple[int, bytes]], int]:
    """Read every available message, returning [(seq, payload_bytes), ...]."""
    out: list[tuple[int, bytes]] = []
    while True:
        seq, _ts, view = ch.read(last_seq=last_seq, require_new=True)
        if view is None:
            break
        last_seq = seq
        out.append((seq, view.tobytes()))
    return out, last_seq


def test_single_message() -> None:
    ch = CpuShmQueue((CAP,), np.uint8, slots=8)
    try:
        _publish(ch, b"hello")
        got, _ = _drain(ch)
        assert got == [(1, b"hello")]
    finally:
        ch.close()


def test_empty_channel_returns_none() -> None:
    ch = CpuShmQueue((CAP,), np.uint8, slots=8)
    try:
        _seq, _ts, view = ch.read(last_seq=0, require_new=True)
        assert view is None
    finally:
        ch.close()


def test_sequential_exact_once() -> None:
    """Every message published within one ring is delivered exactly once, in order."""
    ch = CpuShmQueue((CAP,), np.uint8, slots=8)
    try:
        for i in range(8):
            _publish(ch, f"m{i}".encode())
        got, _ = _drain(ch)
        assert [seq for seq, _ in got] == list(range(1, 9))
        assert [payload for _, payload in got] == [f"m{i}".encode() for i in range(8)]
    finally:
        ch.close()


def test_wraparound_when_reader_keeps_up() -> None:
    """A reader that drains after each publish loses nothing across many wraps."""
    ch = CpuShmQueue((CAP,), np.uint8, slots=4)
    try:
        received: list[bytes] = []
        last = 0
        for i in range(12):  # 3x the ring size
            _publish(ch, f"m{i}".encode())
            got, last = _drain(ch, last)
            received += [payload for _, payload in got]
        assert received == [f"m{i}".encode() for i in range(12)]
    finally:
        ch.close()


def test_reader_outpaced_drops_oldest() -> None:
    """When the ring overflows before a read, only the newest `slots` survive."""
    slots = 4
    ch = CpuShmQueue((CAP,), np.uint8, slots=slots)
    try:
        for i in range(2 * slots):  # publish 8 into 4 slots without reading
            _publish(ch, f"m{i}".encode())
        got, _ = _drain(ch)
        # The oldest `slots` messages were overwritten; loss is visible as a
        # sequence gap: the first delivered seq is slots+1, not 1.
        assert [seq for seq, _ in got] == list(range(slots + 1, 2 * slots + 1))
        assert [payload for _, payload in got] == [
            f"m{i}".encode() for i in range(slots, 2 * slots)
        ]
    finally:
        ch.close()


def test_concurrent_publishers_no_loss() -> None:
    """Threads sharing ONE instance publish concurrently with no loss or dupes.

    This exercises the per-instance ``_pub_lock`` (single-writer-instance thread
    safety). It does not cover multiple writer *instances* over one segment, which
    are not cross-process serialised -- see the ``CpuShmQueue`` class docstring.
    """
    slots = 32
    ch = CpuShmQueue((CAP,), np.uint8, slots=slots)
    try:
        threads = [
            threading.Thread(target=_publish, args=(ch, f"m{i}".encode())) for i in range(slots)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        got, _ = _drain(ch)
        assert len(got) == slots  # exact-once: no message lost
        assert sorted(seq for seq, _ in got) == list(range(1, slots + 1))  # unique seqs
        assert {payload for _, payload in got} == {f"m{i}".encode() for i in range(slots)}
    finally:
        ch.close()


def test_two_instances_share_named_segment() -> None:
    """Writer and reader in separate instances (as ShmRPC uses them) agree."""
    tag = uuid.uuid4().hex[:12]
    data_name, ctrl_name = f"tq_{tag}_data", f"tq_{tag}_ctrl"
    writer = CpuShmQueue((CAP,), np.uint8, data_name=data_name, ctrl_name=ctrl_name, slots=8)
    reader = CpuShmQueue((CAP,), np.uint8, data_name=data_name, ctrl_name=ctrl_name, slots=8)
    try:
        for i in range(5):
            _publish(writer, f"m{i}".encode())
        got, _ = _drain(reader)
        assert [payload for _, payload in got] == [f"m{i}".encode() for i in range(5)]
    finally:
        reader.close()
        writer.close()


def test_attach_roundtrip() -> None:
    """A channel attached from a descriptor reads what the owner published."""
    writer = CpuShmQueue((CAP,), np.uint8, slots=8)
    try:
        reader = CpuShmQueue.attach(writer.descriptor())
        try:
            _publish(writer, b"abc")
            _publish(writer, b"defg")
            got, _ = _drain(reader)
            assert [payload for _, payload in got] == [b"abc", b"defg"]
        finally:
            reader.close()
    finally:
        writer.close()


def test_layout_mismatch_rejected() -> None:
    """Opening a segment a peer built with a different `slots` fails loudly.

    The segment name keys on topic+capacity+class but not slots, so a slots
    (or ring-size) mismatch would otherwise index off the end of the segment.
    """
    tag = uuid.uuid4().hex[:12]
    data_name, ctrl_name = f"tq_{tag}_data", f"tq_{tag}_ctrl"
    owner = CpuShmQueue((CAP,), np.uint8, data_name=data_name, ctrl_name=ctrl_name, slots=4)
    try:
        with pytest.raises(AssertionError, match="slots/capacity mismatch"):
            CpuShmQueue((CAP,), np.uint8, data_name=data_name, ctrl_name=ctrl_name, slots=256)
    finally:
        owner.close()
