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

# frame_ipc.py
# Python 3.9+
from abc import ABC, abstractmethod
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory
import os
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import DTypeLike, NDArray

from dimos.utils.logging_config import setup_logger

_UNLINK_ON_GC = os.getenv("DIMOS_IPC_UNLINK_ON_GC", "0").lower() not in ("0", "false", "no")

logger = setup_logger()


def _unregister(shm: SharedMemory) -> SharedMemory:
    """Remove a SharedMemory segment from the resource tracker.

    We manage lifecycle explicitly via close()/unlink(), so the resource
    tracker must not attempt cleanup on process exit — that causes KeyError
    spam when multiple processes share the same named segment.
    """
    try:
        resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    except Exception:
        pass
    return shm


def _open_shm_with_retry(name: str) -> SharedMemory:
    tries = int(os.getenv("DIMOS_IPC_ATTACH_RETRIES", "40"))  # ~40 tries
    base_ms = float(os.getenv("DIMOS_IPC_ATTACH_BACKOFF_MS", "5"))  # 5 ms
    cap_ms = float(os.getenv("DIMOS_IPC_ATTACH_BACKOFF_CAP_MS", "200"))  # 200 ms
    last = None
    for i in range(tries):
        try:
            return _unregister(SharedMemory(name=name))
        except FileNotFoundError as e:
            last = e
            # exponential backoff, capped
            time.sleep(min((base_ms * (2**i)), cap_ms) / 1000.0)
    raise FileNotFoundError(f"SHM not found after {tries} retries: {name}") from last


class FrameChannel(ABC):
    """Shared-memory IPC channel carrying frames behind a tiny control block.

    Implementations range from a single-slot double-buffered 'freshest frame'
    channel (CpuShmChannel) to a multi-slot ring buffer for reliable delivery
    (CpuShmQueue). Descriptor is JSON-safe; attach() reconstructs in another
    process.
    """

    @abstractmethod
    def __init__(
        self,
        shape: tuple[int, ...],
        dtype: DTypeLike = np.uint8,
        *,
        data_name: str | None = None,
        ctrl_name: str | None = None,
    ) -> None:
        """Create (or attach by name) the channel's shared-memory segments."""
        ...

    @property
    @abstractmethod
    def device(self) -> str:  # "cpu" or "cuda"
        ...

    @property
    @abstractmethod
    def shape(self) -> tuple: ...  # type: ignore[type-arg]

    @property
    @abstractmethod
    def dtype(self) -> np.dtype: ...

    @abstractmethod
    def publish(self, frame, length: int | None = None) -> None:  # type: ignore[no-untyped-def]
        """Write into inactive buffer, then flip visible index (write control last).

        Args:
            frame: The numpy array to publish
            length: Optional length to copy (for variable-size messages). If None, copies full frame.
        """
        ...

    @abstractmethod
    def read(self, last_seq: int = -1, require_new: bool = True):  # type: ignore[no-untyped-def]
        """Return (seq:int, ts_ns:int, view-or-None)."""
        ...

    @abstractmethod
    def descriptor(self) -> dict:  # type: ignore[type-arg]
        """Tiny JSON-safe descriptor (names/handles/shape/dtype/device)."""
        ...

    @classmethod
    @abstractmethod
    def attach(cls, desc: dict) -> "FrameChannel":  # type: ignore[type-arg]
        """Attach in another process."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Detach resources (owner also unlinks manager if applicable)."""
        ...


import os
import weakref


def _safe_unlink(name: str) -> None:
    try:
        shm = SharedMemory(name=name)
        shm.unlink()  # unlink() calls resource_tracker.unregister()
        shm.close()
    except FileNotFoundError:
        pass
    except Exception:
        pass


def _create_or_open(name: str, size: int) -> tuple[SharedMemory, bool]:
    """Create a named SHM segment (owner) or attach to an existing one (reader)."""
    try:
        # Owner: leave registered because unlink() will unregister, and
        # the tracker serves as safety net if the process crashes.
        shm = SharedMemory(create=True, size=size, name=name)
        owner = True
    except FileExistsError:
        # Reader: unregister because we only close(), never unlink().
        shm = _unregister(SharedMemory(name=name))
        owner = False
    return shm, owner


class CpuShmChannel(FrameChannel):
    def __init__(  # type: ignore[no-untyped-def]
        self,
        shape,
        dtype=np.uint8,
        *,
        data_name: str | None = None,
        ctrl_name: str | None = None,
    ) -> None:
        self._shape = tuple(shape)
        self._dtype = np.dtype(dtype)
        self._nbytes = int(self._dtype.itemsize * np.prod(self._shape))

        if data_name is None or ctrl_name is None:
            # Fallback: random names (old behavior) -> always owner
            self._shm_data = SharedMemory(create=True, size=2 * self._nbytes)
            self._shm_ctrl = SharedMemory(create=True, size=24)
            self._is_owner = True
        else:
            self._shm_data, own_d = _create_or_open(data_name, 2 * self._nbytes)
            self._shm_ctrl, own_c = _create_or_open(ctrl_name, 24)
            self._is_owner = own_d and own_c

        self._ctrl = np.ndarray((3,), dtype=np.int64, buffer=self._shm_ctrl.buf)
        if self._is_owner:
            self._ctrl[:] = 0  # initialize only once

        # only owners set unlink finalizers (beware cross-process timing)
        self._finalizer_data = (
            weakref.finalize(self, _safe_unlink, self._shm_data.name)
            if (_UNLINK_ON_GC and self._is_owner)
            else None
        )
        self._finalizer_ctrl = (
            weakref.finalize(self, _safe_unlink, self._shm_ctrl.name)
            if (_UNLINK_ON_GC and self._is_owner)
            else None
        )

    def descriptor(self):  # type: ignore[no-untyped-def]
        return {
            "kind": "cpu",
            "shape": self._shape,
            "dtype": self._dtype.str,
            "nbytes": self._nbytes,
            "data_name": self._shm_data.name,
            "ctrl_name": self._shm_ctrl.name,
        }

    @property
    def device(self) -> str:
        return "cpu"

    @property
    def shape(self):  # type: ignore[no-untyped-def]
        return self._shape

    @property
    def dtype(self):  # type: ignore[no-untyped-def]
        return self._dtype

    def publish(self, frame, length: int | None = None) -> None:  # type: ignore[no-untyped-def]
        assert isinstance(frame, np.ndarray)
        assert frame.shape == self._shape and frame.dtype == self._dtype
        active = int(self._ctrl[2])
        inactive = 1 - active
        view = np.ndarray(
            self._shape,
            dtype=self._dtype,
            buffer=self._shm_data.buf,
            offset=inactive * self._nbytes,
        )
        # Only copy actual payload length if specified, otherwise copy full frame
        if length is not None and length < len(frame):
            np.copyto(view[:length], frame[:length], casting="no")
        else:
            np.copyto(view, frame, casting="no")
        ts = np.int64(time.time_ns())
        # Publish order: ts -> idx -> seq
        self._ctrl[1] = ts
        self._ctrl[2] = inactive
        self._ctrl[0] += 1

    def read(self, last_seq: int = -1, require_new: bool = True):  # type: ignore[no-untyped-def]
        for _ in range(3):
            seq1 = int(self._ctrl[0])
            idx = int(self._ctrl[2])
            ts = int(self._ctrl[1])
            view = np.ndarray(
                self._shape, dtype=self._dtype, buffer=self._shm_data.buf, offset=idx * self._nbytes
            )
            if seq1 == int(self._ctrl[0]):
                if require_new and seq1 == last_seq:
                    return seq1, ts, None
                return seq1, ts, view
        return last_seq, 0, None

    def descriptor(self):  # type: ignore[no-redef, no-untyped-def]
        return {
            "kind": "cpu",
            "shape": self._shape,
            "dtype": self._dtype.str,
            "nbytes": self._nbytes,
            "data_name": self._shm_data.name,
            "ctrl_name": self._shm_ctrl.name,
        }

    @classmethod
    def attach(cls, desc: str):  # type: ignore[no-untyped-def, override]
        obj = object.__new__(cls)
        obj._shape = tuple(desc["shape"])  # type: ignore[index]
        obj._dtype = np.dtype(desc["dtype"])  # type: ignore[index]
        obj._nbytes = int(desc["nbytes"])  # type: ignore[index]
        data_name = desc["data_name"]  # type: ignore[index]
        ctrl_name = desc["ctrl_name"]  # type: ignore[index]
        try:
            obj._shm_data = _open_shm_with_retry(data_name)
            obj._shm_ctrl = _open_shm_with_retry(ctrl_name)
        except FileNotFoundError as e:
            raise FileNotFoundError(
                f"CPU IPC attach failed: control/data SHM not found "
                f"(ctrl='{ctrl_name}', data='{data_name}'). "
                f"Ensure the writer is running on the same host and the channel is alive."
            ) from e
        obj._ctrl = np.ndarray((3,), dtype=np.int64, buffer=obj._shm_ctrl.buf)
        # attachments don’t own/unlink
        obj._finalizer_data = obj._finalizer_ctrl = None
        return obj

    def close(self) -> None:
        if getattr(self, "_is_owner", False):
            try:
                self._shm_ctrl.close()
            finally:
                try:
                    _safe_unlink(self._shm_ctrl.name)
                except:
                    pass
            if hasattr(self, "_shm_data"):
                try:
                    self._shm_data.close()
                finally:
                    try:
                        _safe_unlink(self._shm_data.name)
                    except:
                        pass
            return
        # readers: just close handles
        try:
            self._shm_ctrl.close()
        except:
            pass
        try:
            self._shm_data.close()
        except:
            pass


class CpuShmQueue(FrameChannel):
    """Multi-slot ring-buffer SHM channel for reliable delivery under load.

    Delivery is reliable for a single writer *instance* (publishes are serialised
    by ``_pub_lock``) feeding readers that keep up: every message lands in its own
    slot with a monotonic seq. Two limits remain by design:

    - ``_pub_lock`` is per-instance, so multiple writer instances/processes on the
      *same* named segment are not mutually excluded and can race the seq counter.
      The pubsub uses one writer instance per topic.
    - A reader outpaced by more than ``slots`` messages loses the oldest (logged).
      Size ``slots`` for the expected burst; a multi-reader ring cannot apply
      backpressure.
    """

    _HEADER_FIELDS = 3  # (seq, ts, length) per slot, all int64
    _CTRL_SLOTS = 2  # (producer_seq, last_ts)
    # Ring depth. Sized so a reader polling at ~1ms keeps up with realistic RPC
    # bursts without overflow.
    _DEFAULT_SLOTS = 256

    def __init__(
        self,
        shape: tuple[int, ...],
        dtype: DTypeLike = np.uint8,
        *,
        data_name: str | None = None,
        ctrl_name: str | None = None,
        slots: int = _DEFAULT_SLOTS,
    ) -> None:
        self._shape = tuple(shape)
        self._dtype = np.dtype(dtype)
        self._frame_nbytes = int(self._dtype.itemsize * np.prod(self._shape))
        self._slots = int(slots)
        self._pub_lock = threading.Lock()

        data_size = self._header_bytes + self._slots * self._frame_nbytes
        ctrl_size = self._CTRL_SLOTS * 8

        if data_name is None or ctrl_name is None:
            self._shm_data = SharedMemory(create=True, size=data_size)
            self._shm_ctrl = SharedMemory(create=True, size=ctrl_size)
            self._own_data = self._own_ctrl = True
        else:
            # Track ownership per segment: under a first-attach race two processes
            # can split ownership (one creates data, the other ctrl), and each must
            # unlink the segment it created or it leaks.
            self._shm_data, self._own_data = _create_or_open(data_name, data_size)
            self._shm_ctrl, self._own_ctrl = _create_or_open(ctrl_name, ctrl_size)
            # A segment we opened (didn't create) must be at least our layout
            # size. It won't be if a peer built it with a different slots/capacity:
            # the segment name keys on topic+capacity+class, not slots, so a custom
            # slots -- or a rolling upgrade with a different ring size -- collides on
            # the name and would index off the end.
            assert self._own_data or self._shm_data.size >= data_size, (
                f"opened SHM {data_name!r} is {self._shm_data.size}B < {data_size}B needed: "
                f"CpuShmQueue slots/capacity mismatch with the segment's creator"
            )

        self._ctrl: NDArray[np.int64] = np.ndarray(
            (self._CTRL_SLOTS,), dtype=np.int64, buffer=self._shm_ctrl.buf
        )
        if self._own_ctrl:
            self._ctrl[:] = 0

        self._finalizer_data = (
            weakref.finalize(self, _safe_unlink, self._shm_data.name)
            if (_UNLINK_ON_GC and self._own_data)
            else None
        )
        self._finalizer_ctrl = (
            weakref.finalize(self, _safe_unlink, self._shm_ctrl.name)
            if (_UNLINK_ON_GC and self._own_ctrl)
            else None
        )

    @property
    def _header_bytes(self) -> int:
        return self._slots * self._HEADER_FIELDS * 8

    def _map(self) -> tuple[NDArray[np.int64], NDArray[np.uint8]]:
        """Build fresh header/payload views over the data segment.

        Views are transient (garbage-collected when the caller returns) so
        ``close()`` never trips over exported buffer pointers.
        """
        buf = self._shm_data.buf
        headers: NDArray[np.int64] = np.ndarray(
            (self._slots, self._HEADER_FIELDS), dtype=np.int64, buffer=buf
        )
        payloads: NDArray[np.uint8] = np.ndarray(
            (self._slots, self._frame_nbytes),
            dtype=np.uint8,
            buffer=buf,
            offset=self._header_bytes,
        )
        return headers, payloads

    @property
    def device(self) -> str:
        return "cpu"

    @property
    def shape(self) -> tuple[int, ...]:
        return self._shape

    @property
    def dtype(self) -> np.dtype[Any]:
        return self._dtype

    def publish(self, frame: NDArray[Any], length: int | None = None) -> None:
        assert isinstance(frame, np.ndarray)
        assert frame.shape == self._shape and frame.dtype == self._dtype
        src = np.frombuffer(np.ascontiguousarray(frame), dtype=np.uint8)
        n = self._frame_nbytes if length is None else min(int(length), self._frame_nbytes)
        headers, payloads = self._map()
        with self._pub_lock:
            seq = int(self._ctrl[0]) + 1
            ts = int(time.time_ns())
            slot = seq % self._slots
            # Invalidate the slot before overwriting it: a reader still holding
            # the previous occupant's seq sees 0 on its post-copy re-check and
            # discards the torn payload instead of returning a mix of two
            # messages. Then payload, metadata, and the new seq last, so a reader
            # that observes the seq is guaranteed a fully-written slot.
            headers[slot, 0] = 0
            payloads[slot, :n] = src[:n]
            headers[slot, 1] = ts
            headers[slot, 2] = n
            headers[slot, 0] = seq
            # Publish globally: ts before seq (readers key off _ctrl[0]).
            self._ctrl[1] = ts
            self._ctrl[0] = seq

    def _read_slot(
        self, headers: NDArray[np.int64], payloads: NDArray[np.uint8], want: int
    ) -> tuple[int, NDArray[np.uint8]] | None:
        """Return (ts, payload_copy) if the slot still holds ``want``, else None."""
        slot = want % self._slots
        for _ in range(3):
            if int(headers[slot, 0]) != want:
                return None  # not yet written, or already overwritten
            ts = int(headers[slot, 1])
            n = int(headers[slot, 2])
            if n < 0 or n > self._frame_nbytes:
                continue  # torn header; retry
            payload = np.array(payloads[slot, :n], copy=True)
            if int(headers[slot, 0]) == want:
                return ts, payload  # seq stable across the copy -> consistent
        return None

    def read(
        self, last_seq: int = -1, require_new: bool = True
    ) -> tuple[int, int, NDArray[np.uint8] | None]:
        current = int(self._ctrl[0])
        if current <= 0:
            return last_seq, int(self._ctrl[1]), None
        if require_new:
            if current <= last_seq:
                return last_seq, int(self._ctrl[1]), None
            # Clamp to the first real seq: seqs start at 1, so the ABC's default
            # last_seq=-1 must not make want=0 (a phantom seq that inflates the
            # outpaced-drop count and matches the zero-initialised slot 0).
            want = max(1, last_seq + 1)
            oldest = max(1, current - self._slots + 1)
            if want < oldest:
                logger.warning(
                    f"CpuShmQueue reader outpaced: dropping {oldest - want} message(s) "
                    f"(seq {want} -> {oldest}); increase slots or poll faster"
                )
                want = oldest
        else:
            want = current  # newest available
        headers, payloads = self._map()
        # Skip any messages overwritten between the snapshot and our read.
        while want <= current:
            got = self._read_slot(headers, payloads, want)
            if got is not None:
                ts, payload = got
                return want, ts, payload
            want += 1
        return last_seq, int(self._ctrl[1]), None

    def descriptor(self) -> dict[str, Any]:
        return {
            "kind": "cpu_queue",
            "shape": self._shape,
            "dtype": self._dtype.str,
            "frame_nbytes": self._frame_nbytes,
            "slots": self._slots,
            "data_name": self._shm_data.name,
            "ctrl_name": self._shm_ctrl.name,
        }

    @classmethod
    def attach(cls, desc: dict[str, Any]) -> "CpuShmQueue":
        obj = object.__new__(cls)
        obj._shape = tuple(desc["shape"])
        obj._dtype = np.dtype(desc["dtype"])
        obj._frame_nbytes = int(desc["frame_nbytes"])
        obj._slots = int(desc["slots"])
        obj._pub_lock = threading.Lock()
        data_name = desc["data_name"]
        ctrl_name = desc["ctrl_name"]
        try:
            obj._shm_data = _open_shm_with_retry(data_name)
            obj._shm_ctrl = _open_shm_with_retry(ctrl_name)
        except FileNotFoundError as e:
            raise FileNotFoundError(
                f"CPU IPC queue attach failed: control/data SHM not found "
                f"(ctrl='{ctrl_name}', data='{data_name}'). "
                f"Ensure the writer is running on the same host and the channel is alive."
            ) from e
        obj._ctrl = np.ndarray((cls._CTRL_SLOTS,), dtype=np.int64, buffer=obj._shm_ctrl.buf)
        obj._own_data = obj._own_ctrl = False
        obj._finalizer_data = obj._finalizer_ctrl = None
        return obj

    def close(self) -> None:
        self._shm_ctrl.close()
        self._shm_data.close()
        # Unlink each segment we created; a reader that created neither just drops
        # its handles.
        if self._own_ctrl:
            _safe_unlink(self._shm_ctrl.name)
        if self._own_data:
            _safe_unlink(self._shm_data.name)


class CPU_IPC_Factory:
    """Creates/attaches CPU shared-memory channels."""

    @staticmethod
    def create(shape, dtype=np.uint8) -> CpuShmChannel:  # type: ignore[no-untyped-def]
        return CpuShmChannel(shape, dtype=dtype)

    @staticmethod
    def attach(desc: dict) -> CpuShmChannel:  # type: ignore[type-arg]
        assert desc.get("kind") == "cpu", "Descriptor kind mismatch"
        return CpuShmChannel.attach(desc)  # type: ignore[arg-type, no-any-return]


def make_frame_channel(  # type: ignore[no-untyped-def]
    shape, dtype=np.uint8, prefer: str = "auto", device: int = 0
) -> FrameChannel:
    """Choose CUDA IPC if available (or requested), otherwise CPU SHM."""
    # TODO: Implement the CUDA version of creating this factory
    return CPU_IPC_Factory.create(shape, dtype=dtype)
