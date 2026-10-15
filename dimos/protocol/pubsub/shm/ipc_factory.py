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

from abc import ABC, abstractmethod
from multiprocessing.shared_memory import SharedMemory
import os
import time
from typing import Any
import weakref

import numpy as np
from numpy.typing import DTypeLike

from dimos.utils.logging_config import setup_logger
from dimos.utils.shm import ShmNotReadyError, attach_shm

_UNLINK_ON_GC = os.getenv("DIMOS_IPC_UNLINK_ON_GC", "0").lower() not in ("0", "false", "no")

logger = setup_logger()


class FrameChannel(ABC):
    """Shared-memory IPC channel carrying frames behind a tiny control block.

    The double buffer retains the latest frame. Descriptors are JSON-safe;
    attach() reconstructs a reader in another process.
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
        return SharedMemory(create=True, size=size, name=name), True
    except FileExistsError:
        segment = attach_shm(name)
        if segment.size != size:
            segment.close()
            raise ValueError(f"SHM {name!r} layout mismatch: expected {size} bytes")
        return segment, False


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
            self._own_data = self._own_ctrl = True
        else:
            self._shm_data, self._own_data = _create_or_open(data_name, 2 * self._nbytes)
            self._shm_ctrl, self._own_ctrl = _create_or_open(ctrl_name, 24)

        self._ctrl = np.ndarray((3,), dtype=np.int64, buffer=self._shm_ctrl.buf)
        if self._own_ctrl:
            self._ctrl[:] = 0  # initialize only once

        # only owners set unlink finalizers (beware cross-process timing)
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

    def current_seq(self) -> int:
        return int(self._ctrl[0])

    def read(self, last_seq: int = -1, require_new: bool = True):  # type: ignore[no-untyped-def]
        for _ in range(3):
            seq1 = int(self._ctrl[0])
            idx = int(self._ctrl[2])
            ts = int(self._ctrl[1])
            view = np.ndarray(
                self._shape, dtype=self._dtype, buffer=self._shm_data.buf, offset=idx * self._nbytes
            )
            if require_new and seq1 == last_seq:
                return seq1, ts, None
            snapshot = np.array(view, copy=True)
            if seq1 == int(self._ctrl[0]):
                return seq1, ts, snapshot
        return last_seq, 0, None

    def descriptor(self):  # type: ignore[no-untyped-def]
        return {
            "kind": "cpu",
            "shape": self._shape,
            "dtype": self._dtype.str,
            "nbytes": self._nbytes,
            "data_name": self._shm_data.name,
            "ctrl_name": self._shm_ctrl.name,
        }

    @classmethod
    def attach(cls, desc: dict[str, Any]) -> "CpuShmChannel":
        obj = object.__new__(cls)
        obj._shape = tuple(desc["shape"])
        obj._dtype = np.dtype(desc["dtype"])
        obj._nbytes = int(desc["nbytes"])
        data_name = desc["data_name"]
        ctrl_name = desc["ctrl_name"]
        try:
            obj._shm_data = attach_shm(data_name)
            obj._shm_ctrl = attach_shm(ctrl_name)
        except ShmNotReadyError as e:
            raise ShmNotReadyError(
                f"CPU IPC attach failed: control/data SHM never became ready "
                f"(ctrl='{ctrl_name}', data='{data_name}'): {e}. "
                f"Ensure the writer is running on the same host and the channel is alive."
            ) from e
        obj._ctrl = np.ndarray((3,), dtype=np.int64, buffer=obj._shm_ctrl.buf)
        # attachments don’t own/unlink
        obj._own_data = obj._own_ctrl = False
        obj._finalizer_data = obj._finalizer_ctrl = None
        return obj

    def close(self) -> None:
        self._shm_ctrl.close()
        self._shm_data.close()
        if self._own_ctrl:
            _safe_unlink(self._shm_ctrl.name)
        if self._own_data:
            _safe_unlink(self._shm_data.name)
