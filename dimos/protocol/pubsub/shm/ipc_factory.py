# Copyright 2025 Dimensional Inc.
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
from multiprocessing.shared_memory import SharedMemory
import os
import time

import numpy as np

_UNLINK_ON_GC = os.getenv("DIMOS_IPC_UNLINK_ON_GC", "0").lower() not in ("0", "false", "no")


def _open_shm_with_retry(name: str) -> SharedMemory:
    tries = int(os.getenv("DIMOS_IPC_ATTACH_RETRIES", "40"))  # ~40 tries
    base_ms = float(os.getenv("DIMOS_IPC_ATTACH_BACKOFF_MS", "5"))  # 5 ms
    cap_ms = float(os.getenv("DIMOS_IPC_ATTACH_BACKOFF_CAP_MS", "200"))  # 200 ms
    last = None
    for i in range(tries):
        try:
            return SharedMemory(name=name)
        except FileNotFoundError as e:
            last = e
            # exponential backoff, capped
            time.sleep(min((base_ms * (2**i)), cap_ms) / 1000.0)
    raise FileNotFoundError(f"SHM not found after {tries} retries: {name}") from last


def _sanitize_shm_name(name: str) -> str:
    #  Python's SharedMemory expects names like 'psm_abc', without leading '/'
    return name.lstrip("/") if isinstance(name, str) else name


# ---------------------------
# 1) Abstract interface
# ---------------------------


class FrameChannel(ABC):
    """Single-slot 'freshest frame' IPC channel with a tiny control block.
    - Double-buffered to avoid torn reads.
    - Descriptor is JSON-safe; attach() reconstructs in another process.
    """

    @property
    @abstractmethod
    def device(self) -> str:  # "cpu" or "cuda"
        ...

    @property
    @abstractmethod
    def shape(self) -> tuple: ...  # type: ignore[type-arg]

    @property
    @abstractmethod
    def dtype(self) -> np.dtype: ...  # type: ignore[type-arg]

    @abstractmethod
    def publish(self, frame) -> None:  # type: ignore[no-untyped-def]
        """Write into inactive buffer, then flip visible index (write control last)."""
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


from multiprocessing.shared_memory import SharedMemory
import os
import weakref


def _safe_unlink(name: str) -> None:
    try:
        shm = SharedMemory(name=name)
        shm.unlink()
    except FileNotFoundError:
        pass
    except Exception:
        pass


# ---------------------------
# 2) CPU shared-memory backend
# ---------------------------


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

        def _create_or_open(name: str, size: int):  # type: ignore[no-untyped-def]
            try:
                shm = SharedMemory(create=True, size=size, name=name)
                owner = True
            except FileExistsError:
                shm = SharedMemory(name=name)  # attach existing
                owner = False
            return shm, owner

        if data_name is None or ctrl_name is None:
            # fallback: random names (old behavior)
            self._shm_data = SharedMemory(create=True, size=2 * self._nbytes)
            self._shm_ctrl = SharedMemory(create=True, size=24)
            self._is_owner = True
        else:
            self._shm_data, own_d = _create_or_open(data_name, 2 * self._nbytes)
            self._shm_ctrl, own_c = _create_or_open(ctrl_name, 24)
            self._is_owner = own_d and own_c

        self._ctrl = np.ndarray((3,), dtype=np.int64, buffer=self._shm_ctrl.buf)  # type: ignore[var-annotated]
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

    def publish(self, frame) -> None:  # type: ignore[no-untyped-def]
        assert isinstance(frame, np.ndarray)
        assert frame.shape == self._shape and frame.dtype == self._dtype
        active = int(self._ctrl[2])
        inactive = 1 - active
        view = np.ndarray(  # type: ignore[var-annotated]
            self._shape,
            dtype=self._dtype,
            buffer=self._shm_data.buf,
            offset=inactive * self._nbytes,
        )
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
            view = np.ndarray(  # type: ignore[var-annotated]
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


# ---------------------------
# 3) Factories
# ---------------------------


class CPU_IPC_Factory:
    """Creates/attaches CPU shared-memory channels."""

    @staticmethod
    def create(shape, dtype=np.uint8) -> CpuShmChannel:  # type: ignore[no-untyped-def]
        return CpuShmChannel(shape, dtype=dtype)

    @staticmethod
    def attach(desc: dict) -> CpuShmChannel:  # type: ignore[type-arg]
        assert desc.get("kind") == "cpu", "Descriptor kind mismatch"
        return CpuShmChannel.attach(desc)  # type: ignore[arg-type, no-any-return]


# ---------------------------
# 4) Runtime selector
# ---------------------------


def make_frame_channel(  # type: ignore[no-untyped-def]
    shape, dtype=np.uint8, prefer: str = "auto", device: int = 0
) -> FrameChannel:
    """Choose CUDA IPC if available (or requested), otherwise CPU SHM."""
    # TODO: Implement the CUDA version of creating this factory
    return CPU_IPC_Factory.create(shape, dtype=dtype)
