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
import base64
import time
from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.managers import SharedMemoryManager


def _ensure_cuda_context(cp, dev: int) -> None:
    """Create/init runtime+primary context on this thread for device `dev`."""
    cp.cuda.Device(dev).use()  # makes primary context current (creates if needed)
    try:
        cp.cuda.runtime.setDevice(dev)  # initialize runtime API binding
    except Exception:
        pass
    _ = cp.empty((1,), dtype=cp.uint8)  # force lazy init paths
    try:
        cp.cuda.runtime.deviceSynchronize()
    except Exception:
        pass


def _get_pci_triple(cp, dev: int) -> tuple[int, int, int]:
    """(domain, bus, device) from cudaDeviceProp; falls back to (-1,-1,-1)."""
    props = cp.cuda.runtime.getDeviceProperties(dev)
    dom = int(props.get("pciDomainID", 0))
    bus = int(props.get("pciBusID", -1))
    devn = int(props.get("pciDeviceID", -1))
    return (dom, bus, devn)


def _map_pci_to_local_device(cp, target_pci: tuple[int, int, int]) -> int | None:
    """Find local ordinal whose PCI triple matches target_pci."""
    n = cp.cuda.runtime.getDeviceCount()
    for d in range(n):
        props = cp.cuda.runtime.getDeviceProperties(d)
        if (
            int(props.get("pciDomainID", 0)),
            int(props.get("pciBusID", -1)),
            int(props.get("pciDeviceID", -1)),
        ) == target_pci:
            return d
    return None


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
    def shape(self) -> tuple: ...

    @property
    @abstractmethod
    def dtype(self) -> np.dtype: ...

    @abstractmethod
    def publish(self, frame) -> None:
        """Write into inactive buffer, then flip visible index (write control last)."""
        ...

    @abstractmethod
    def read(self, last_seq: int = -1, require_new: bool = True):
        """Return (seq:int, ts_ns:int, view-or-None)."""
        ...

    @abstractmethod
    def descriptor(self) -> dict:
        """Tiny JSON-safe descriptor (names/handles/shape/dtype/device)."""
        ...

    @classmethod
    @abstractmethod
    def attach(cls, desc: dict) -> "FrameChannel":
        """Attach in another process."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Detach resources (owner also unlinks manager if applicable)."""
        ...


from multiprocessing.shared_memory import SharedMemory
import weakref, os


def _safe_unlink(name):
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
    def __init__(self, shape, dtype=np.uint8):
        self._shape = tuple(shape)
        self._dtype = np.dtype(dtype)
        self._nbytes = int(self._dtype.itemsize * np.prod(self._shape))

        # Create two buffers back-to-back + tiny control block
        self._shm_data = SharedMemory(create=True, size=2 * self._nbytes)
        self._shm_ctrl = SharedMemory(create=True, size=24)
        self._ctrl = np.ndarray((3,), dtype=np.int64, buffer=self._shm_ctrl.buf)
        self._ctrl[:] = 0

        # Owner-only finalizers (in case close() isn’t called)
        self._finalizer_data = weakref.finalize(self, _safe_unlink, self._shm_data.name)
        self._finalizer_ctrl = weakref.finalize(self, _safe_unlink, self._shm_ctrl.name)

    @property
    def device(self):
        return "cpu"

    @property
    def shape(self):
        return self._shape

    @property
    def dtype(self):
        return self._dtype

    def publish(self, frame):
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
        np.copyto(view, frame, casting="no")
        self._ctrl[1] = np.int64(time.time_ns())
        self._ctrl[0] += 1
        self._ctrl[2] = inactive

    def read(self, last_seq: int = -1, require_new=True):
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

    def descriptor(self):
        return {
            "kind": "cpu",
            "shape": self._shape,
            "dtype": self._dtype.str,
            "nbytes": self._nbytes,
            "data_name": self._shm_data.name,
            "ctrl_name": self._shm_ctrl.name,
        }

    @classmethod
    def attach(cls, desc):
        obj = object.__new__(cls)
        obj._shape = tuple(desc["shape"])
        obj._dtype = np.dtype(desc["dtype"])
        obj._nbytes = int(desc["nbytes"])
        obj._shm_data = SharedMemory(name=desc["data_name"])
        obj._shm_ctrl = SharedMemory(name=desc["ctrl_name"])
        obj._ctrl = np.ndarray((3,), dtype=np.int64, buffer=obj._shm_ctrl.buf)
        # attachments don’t own/unlink
        obj._finalizer_data = obj._finalizer_ctrl = None
        return obj

    def close(self):
        # Close our handles
        self._shm_data.close()
        self._shm_ctrl.close()
        # Only the owner should unlink
        if self._finalizer_data:
            _safe_unlink(self._shm_data.name)
            _safe_unlink(self._shm_ctrl.name)
            self._finalizer_data.detach()
            self._finalizer_ctrl.detach()


# ---------------------------
# 3) CUDA IPC backend (CuPy)
# ---------------------------


def _b64(b: bytes) -> str:
    return base64.b64encode(b).decode("ascii")


def _unb64(s: str) -> bytes:
    return base64.b64decode(s.encode("ascii"))


# --- REPLACEMENT: CudaIpcChannel (no SharedMemoryManager, owner-only weakref unlink) ---
class CudaIpcChannel(FrameChannel):
    """CUDA IPC via CuPy. Two device buffers + CPU ctrl shm. No child procs."""

    def __init__(
        self, shape, dtype=np.uint8, device: int = 0, manager: SharedMemoryManager | None = None
    ):
        import cupy as cp

        self._cp = cp
        self._shape = tuple(shape)
        self._dtype = np.dtype(dtype)
        self._device = int(device)

        # Control block in raw SharedMemory (owner unlink via weakref)
        self._shm_ctrl = SharedMemory(create=True, size=24)
        self._ctrl = np.ndarray((3,), dtype=np.int64, buffer=self._shm_ctrl.buf)
        self._ctrl[:] = 0
        self._finalizer_ctrl = weakref.finalize(self, _safe_unlink, self._shm_ctrl.name)

        self._is_owner = True
        self._handles = None  # lazy export
        self._ipc_ptrs = []  # used on attach only

        # Allocate two device buffers
        _ensure_cuda_context(self._cp, self._device)
        with self._cp.cuda.Device(self._device):
            self._bufs = [
                self._cp.empty(self._shape, dtype=self._dtype),
                self._cp.empty(self._shape, dtype=self._dtype),
            ]
            self._nbytes = int(self._bufs[0].nbytes)
            self._h2d_stream = self._cp.cuda.Stream(non_blocking=True)

    @property
    def device(self) -> str:
        return "cuda"

    @property
    def shape(self) -> tuple:
        return self._shape

    @property
    def dtype(self) -> np.dtype:
        return self._dtype

    def publish(self, frame) -> None:
        cp = self._cp
        with cp.cuda.Device(self._device), self._h2d_stream:
            active = int(self._ctrl[2])
            inactive = 1 - active
            dst = self._bufs[inactive]
            if isinstance(frame, cp.ndarray):
                cp.copyto(dst, frame, casting="no")
            else:
                dst.set(frame)
        self._h2d_stream.synchronize()
        self._ctrl[1] = np.int64(time.time_ns())
        self._ctrl[0] += 1
        self._ctrl[2] = inactive

    def read(self, last_seq: int = -1, require_new: bool = True):
        seq1 = int(self._ctrl[0])
        idx = int(self._ctrl[2])
        ts = int(self._ctrl[1])
        view = self._bufs[idx]
        if seq1 == int(self._ctrl[0]):
            if require_new and seq1 == last_seq:
                return seq1, ts, None
            return seq1, ts, view
        return last_seq, 0, None

    def descriptor(self) -> dict:
        if self._handles is None:
            _ensure_cuda_context(self._cp, self._device)
            with self._cp.cuda.Device(self._device):
                h0 = self._cp.cuda.runtime.ipcGetMemHandle(int(self._bufs[0].data.ptr))
                h1 = self._cp.cuda.runtime.ipcGetMemHandle(int(self._bufs[1].data.ptr))
                self._handles = (_b64(h0), _b64(h1))
        pci = _get_pci_triple(self._cp, self._device)
        return {
            "kind": "cuda",
            "shape": self._shape,
            "dtype": self._dtype.str,
            "device": self._device,  # informational; attach will prefer PCI
            "pci": pci,  # <<< new
            "nbytes": self._nbytes,
            "ctrl_name": self._shm_ctrl.name,
            "ptr0": int(self._bufs[0].data.ptr),
            "ptr1": int(self._bufs[1].data.ptr),
            "handle0": self._handles[0],
            "handle1": self._handles[1],
        }

    @classmethod
    def attach(cls, desc: dict) -> "CudaIpcChannel":
        import cupy as cp

        obj = object.__new__(cls)
        obj._cp = cp
        obj._shape = tuple(desc["shape"])
        obj._dtype = np.dtype(desc["dtype"])
        obj._nbytes = int(desc["nbytes"])
        obj._is_owner = False

        # control shm
        obj._shm_ctrl = SharedMemory(name=desc["ctrl_name"])
        obj._ctrl = np.ndarray((3,), dtype=np.int64, buffer=obj._shm_ctrl.buf)
        obj._finalizer_ctrl = None

        # Map to correct local device via PCI (handles CUDA_VISIBLE_DEVICES)
        target_pci = tuple(desc.get("pci", (0, -1, -1)))
        local_dev = _map_pci_to_local_device(cp, target_pci)
        if local_dev is None:
            raise RuntimeError(f"Exported GPU {target_pci} not visible on this worker.")
        obj._device = int(local_dev)

        _ensure_cuda_context(cp, obj._device)
        obj._h2d_stream = cp.cuda.Stream(non_blocking=True)

        def _ptr_accessible(ptr: int) -> bool:
            try:
                with cp.cuda.Device(obj._device):
                    cp.cuda.runtime.pointerGetAttributes(ptr)
                return True
            except Exception:
                return False

        # Prefer RAW POINTER path if present and accessible in this context
        use_ptrs = (
            "ptr0" in desc
            and "ptr1" in desc
            and _ptr_accessible(int(desc["ptr0"]))
            and _ptr_accessible(int(desc["ptr1"]))
        )

        if use_ptrs:
            p0_i = int(desc["ptr0"])
            p1_i = int(desc["ptr1"])
            # No ipcCloseMemHandle needed for raw-ptr mode
            obj._ipc_ptrs = []
            mem0 = cp.cuda.UnownedMemory(p0_i, obj._nbytes, owner=obj)
            mem1 = cp.cuda.UnownedMemory(p1_i, obj._nbytes, owner=obj)
        else:
            # Fallback: open IPC handles
            flags = cp.cuda.runtime.cudaIpcMemLazyEnablePeerAccess
            with cp.cuda.Device(obj._device):
                p0 = cp.cuda.runtime.ipcOpenMemHandle(_unb64(desc["handle0"]), flags)
                p1 = cp.cuda.runtime.ipcOpenMemHandle(_unb64(desc["handle1"]), flags)
            obj._ipc_ptrs = [int(p0), int(p1)]
            mem0 = cp.cuda.UnownedMemory(int(p0), obj._nbytes, owner=obj)
            mem1 = cp.cuda.UnownedMemory(int(p1), obj._nbytes, owner=obj)

        obj._bufs = [
            cp.ndarray(obj._shape, dtype=obj._dtype, memptr=cp.cuda.MemoryPointer(mem0, 0)),
            cp.ndarray(obj._shape, dtype=obj._dtype, memptr=cp.cuda.MemoryPointer(mem1, 0)),
        ]
        return obj

    def close(self) -> None:
        try:
            self._shm_ctrl.close()
        except Exception:
            pass
        if self._is_owner:
            if self._finalizer_ctrl:
                try:
                    _safe_unlink(self._shm_ctrl.name)
                finally:
                    self._finalizer_ctrl.detach()
        else:
            cp = self._cp
            for ptr in getattr(self, "_ipc_ptrs", []):
                try:
                    cp.cuda.runtime.ipcCloseMemHandle(ptr)
                except Exception:
                    pass


# ---------------------------
# 4) Factories
# ---------------------------


class CPU_IPC_Factory:
    """Creates/attaches CPU shared-memory channels."""

    @staticmethod
    def create(shape, dtype=np.uint8) -> CpuShmChannel:
        return CpuShmChannel(shape, dtype=dtype)

    @staticmethod
    def attach(desc: dict) -> CpuShmChannel:
        assert desc.get("kind") == "cpu", "Descriptor kind mismatch"
        return CpuShmChannel.attach(desc)


class CUDA_IPC_Factory:
    """Creates/attaches CUDA IPC channels (CuPy)."""

    @staticmethod
    def is_available(device: int = 0) -> bool:
        try:
            import cupy as cp

            with cp.cuda.Device(device):
                _ = cp.cuda.runtime.getDevice()  # probe
            return True
        except Exception:
            return False

    @staticmethod
    def create(shape, dtype=np.uint8, device: int = 0) -> CudaIpcChannel:
        return CudaIpcChannel(shape, dtype=dtype, device=device)

    @staticmethod
    def attach(desc: dict) -> CudaIpcChannel:
        assert desc.get("kind") == "cuda", "Descriptor kind mismatch"
        return CudaIpcChannel.attach(desc)


# ---------------------------
# 5) Runtime selector
# ---------------------------


def make_frame_channel(
    shape, dtype=np.uint8, prefer: str = "auto", device: int = 0
) -> FrameChannel:
    """Choose CUDA IPC if available (or requested), otherwise CPU SHM."""
    if prefer in ("cuda", "auto"):
        if CUDA_IPC_Factory.is_available(device):
            try:
                return CUDA_IPC_Factory.create(shape, dtype=dtype, device=device)
            except Exception:
                if prefer == "cuda":
                    raise
                # fall back
    return CPU_IPC_Factory.create(shape, dtype=dtype)
