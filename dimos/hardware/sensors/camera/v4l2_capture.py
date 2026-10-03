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

"""Frames off a V4L2 node through the kernel's own streaming ioctls.

OpenCV's V4L2 backend only knows the pixel formats it can turn into BGR, so it
cannot open a depth node: a RealSense camera's Z16 stream rejects the probe
OpenCV starts with. This is the short path underneath it -- set the format and
frame rate, share a few mmap'd buffers with the driver, and dequeue them as
numpy arrays -- with the same ``isOpened``/``read``/``release`` surface as
``cv2.VideoCapture`` so a module can drive either.

Linux only. The structs are declared with ctypes, so their layout follows the
platform's ABI.
"""

from __future__ import annotations

import ctypes
import fcntl
import mmap
import os
import select
from typing import Any

import numpy as np

# fourcc -> (dtype, channels) of one pixel.
FORMATS: dict[str, tuple[type[np.generic], int]] = {
    "Z16 ": (np.uint16, 1),
    "GREY": (np.uint8, 1),
    "YUYV": (np.uint8, 2),
}

_BUF_TYPE_VIDEO_CAPTURE = 1
_MEMORY_MMAP = 1
_BUF_FLAG_ERROR = 0x40


class _PixFormat(ctypes.Structure):
    _fields_ = [
        (name, ctypes.c_uint32)
        for name in (
            "width",
            "height",
            "pixelformat",
            "field",
            "bytesperline",
            "sizeimage",
            "colorspace",
            "priv",
            "flags",
            "ycbcr_enc",
            "quantization",
            "xfer_func",
        )
    ]


class _FormatUnion(ctypes.Union):
    # The kernel union also holds v4l2_window, whose pointers set its alignment.
    _fields_ = [
        ("pix", _PixFormat),
        ("raw_data", ctypes.c_uint8 * 200),
        ("_align", ctypes.c_void_p),
    ]


class _Format(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("fmt", _FormatUnion)]


class _Fract(ctypes.Structure):
    _fields_ = [("numerator", ctypes.c_uint32), ("denominator", ctypes.c_uint32)]


class _CaptureParm(ctypes.Structure):
    _fields_ = [
        ("capability", ctypes.c_uint32),
        ("capturemode", ctypes.c_uint32),
        ("timeperframe", _Fract),
        ("extendedmode", ctypes.c_uint32),
        ("readbuffers", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 4),
    ]


class _ParmUnion(ctypes.Union):
    _fields_ = [("capture", _CaptureParm), ("raw_data", ctypes.c_uint8 * 200)]


class _StreamParm(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("parm", _ParmUnion)]


class _RequestBuffers(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("frames", ctypes.c_uint8),
        ("seconds", ctypes.c_uint8),
        ("minutes", ctypes.c_uint8),
        ("hours", ctypes.c_uint8),
        ("userbits", ctypes.c_uint8 * 4),
    ]


class _BufferM(ctypes.Union):
    _fields_ = [
        ("offset", ctypes.c_uint32),
        ("userptr", ctypes.c_ulong),
        ("planes", ctypes.c_void_p),
        ("fd", ctypes.c_int32),
    ]


class _Buffer(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("timestamp", _Timeval),
        ("timecode", _Timecode),
        ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("m", _BufferM),
        ("length", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("request_fd", ctypes.c_int32),
    ]


def _iowr(nr: int, struct: type[ctypes.Structure]) -> int:
    return (3 << 30) | (ctypes.sizeof(struct) << 16) | (ord("V") << 8) | nr


def _iow_int(nr: int) -> int:
    return (1 << 30) | (ctypes.sizeof(ctypes.c_int) << 16) | (ord("V") << 8) | nr


VIDIOC_S_FMT = _iowr(5, _Format)
VIDIOC_REQBUFS = _iowr(8, _RequestBuffers)
VIDIOC_QUERYBUF = _iowr(9, _Buffer)
VIDIOC_QBUF = _iowr(15, _Buffer)
VIDIOC_DQBUF = _iowr(17, _Buffer)
VIDIOC_STREAMON = _iow_int(18)
VIDIOC_STREAMOFF = _iow_int(19)
VIDIOC_S_PARM = _iowr(22, _StreamParm)


def fourcc_code(fourcc: str) -> int:
    return int.from_bytes(fourcc.ljust(4).encode("ascii"), "little")


class V4L2Capture:
    """One streaming V4L2 capture node, read as numpy frames.

    Opening never raises: a node that is absent, busy, or refuses the mode
    leaves ``isOpened()`` false and the reason in ``error``.
    """

    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: float,
        fourcc: str = "Z16 ",
        n_buffers: int = 4,
        read_timeout_s: float = 1.0,
        start: bool = True,
    ) -> None:
        if fourcc not in FORMATS:
            raise ValueError(f"unsupported fourcc {fourcc!r}; known: {sorted(FORMATS)}")
        self._dtype, self._channels = FORMATS[fourcc]
        self._read_timeout_s = read_timeout_s
        self._fd = -1
        self._maps: list[mmap.mmap] = []
        self._streaming = False
        self.width = width
        self.height = height
        self.fps = fps
        self.error: str | None = None
        try:
            self._configure(device, width, height, fps, fourcc, n_buffers)
            if start:
                self.start()
        except OSError as e:
            self.error = str(e)
            self.release()

    def _configure(
        self, device: str, width: int, height: int, fps: float, fourcc: str, n_buffers: int
    ) -> None:
        self._fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)

        fmt = _Format(type=_BUF_TYPE_VIDEO_CAPTURE)
        fmt.fmt.pix.width = width
        fmt.fmt.pix.height = height
        fmt.fmt.pix.pixelformat = fourcc_code(fourcc)
        fcntl.ioctl(self._fd, VIDIOC_S_FMT, fmt)
        if fmt.fmt.pix.pixelformat != fourcc_code(fourcc):
            raise OSError(f"{device} does not offer {fourcc.strip()}")
        # The driver snaps to the nearest mode it has.
        self.width, self.height = fmt.fmt.pix.width, fmt.fmt.pix.height

        parm = _StreamParm(type=_BUF_TYPE_VIDEO_CAPTURE)
        parm.parm.capture.timeperframe.numerator = 1000
        parm.parm.capture.timeperframe.denominator = round(fps * 1000)
        fcntl.ioctl(self._fd, VIDIOC_S_PARM, parm)
        tpf = parm.parm.capture.timeperframe
        if tpf.numerator:
            self.fps = tpf.denominator / tpf.numerator

        req = _RequestBuffers(count=n_buffers, type=_BUF_TYPE_VIDEO_CAPTURE, memory=_MEMORY_MMAP)
        fcntl.ioctl(self._fd, VIDIOC_REQBUFS, req)
        for index in range(req.count):
            buf = _Buffer(index=index, type=_BUF_TYPE_VIDEO_CAPTURE, memory=_MEMORY_MMAP)
            fcntl.ioctl(self._fd, VIDIOC_QUERYBUF, buf)
            self._maps.append(
                mmap.mmap(
                    self._fd, buf.length, mmap.MAP_SHARED, mmap.PROT_READ, offset=buf.m.offset
                )
            )
            fcntl.ioctl(self._fd, VIDIOC_QBUF, buf)

    def start(self) -> None:
        """Start streaming a node configured with ``start=False``.

        A multi-stream camera such as a RealSense refuses to configure one of
        its streams while another is already running, so a caller opening
        several configures them all first and starts them after.
        """
        fcntl.ioctl(self._fd, VIDIOC_STREAMON, ctypes.c_int(_BUF_TYPE_VIDEO_CAPTURE))
        self._streaming = True

    def fileno(self) -> int:
        return self._fd

    def isOpened(self) -> bool:  # cv2.VideoCapture spelling
        return self._streaming

    def read(self, timeout_s: float | None = None) -> tuple[bool, np.ndarray[Any, Any] | None]:
        """The next frame, or ``(False, None)`` on timeout or a corrupt frame."""
        if not self._streaming:
            return False, None
        timeout = self._read_timeout_s if timeout_s is None else timeout_s
        ready, _, _ = select.select([self._fd], [], [], timeout)
        if not ready:
            return False, None
        buf = _Buffer(type=_BUF_TYPE_VIDEO_CAPTURE, memory=_MEMORY_MMAP)
        try:
            fcntl.ioctl(self._fd, VIDIOC_DQBUF, buf)
        except BlockingIOError:
            return False, None
        try:
            expected = self.width * self.height * self._channels * np.dtype(self._dtype).itemsize
            if buf.flags & _BUF_FLAG_ERROR or buf.bytesused < expected:
                return False, None
            flat = np.frombuffer(
                self._maps[buf.index],
                dtype=self._dtype,
                count=expected // np.dtype(self._dtype).itemsize,
            )
            shape = (
                (self.height, self.width)
                if self._channels == 1
                else (self.height, self.width, self._channels)
            )
            # Copy out before the buffer goes back to the driver.
            return True, flat.reshape(shape).copy()
        finally:
            fcntl.ioctl(self._fd, VIDIOC_QBUF, buf)

    def release(self) -> None:
        if self._streaming:
            try:
                fcntl.ioctl(self._fd, VIDIOC_STREAMOFF, ctypes.c_int(_BUF_TYPE_VIDEO_CAPTURE))
            except OSError:
                pass
            self._streaming = False
        for m in self._maps:
            m.close()
        self._maps.clear()
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1
