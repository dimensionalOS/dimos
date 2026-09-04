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

"""SDK-free ZED-M IMU at the full ~800 Hz (plain HID, no pyzed).

The ZED-M's 6-DoF IMU is reachable over a USB HID interface (vid 0x2b03,
pid 0xf681) without the ZED SDK. The catch is *how the stream is started*:
the open-source ``zed-open-capture`` path enables the stream with a feature
report and then pings the MCU every 250 ms to keep it alive, and on this
firmware that path suffers sporadic ~1 s stream suspensions. The official SDK
instead sends two output reports at open and then reads with no keepalive at
all, and never drops. This module replays that SDK handshake (captured off the
wire with usbmon) so we get the SDK's gap-free 800 Hz stream with no SDK, no
CUDA, and no ping thread.

On Linux (hidraw) the full 800 Hz arrives regardless of topology. On Apple
Silicon Macs, plugged straight into a root port, only ~533 Hz arrives: the XHCI
under-services full-speed interrupt endpoints, silently losing ~1/3 of reports
below hidapi (the device still samples at 800 Hz — its MCU ticks stay a clean
1.25 ms apart). Neither hidapi, direct IOHIDManager reads, nor forcing the
IOKit ``ReportInterval`` property recovers the loss; it happens in the kernel.
This is a known M-series quirk, not specific to the ZED: full-speed
``bInterval=1`` devices cap at ~500 Hz on Apple Silicon but reach 1000 Hz on
Intel Macs and Windows (https://github.com/hathach/tinyusb/issues/1705).

Workaround: plug the ZED through a USB hub. The hub's transaction translator
(see e.g. https://blog.adafruit.com/2025/11/03/a-usb-2-hub-with-per-port-power-and-mtt/)
schedules full-speed polls on a strict 1 ms frame cadence, restoring the full
rate — measured 795 Hz, gap-free, behind a USB 3.0 hub.

Wire format and scale constants come from ``zed-open-capture``'s ``RawData``
struct (packed, little-endian, no report-id prefix on the ZED-M): the IMU is
±8 g / ±1000 deg/s over int16, timestamped in 39.0625 µs MCU ticks. Factory
accel calibration is SDK-only, so accel is delivered raw-scaled — on this rig
it reads a few percent high; calibrate downstream if needed.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import sys
import threading
import time

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ZED_VID = 0x2B03
ZED_HID_PIDS = (0xF681, 0xF680)  # f681 normally; f680 when the SDK holds the camera

# RawData conversions (zed-open-capture sensorcapture_def.hpp)
_DEG2RAD = 3.141592653589793 / 180.0
_ACC_SCALE = 9.81 * (8.0 / 32768.0)  # int16 -> m/s^2  (±8 g)
_GYRO_SCALE = (1000.0 / 32768.0) * _DEG2RAD  # int16 -> rad/s  (±1000 deg/s)
_TS_TICK_NS = 39062.5  # MCU timestamp tick -> nanoseconds

# SDK stream-start handshake (output reports), from the usbmon capture.
_REP_START_A = bytes([0x22, 0xE1] + [0x00] * 63)
_REP_START_B = bytes([0x23] + [0x00] * 63 + [0xFF])
_START_GAP_S = 0.7  # SDK waits ~0.7 s between the two reports


# macOS ships hidapi with the IOKit backend (libhidapi.dylib); Linux uses the
# libusb backend (libhidapi-libusb.so.0). Try the platform-appropriate names,
# then fall back to whatever ctypes can locate. Homebrew's /opt/homebrew/lib is
# not on the default dlopen search path, so try those absolute paths too.
_HIDAPI_NAMES = (
    (
        "libhidapi.dylib",
        "libhidapi-iohidmanager.dylib",
        "libhidapi-libusb.dylib",
        "/opt/homebrew/lib/libhidapi.dylib",
        "/usr/local/lib/libhidapi.dylib",
    )
    if sys.platform == "darwin"
    else ("libhidapi-libusb.so.0", "libhidapi-hidraw.so.0")
)


def _open_hidapi_cdll() -> ctypes.CDLL:
    for name in _HIDAPI_NAMES:
        try:
            return ctypes.CDLL(name)
        except OSError:
            continue
    found = ctypes.util.find_library("hidapi")
    if found:
        return ctypes.CDLL(found)
    raise RuntimeError(
        "hidapi shared library not found. Install it (macOS: `brew install hidapi`, "
        "Linux: `apt install libhidapi-libusb0`)."
    )


def _load_hidapi() -> ctypes.CDLL:
    lib = _open_hidapi_cdll()
    lib.hid_open.restype = ctypes.c_void_p
    lib.hid_open.argtypes = [ctypes.c_ushort, ctypes.c_ushort, ctypes.c_wchar_p]
    lib.hid_write.restype = ctypes.c_int
    lib.hid_write.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
    lib.hid_read_timeout.restype = ctypes.c_int
    lib.hid_read_timeout.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_size_t,
        ctypes.c_int,
    ]
    lib.hid_close.argtypes = [ctypes.c_void_p]
    return lib


class ZedImuConfig(ModuleConfig):
    frame_id: str = "zed_imu_link"
    read_timeout_ms: int = Field(default=1000, gt=0)
    # Log a warning when consecutive samples are farther apart than this — a
    # real firmware suspension would be ~1000 ms; normal jitter is < 5 ms.
    gap_warn_ms: float = Field(default=200.0, gt=0.0)


class ZedImu(Module):
    """Publishes the ZED-M onboard IMU as an ``Imu`` stream at ~800 Hz."""

    config: ZedImuConfig

    zed_imu: Out[Imu]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._lib: ctypes.CDLL | None = None
        self._dev: int | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def _open(self) -> None:
        self._lib = _load_hidapi()
        for pid in ZED_HID_PIDS:
            handle = self._lib.hid_open(ZED_VID, pid, None)
            if handle:
                self._dev = handle
                break
        if not self._dev:
            raise RuntimeError(
                f"No ZED HID device found (vid 0x{ZED_VID:04x}, pids "
                + "/".join(f"0x{p:04x}" for p in ZED_HID_PIDS)
                + "). Is the ZED-M plugged in and the udev rule installed?"
            )
        # SDK stream-start handshake: two output reports, ~0.7 s apart.
        if self._lib.hid_write(self._dev, _REP_START_A, len(_REP_START_A)) < 0:
            raise RuntimeError("ZED IMU: stream-start report A failed")
        time.sleep(_START_GAP_S)
        if self._lib.hid_write(self._dev, _REP_START_B, len(_REP_START_B)) < 0:
            raise RuntimeError("ZED IMU: stream-start report B failed")

    @rpc
    def start(self) -> None:
        super().start()
        if self._thread and self._thread.is_alive():
            return
        self._open()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        assert self._lib is not None and self._dev is not None
        buf = ctypes.create_string_buffer(64)
        anchor: tuple[int, int] | None = None  # (host_ns, tick) of the first sample
        last_ts_ns: int | None = None
        gap_warn_ns = self.config.gap_warn_ms * 1e6
        while not self._stop_event.is_set():
            n = self._lib.hid_read_timeout(self._dev, buf, 64, self.config.read_timeout_ms)
            if n <= 0:
                if self._stop_event.is_set():
                    break
                logger.warning("ZED IMU: read %s", "timeout" if n == 0 else f"error ({n})")
                continue
            raw = buf.raw
            # struct_id==0 & imu_not_valid==0 → a valid sensor sample; the stream
            # interleaves other report types (status/env frames) we skip here.
            if n < 22 or raw[0] != 0 or raw[1] != 0:
                continue
            tick = int.from_bytes(raw[2:10], "little", signed=False)
            gx, gy, gz, ax, ay, az = (
                int.from_bytes(raw[o : o + 2], "little", signed=True) for o in range(10, 22, 2)
            )
            # Anchor MCU ticks to host wall-clock at the first sample and advance
            # by MCU deltas: epoch-aligned, smooth, and preserving real gaps.
            if anchor is None:
                anchor = (time.time_ns(), tick)
            ts_ns = anchor[0] + int((tick - anchor[1]) * _TS_TICK_NS)

            if last_ts_ns is not None and (ts_ns - last_ts_ns) > gap_warn_ns:
                logger.warning("ZED IMU: %.0f ms gap", (ts_ns - last_ts_ns) / 1e6)
            last_ts_ns = ts_ns

            self.zed_imu.publish(
                Imu(
                    angular_velocity=Vector3(gx * _GYRO_SCALE, gy * _GYRO_SCALE, gz * _GYRO_SCALE),
                    linear_acceleration=Vector3(ax * _ACC_SCALE, ay * _ACC_SCALE, az * _ACC_SCALE),
                    frame_id=self.config.frame_id,
                    ts=ts_ns / 1e9,
                )
            )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._thread = None
        if self._lib and self._dev:
            self._lib.hid_close(self._dev)
            self._dev = None
        super().stop()
