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

"""6-DoF 3Dconnexion SpaceMouse teleop module with global keyboard deadman gating.

Reads 6-DoF motion packets from ``spacenavd`` (``/run/spnav.sock``) and/or
directly from the Linux ``evdev`` 3Dconnexion input node
(``/dev/input/by-id/*3Dconnexion*event*``), and gates output strictly on the
global keyboard deadman key (``deadman_key``, ``Ctrl`` by default).

The deadman key state is read via:
1. Linux kernel ``evdev`` ``EVIOCGKEY`` ioctl across all keyboard devices in
   ``/dev/input/event*`` (works globally on **both Wayland and X11**, regardless
   of whether Rerun, Terminal, or another window is focused).
2. X11 ``XQueryKeymap`` fallback on pure X11 sessions.

While the key is held, SpaceMouse deflections are published on ``ee_twist_command``;
when it is released, a zero ``TwistStamped`` is emitted immediately so
``EEFTwistTask`` and the hardware adapter hold the arm locked in the air at its
current pose.
"""

from __future__ import annotations

from collections.abc import Callable
import ctypes
import fcntl
import glob
import os
from pathlib import Path
import select
import socket
import struct
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SPNAV_EVENT_MOTION = 0
SPNAV_EVENT_BUTTON = 1
SPNAV_EVENT_BUTTON_RELEASE = 2
SPNAV_PACKET_FORMAT = "=8i"
SPNAV_PACKET_SIZE = struct.calcsize(SPNAV_PACKET_FORMAT)
DEFAULT_SPNAV_SOCKETS: tuple[str, ...] = ("/run/spnav.sock", "/var/run/spnav.sock")

# Linux <linux/input.h> / <linux/input-event-codes.h> constants (x86_64 / arm64)
EV_SYN = 0x00
EV_KEY = 0x01
EV_REL = 0x02
EV_ABS = 0x03
BTN_0 = 0x100
BTN_1 = 0x101
EVIOCGBIT_EV_4 = 0x80044520  # EVIOCGBIT(0, 4)
EVIOCGBIT_KEY_96 = 0x80604521  # EVIOCGBIT(EV_KEY, 96)
EVIOCGKEY_96 = 0x80604518  # EVIOCGKEY(96)
INPUT_EVENT_FORMAT = "=llHHi"
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FORMAT)

LINUX_KEYCODES: dict[str, int] = {
    "ctrl": 29,
    "leftctrl": 29,
    "rightctrl": 97,
    "x": 45,
    "z": 44,
    "c": 46,
    "v": 47,
    "space": 57,
}

LINUX_KEYCODE_GROUPS: dict[str, tuple[int, ...]] = {
    "ctrl": (29, 97),
    "control": (29, 97),
    "leftctrl": (29,),
    "rightctrl": (97,),
    "x": (45,),
    "z": (44,),
    "c": (46,),
    "v": (47,),
    "space": (57,),
}


class SpaceMouseTeleopConfig(ModuleConfig):
    """Configuration for 3Dconnexion SpaceMouse teleoperation."""

    frame_id: str = "spacemouse_teleop"
    socket_path: str = "/run/spnav.sock"
    linear_speed: float = 0.15
    angular_speed: float = 0.60
    full_scale: float = 350.0
    deadzone: float = 0.04
    send_rate_hz: float = 50.0
    motion_timeout_s: float = 0.15
    deadman_key: str = "ctrl"
    # When True, starts in ARM Mode (only linear X/Y/Z translations are sent,
    # angular locked to 0). Clicking the SpaceMouse Left Button (Button 0)
    # switches sequentially to GRIPPER Mode (only angular tilt/pan/roll is sent,
    # linear locked to 0 so the arm stays still while tilting the gripper).
    translation_only: bool = False
    decouple_arm_and_gripper: bool = True
    # Map spacenavd / evdev (x, y, z, rx, ry, rz) axes to robot base_link:
    # axis_order = (2, 0, 1, 5, 3, 4) -> (z, x, y, rz, rx, ry)
    # Note: rx > 0 (tilting puck forward/down) maps to +angular.y (pitching gripper down).
    axis_signs: tuple[float, float, float, float, float, float] = (
        -1.0,
        -1.0,
        1.0,
        -1.0,
        1.0,
        1.0,
    )
    axis_order: tuple[int, int, int, int, int, int] = (2, 0, 1, 5, 3, 4)


def _test_bit(buf: bytes | bytearray, bit: int) -> bool:
    byte_idx = bit >> 3
    if byte_idx >= len(buf):
        return False
    return bool(buf[byte_idx] & (1 << (bit & 7)))


class _EvdevKeyPoller:
    """Global physical key state reader via Linux ``evdev`` ``EVIOCGKEY`` ioctl.

    Queries the kernel's live key-down bitmap across all keyboard devices in
    ``/dev/input/event*`` without grabbing the device, so it works globally on
    **both Wayland and X11** regardless of which window has focus.
    """

    def __init__(self, key_symbol: str = "ctrl") -> None:
        sym = key_symbol.lower()
        self._keycodes = LINUX_KEYCODE_GROUPS.get(sym, (LINUX_KEYCODES.get(sym, 29),))
        self._keycode = self._keycodes[0]
        self._fds: dict[str, int] = {}
        self._last_scan_ts: float = 0.0
        self._warned_perm: bool = False
        self.scan_devices()

    @property
    def device_paths(self) -> list[str]:
        return sorted(self._fds.keys())

    def scan_devices(self) -> None:
        self._last_scan_ts = time.monotonic()
        perm_denied = False
        for path in sorted(glob.glob("/dev/input/event*")):
            if path in self._fds:
                continue
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except PermissionError:
                perm_denied = True
                continue
            except OSError:
                continue

            try:
                ev_bits = bytearray(4)
                fcntl.ioctl(fd, EVIOCGBIT_EV_4, ev_bits, True)
                if not _test_bit(ev_bits, EV_KEY):
                    os.close(fd)
                    continue
                key_bits = bytearray(96)
                fcntl.ioctl(fd, EVIOCGBIT_KEY_96, key_bits, True)
                if not any(_test_bit(key_bits, code) for code in self._keycodes):
                    os.close(fd)
                    continue
                self._fds[path] = fd
            except OSError:
                try:
                    os.close(fd)
                except OSError:
                    pass

        if self._fds:
            self._warned_perm = False
        elif perm_denied and not self._warned_perm:
            self._warned_perm = True
            logger.warning(
                "SpaceMouseTeleopModule: /dev/input/event* is not readable by current user; "
                "run `sudo setfacl -m u:$USER:r /dev/input/event*` for global Wayland key polling"
            )

    def is_pressed(self) -> bool:
        now = time.monotonic()
        if not self._fds and (now - self._last_scan_ts) >= 2.0:
            self.scan_devices()

        buf = bytearray(96)
        dead_paths: list[str] = []
        pressed = False
        for path, fd in self._fds.items():
            try:
                fcntl.ioctl(fd, EVIOCGKEY_96, buf, True)
                if any(_test_bit(buf, code) for code in self._keycodes):
                    pressed = True
                    break
            except OSError:
                dead_paths.append(path)

        for path in dead_paths:
            fd = self._fds.pop(path, -1)
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass
        return pressed

    def close(self) -> None:
        for fd in self._fds.values():
            try:
                os.close(fd)
            except OSError:
                pass
        self._fds.clear()


class _X11KeyPoller:
    """Fallback X11 key state reader via ``XQueryKeymap`` (for pure X11 sessions)."""

    def __init__(self, key_symbol: str = "ctrl") -> None:
        self._key_symbol = key_symbol
        self._x11: Any = None
        self._display: int | None = None
        self._keycodes: tuple[int, ...] = ()
        self._open()

    def _open(self) -> None:
        if os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland":
            # Under Wayland, XWayland does not receive key state from native Wayland windows.
            return
        try:
            x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
            x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
            x11.XOpenDisplay.restype = ctypes.c_void_p
            x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
            x11.XCloseDisplay.restype = ctypes.c_int
            x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
            x11.XStringToKeysym.restype = ctypes.c_ulong
            x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
            x11.XKeysymToKeycode.restype = ctypes.c_ubyte
            x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
            x11.XQueryKeymap.restype = ctypes.c_int

            display = x11.XOpenDisplay(None)
            if not display:
                return
            sym_lower = self._key_symbol.lower()
            syms = (
                ("Control_L", "Control_R")
                if sym_lower in ("ctrl", "control")
                else (self._key_symbol,)
            )
            codes: list[int] = []
            for sym in syms:
                keysym = x11.XStringToKeysym(sym.encode("ascii"))
                kc = int(x11.XKeysymToKeycode(display, keysym))
                if kc > 0:
                    codes.append(kc)
            if not codes:
                x11.XCloseDisplay(display)
                return
            self._x11 = x11
            self._display = int(display)
            self._keycodes = tuple(codes)
        except Exception:
            pass

    def is_pressed(self) -> bool:
        if self._x11 is None or self._display is None or not self._keycodes:
            return False
        buf = (ctypes.c_char * 32)()
        self._x11.XQueryKeymap(self._display, buf)
        for keycode in self._keycodes:
            byte_index = keycode >> 3
            bit_mask = 1 << (keycode & 7)
            raw_byte = buf[byte_index]
            val = raw_byte[0] if isinstance(raw_byte, (bytes, bytearray)) else ord(raw_byte)
            if val & bit_mask:
                return True
        return False

    def close(self) -> None:
        if self._x11 is not None and self._display is not None:
            try:
                self._x11.XCloseDisplay(self._display)
            except Exception:
                pass
        self._display = None
        self._x11 = None


def parse_spnav_packet(
    packet: bytes,
) -> tuple[int, int, int, int, int, int] | None:
    """Decode one 32-byte ``spacenavd`` motion packet.

    Returns ``(x, y, z, rx, ry, rz)`` for ``SPNAV_EVENT_MOTION`` packets, or
    ``None`` for non-motion packets.
    """
    if len(packet) != SPNAV_PACKET_SIZE:
        raise ValueError(f"Expected {SPNAV_PACKET_SIZE}-byte spnav packet, got {len(packet)}")
    ev_type, x, y, z, rx, ry, rz, _period = struct.unpack(SPNAV_PACKET_FORMAT, packet)
    if ev_type != SPNAV_EVENT_MOTION:
        return None
    return (x, y, z, rx, ry, rz)


def parse_spnav_button_packet(packet: bytes) -> tuple[int, bool] | None:
    """Decode one 32-byte ``spacenavd`` button packet.

    In ``spacenavd``'s unix socket protocol (``proto_unix.c``):
    - ``packet[0]``: ``1`` (``UEV_PRESS``) on press, ``2`` (``UEV_RELEASE``) on release
    - ``packet[1]``: ``bnum`` (``0`` = Left button, ``1`` = Right button)
    - ``packet[2]``: ``press`` (``1`` = pressed, ``0`` = released)

    Returns ``(button_number, pressed)`` for button packets, or ``None`` otherwise.
    """
    if len(packet) != SPNAV_PACKET_SIZE:
        raise ValueError(f"Expected {SPNAV_PACKET_SIZE}-byte spnav packet, got {len(packet)}")
    ev_type, bnum, press, *_ = struct.unpack(SPNAV_PACKET_FORMAT, packet)
    if ev_type == SPNAV_EVENT_BUTTON:
        return (int(bnum), bool(press) if press in (0, 1) else True)
    if ev_type == SPNAV_EVENT_BUTTON_RELEASE:
        return (int(bnum), False)
    return None


class _EvdevSpaceMouseButtonPoller:
    """Direct kernel ``EVIOCGKEY`` button state poller for 3Dconnexion SpaceMouse.

    Reads ``BTN_0`` (Left button = 256) and ``BTN_1`` (Right button = 257)
    directly from the Linux kernel key bitmap on
    ``/dev/input/by-id/*3Dconnexion*event*`` without consuming the event stream,
    so it works even while ``spacenavd`` holds the device open.
    """

    def __init__(self) -> None:
        self._fd: int = -1
        self._prev: tuple[bool, bool] = (False, False)
        self._last_open_ts: float = 0.0
        self._open()

    def _open(self) -> None:
        self._last_open_ts = time.monotonic()
        for path in sorted(glob.glob("/dev/input/by-id/*3Dconnexion*event*")):
            try:
                self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                return
            except OSError:
                continue

    def poll_rising_edges(self) -> tuple[bool, bool]:
        now = time.monotonic()
        if self._fd < 0 and (now - self._last_open_ts) >= 2.0:
            self._open()
        if self._fd < 0:
            return (False, False)
        buf = bytearray(96)
        try:
            fcntl.ioctl(self._fd, EVIOCGKEY_96, buf, True)
        except OSError:
            self.close()
            return (False, False)
        b0 = _test_bit(buf, BTN_0)
        b1 = _test_bit(buf, BTN_1)
        prev0, prev1 = self._prev
        self._prev = (b0, b1)
        return (b0 and not prev0, b1 and not prev1)

    def close(self) -> None:
        if self._fd >= 0:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = -1
        self._prev = (False, False)


class SpaceMouseTeleopModule(Module):
    """6-DoF SpaceMouse teleoperation gated strictly by the global deadman key."""

    config: SpaceMouseTeleopConfig
    default_config = SpaceMouseTeleopConfig
    ee_twist_command: Out[TwistStamped]
    gripper_command: Out[Float32]

    def __init__(
        self,
        *args: Any,
        deadman_fn: Callable[[], bool] | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self._deadman_fn = deadman_fn
        self._evdev_key_poller: _EvdevKeyPoller | None = None
        self._x11_key_poller: _X11KeyPoller | None = None
        self._evdev_button_poller: _EvdevSpaceMouseButtonPoller | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._raw_axes: tuple[int, int, int, int, int, int] = (0, 0, 0, 0, 0, 0)
        self._evdev_axes: list[int] = [0, 0, 0, 0, 0, 0]
        self._last_motion_ts: float = 0.0
        self._deadman_active: bool = False
        self._logged_motion_in_engagement: bool = False
        self._translation_only: bool = bool(self.config.translation_only)
        # When started with translation_only=True, Left Button toggles sequentially
        # between ARM Mode (linear-only) and GRIPPER Mode (angular-only).
        self._decouple_arm_and_gripper: bool = bool(self.config.decouple_arm_and_gripper) and bool(
            self.config.translation_only
        )
        self._gripper_open: bool = True
        self._last_button_press_ts: dict[int, float] = {}

    @rpc
    def start(self) -> None:
        super().start()
        if self._thread is not None and self._thread.is_alive():
            return
        self._translation_only = bool(self.config.translation_only)
        self._decouple_arm_and_gripper = bool(self.config.decouple_arm_and_gripper) and bool(
            self.config.translation_only
        )
        if self._deadman_fn is None:
            self._evdev_key_poller = _EvdevKeyPoller(self.config.deadman_key)
            self._x11_key_poller = _X11KeyPoller(self.config.deadman_key)
            self._evdev_button_poller = _EvdevSpaceMouseButtonPoller()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop,
            name="spacemouse-teleop",
            daemon=True,
        )
        self._thread.start()
        logger.info(
            "SpaceMouseTeleopModule started",
            socket=self.config.socket_path,
            deadman_key=self.config.deadman_key.upper(),
            mode="ARM (Translation-Only)" if self._translation_only else "6dof",
            evdev_keyboards=(
                self._evdev_key_poller.device_paths if self._evdev_key_poller is not None else []
            ),
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._evdev_key_poller is not None:
            self._evdev_key_poller.close()
            self._evdev_key_poller = None
        if self._x11_key_poller is not None:
            self._x11_key_poller.close()
            self._x11_key_poller = None
        if self._evdev_button_poller is not None:
            self._evdev_button_poller.close()
            self._evdev_button_poller = None
        self._publish_zero_twist()
        super().stop()

    def _is_deadman_pressed(self, now: float | None = None) -> bool:
        if self._deadman_fn is not None:
            return bool(self._deadman_fn())
        if self._evdev_key_poller is not None and self._evdev_key_poller.is_pressed():
            return True
        if self._x11_key_poller is not None and self._x11_key_poller.is_pressed():
            return True
        return False

    def on_button_event(self, button_num: int, pressed: bool, *, debounce_s: float = 0.0) -> None:
        """Handle SpaceMouse side button presses (Button 0: Arm/Gripper toggle, Button 1: Open/Close)."""
        if not pressed:
            return
        now = time.monotonic()
        with self._lock:
            last_ts = self._last_button_press_ts.get(button_num, -1e9)
            if debounce_s > 0.0 and (now - last_ts) < debounce_s:
                return
            self._last_button_press_ts[button_num] = now
            if button_num == 0:
                self._translation_only = not self._translation_only
                self._logged_motion_in_engagement = False
                if self._decouple_arm_and_gripper:
                    mode = (
                        "ARM Mode (move arm X/Y/Z - gripper tilt locked)"
                        if self._translation_only
                        else "GRIPPER Mode (tilt/pan/roll gripper only - arm stays still)"
                    )
                else:
                    mode = (
                        "Translation-Only (XYZ)"
                        if self._translation_only
                        else "6-DoF (XYZ + Rotation)"
                    )
        if button_num == 0:
            logger.info(f"SpaceMouse Left Button pressed - switched to {mode}")
        elif button_num == 1:
            self._gripper_open = not self._gripper_open
            opening = 1.0 if self._gripper_open else 0.0
            state_label = "OPEN (1.0)" if self._gripper_open else "CLOSE (0.0)"
            logger.info(f"SpaceMouse Right Button pressed - Gripper {state_label}")
            try:
                if self.gripper_command.transport is not None:
                    self.gripper_command.publish(Float32(data=opening))
            except Exception:
                pass

    def ingest_motion(
        self, axes: tuple[int, int, int, int, int, int], now: float | None = None
    ) -> None:
        """Record a 6-DoF SpaceMouse deflection sample ``(x, y, z, rx, ry, rz)``."""
        ts = time.monotonic() if now is None else now
        with self._lock:
            self._raw_axes = axes
            self._last_motion_ts = ts

    def compute_twist(self, now: float | None = None) -> tuple[TwistStamped, bool]:
        """Compute the gated ``TwistStamped`` for the current deadman and SpaceMouse state.

        Returns ``(twist_msg, deadman_pressed)``. Whenever the deadman key is
        not pressed, the returned twist is strictly zero.
        """
        ts_mono = time.monotonic() if now is None else now
        pressed = self._is_deadman_pressed(now=ts_mono)
        if not pressed:
            return self._zero_twist(), False

        with self._lock:
            axes = self._raw_axes
            age = ts_mono - self._last_motion_ts
            translation_only = self._translation_only
            decouple = self._decouple_arm_and_gripper

        if age > self.config.motion_timeout_s:
            axes = (0, 0, 0, 0, 0, 0)

        scale = max(self.config.full_scale, 1.0)
        ordered = [axes[idx] for idx in self.config.axis_order]
        normalized: list[float] = []
        for val, sign in zip(ordered, self.config.axis_signs, strict=True):
            unit = max(-1.0, min(1.0, (float(val) / scale) * sign))
            if abs(unit) < self.config.deadzone:
                unit = 0.0
            normalized.append(unit)

        if translation_only:
            # Mode 1: ARM Mode - move only arm X/Y/Z, lock gripper orientation.
            lin = self.config.linear_speed
            return (
                TwistStamped(
                    ts=time.time(),
                    frame_id=self.config.frame_id,
                    linear=Vector3(
                        normalized[0] * lin,
                        normalized[1] * lin,
                        normalized[2] * lin,
                    ),
                    angular=Vector3(0.0, 0.0, 0.0),
                ),
                True,
            )

        if decouple:
            # Mode 2: GRIPPER Mode - arm stays still (linear = 0), control only gripper tilt/pan/roll.
            # Allow tilting the gripper down/up via either puck pitch tilt (normalized[4]),
            # puck forward/back push (normalized[0]), or puck down/up push (-normalized[2]).
            ang = self.config.angular_speed
            pitch_candidates = (normalized[4], normalized[0], -normalized[2])
            pitch_unit = max(pitch_candidates, key=abs)
            yaw_candidates = (normalized[5], normalized[1])
            yaw_unit = max(yaw_candidates, key=abs)
            roll_unit = normalized[3]
            return (
                TwistStamped(
                    ts=time.time(),
                    frame_id=self.config.frame_id,
                    linear=Vector3(0.0, 0.0, 0.0),
                    angular=Vector3(
                        roll_unit * ang,
                        pitch_unit * ang,
                        yaw_unit * ang,
                    ),
                ),
                True,
            )

        lin = self.config.linear_speed
        ang = self.config.angular_speed
        return (
            TwistStamped(
                ts=time.time(),
                frame_id=self.config.frame_id,
                linear=Vector3(
                    normalized[0] * lin,
                    normalized[1] * lin,
                    normalized[2] * lin,
                ),
                angular=Vector3(
                    normalized[3] * ang,
                    normalized[4] * ang,
                    normalized[5] * ang,
                ),
            ),
            True,
        )

    def step_once(self, now: float | None = None) -> TwistStamped | None:
        """Run one deadman-gated control tick and publish if active or just released."""
        twist, pressed = self.compute_twist(now=now)
        key_label = self.config.deadman_key.upper()
        if pressed:
            if not self._deadman_active:
                self._deadman_active = True
                self._logged_motion_in_engagement = False
                logger.info(
                    f"SpaceMouse teleop ENGAGED ({key_label} held) - move 3Dconnexion puck to jog arm"
                )
            if not self._logged_motion_in_engagement and (
                abs(twist.linear.x) > 0.0
                or abs(twist.linear.y) > 0.0
                or abs(twist.linear.z) > 0.0
                or abs(twist.angular.x) > 0.0
                or abs(twist.angular.y) > 0.0
                or abs(twist.angular.z) > 0.0
            ):
                self._logged_motion_in_engagement = True
                logger.info(
                    "SpaceMouse teleop publishing twist",
                    linear=(
                        round(twist.linear.x, 3),
                        round(twist.linear.y, 3),
                        round(twist.linear.z, 3),
                    ),
                    angular=(
                        round(twist.angular.x, 3),
                        round(twist.angular.y, 3),
                        round(twist.angular.z, 3),
                    ),
                )
            self.ee_twist_command.publish(twist)
            return twist
        if self._deadman_active:
            # Deadman key was just released: immediately send zero twist to lock the arm in place.
            self._deadman_active = False
            self._logged_motion_in_engagement = False
            logger.info(f"SpaceMouse teleop LOCKED ({key_label} released) - holding arm pose")
            self.ee_twist_command.publish(twist)
            return twist
        return None

    def _zero_twist(self) -> TwistStamped:
        return TwistStamped(
            ts=time.time(),
            frame_id=self.config.frame_id,
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )

    def _publish_zero_twist(self) -> None:
        try:
            if self.ee_twist_command.transport is not None:
                self.ee_twist_command.publish(self._zero_twist())
        except Exception:
            pass

    def _connect_spnav_socket(self) -> socket.socket | None:
        candidates = (self.config.socket_path, *DEFAULT_SPNAV_SOCKETS)
        for path in dict.fromkeys(candidates):
            if not Path(path).exists():
                continue
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(path)
                sock.setblocking(False)
                logger.info(f"Connected to spacenavd socket at {path}")
                return sock
            except OSError as exc:
                logger.warning(f"Failed to connect to {path}: {exc}")
        return None

    def _open_evdev_spacemouse(self) -> int | None:
        for path in sorted(glob.glob("/dev/input/by-id/*3Dconnexion*event*")):
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                logger.info(f"Opened 3Dconnexion evdev fallback device at {path}")
                return fd
            except OSError:
                continue
        return None

    def _drain_evdev_spacemouse(self, fd: int, buf: bytearray) -> bool:
        try:
            while True:
                ready, _, _ = select.select([fd], [], [], 0.0)
                if not ready:
                    break
                chunk = os.read(fd, 256)
                if not chunk:
                    return False
                buf.extend(chunk)
                updated = False
                while len(buf) >= INPUT_EVENT_SIZE:
                    raw = bytes(buf[:INPUT_EVENT_SIZE])
                    del buf[:INPUT_EVENT_SIZE]
                    _sec, _usec, ev_type, code, value = struct.unpack(INPUT_EVENT_FORMAT, raw)
                    if ev_type in (EV_REL, EV_ABS) and 0 <= code < 6:
                        self._evdev_axes[code] = int(value)
                        updated = True
                    elif ev_type == EV_KEY and code in (BTN_0, BTN_1):
                        self.on_button_event(code - BTN_0, bool(value), debounce_s=0.15)
                    elif ev_type == EV_SYN and updated:
                        tx, ty, tz, rx, ry, rz = self._evdev_axes
                        # Map raw HID (x=right, y=back, z=down, rx=pitch, ry=roll, rz=yaw)
                        # to spacenavd convention (x=right, y=up, z=back, rx=pitch, ry=yaw, rz=roll).
                        self.ingest_motion((tx, -tz, ty, rx, -rz, -ry))
                        self._evdev_axes = [0, 0, 0, 0, 0, 0]
                        updated = False
        except BlockingIOError:
            return True
        except OSError:
            return False
        return True

    def _drain_socket(self, sock: socket.socket, buf: bytearray) -> bool:
        try:
            while True:
                ready, _, _ = select.select([sock], [], [], 0.0)
                if not ready:
                    break
                chunk = sock.recv(256)
                if not chunk:
                    return False
                buf.extend(chunk)
                while len(buf) >= SPNAV_PACKET_SIZE:
                    packet = bytes(buf[:SPNAV_PACKET_SIZE])
                    del buf[:SPNAV_PACKET_SIZE]
                    motion = parse_spnav_packet(packet)
                    if motion is not None:
                        self.ingest_motion(motion)
                        continue
                    button = parse_spnav_button_packet(packet)
                    if button is not None:
                        self.on_button_event(button[0], button[1], debounce_s=0.15)
        except BlockingIOError:
            return True
        except OSError:
            return False
        return True

    def _run_loop(self) -> None:
        period = 1.0 / max(self.config.send_rate_hz, 1.0)
        sock: socket.socket | None = None
        evdev_fd: int | None = None
        buf = bytearray()
        evdev_buf = bytearray()
        last_connect_attempt = 0.0

        try:
            while not self._stop_event.is_set():
                now = time.monotonic()
                if sock is None and evdev_fd is None and (now - last_connect_attempt) >= 2.0:
                    last_connect_attempt = now
                    sock = self._connect_spnav_socket()
                    buf.clear()
                    if sock is None:
                        evdev_fd = self._open_evdev_spacemouse()
                        evdev_buf.clear()

                if sock is not None and not self._drain_socket(sock, buf):
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = None
                    buf.clear()

                if (
                    sock is None
                    and evdev_fd is not None
                    and not self._drain_evdev_spacemouse(evdev_fd, evdev_buf)
                ):
                    try:
                        os.close(evdev_fd)
                    except OSError:
                        pass
                    evdev_fd = None
                    evdev_buf.clear()

                if self._evdev_button_poller is not None:
                    left_edge, right_edge = self._evdev_button_poller.poll_rising_edges()
                    if left_edge:
                        self.on_button_event(0, True, debounce_s=0.15)
                    if right_edge:
                        self.on_button_event(1, True, debounce_s=0.15)

                self.step_once(now=time.monotonic())
                self._stop_event.wait(period)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
            if evdev_fd is not None:
                try:
                    os.close(evdev_fd)
                except OSError:
                    pass


spacemouse_teleop_module = SpaceMouseTeleopModule.blueprint
