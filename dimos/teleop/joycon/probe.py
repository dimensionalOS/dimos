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

"""Print live Joy-Con input: HID labels on macOS, kernel event codes on Linux.

    python -m dimos.teleop.joycon.probe [seconds]

Press each button once and note its code; move each stick to its ends. Put
the codes into the Joy-Con module config when they differ from the defaults
in dimos.teleop.joycon.mapping.
"""

from __future__ import annotations

import os
import select
import struct
import sys
import time

from dimos.teleop.joycon.hid_backend import HidJoyConBackend
from dimos.teleop.joycon.joycon_module import _EVENT_FORMAT, _EVENT_SIZE, input_device_paths


def probe_hid(seconds: float) -> None:
    backend = HidJoyConBackend()
    previous: dict[str, tuple[tuple[str, ...], tuple[float, ...]] | None] = {}
    deadline = time.monotonic() + seconds
    try:
        backend.open()
        print("HID input; press buttons and move sticks; Ctrl-C to stop")
        while time.monotonic() < deadline:
            for side, reading in backend.read().items():
                state = (
                    (tuple(sorted(reading.pressed)), tuple(round(v, 2) for v in reading.stick))
                    if reading is not None
                    else None
                )
                if side not in previous or previous[side] != state:
                    print(f"{side}: {state if state is not None else 'not connected'}")
                    previous[side] = state
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        backend.close()


def main(seconds: float) -> None:
    if sys.platform == "darwin":
        probe_hid(seconds)
        return
    # The driver's IMU device is root-only and carries no buttons.
    devices = {
        name: path
        for name, path in input_device_paths().items()
        if "Joy-Con" in name and "(IMU)" not in name
    }
    if not devices:
        print("no Joy-Con input devices; pair over Bluetooth (hold SYNC on the rail) and retry")
        return
    fds = {}
    for name, path in devices.items():
        try:
            fds[os.open(path, os.O_RDONLY | os.O_NONBLOCK)] = name
        except OSError as exc:
            print(f"{path}: {name}: cannot open ({exc})")
            continue
        print(f"{path}: {name}")
    print("press buttons and move sticks; Ctrl-C to stop")
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            ready, _, _ = select.select(list(fds), [], [], 0.5)
            for fd in ready:
                data = os.read(fd, _EVENT_SIZE * 64)
                for offset in range(0, len(data) - len(data) % _EVENT_SIZE, _EVENT_SIZE):
                    _, _, kind, code, value = struct.unpack_from(_EVENT_FORMAT, data, offset)
                    if kind == 1:
                        print(f"{fds[fd]}: key {code} {'down' if value else 'up'}")
                    elif kind == 3:
                        print(f"{fds[fd]}: axis {code} = {value}")
    except KeyboardInterrupt:
        pass
    finally:
        for fd in fds:
            os.close(fd)


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 120.0)
