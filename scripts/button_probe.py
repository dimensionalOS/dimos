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

"""Live decoded view of the Quest button stream.

Prints a line whenever any digital button changes, so pressing buttons on
either controller shows exactly what reaches the laptop.

    uv run python scripts/button_probe.py
"""

from __future__ import annotations

import time

from dimos.core.transport_factory import make_transport
from dimos.teleop.quest.quest_types import Buttons

WATCH = (
    "left_primary",
    "left_secondary",
    "left_trigger",
    "left_grip",
    "right_primary",
    "right_secondary",
    "right_trigger",
    "right_grip",
)

_last: dict[str, bool] = {}


def on_msg(msg: Buttons) -> None:
    state = {name: bool((int(msg.data) >> Buttons.BITS[name]) & 1) for name in WATCH}
    if state != _last:
        _last.clear()
        _last.update(state)
        pressed = [name for name, value in state.items() if value]
        print(f"{time.strftime('%H:%M:%S')} pressed: {pressed or ['(none)']}")


def main() -> None:
    transport = make_transport("teleop_buttons", Buttons)
    transport.subscribe(on_msg)
    print("Watching Quest buttons... press buttons on each controller. Ctrl-C to stop.")
    while True:
        time.sleep(0.5)


if __name__ == "__main__":
    main()
