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

"""Print every joystick pygame sees and its live button and axis values.

    python -m dimos.teleop.joycon.probe [seconds]

Press each Joy-Con button once and note its index; move each stick to its
ends. Put the indices into the Joy-Con module config when they differ from
the defaults in dimos.teleop.joycon.mapping.
"""

from __future__ import annotations

import importlib
import os
import sys
import time
from typing import Any


def main(seconds: float) -> None:
    os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "0")
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame: Any = importlib.import_module("pygame")

    pygame.init()
    pygame.joystick.init()
    sticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    if not sticks:
        print("no joysticks; pair the Joy-Cons over bluetoothctl and press a button")
        return
    for stick in sticks:
        stick.init()
        print(
            f"[{stick.get_instance_id()}] {stick.get_name()}: "
            f"{stick.get_numbuttons()} buttons, {stick.get_numaxes()} axes, "
            f"{stick.get_numhats()} hats"
        )
    print("press buttons and move sticks; Ctrl-C to stop")
    last: dict[tuple[int, str], str] = {}
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            pygame.event.pump()
            for stick in sticks:
                sid = stick.get_instance_id()
                down = [i for i in range(stick.get_numbuttons()) if stick.get_button(i)]
                axes = [round(stick.get_axis(i), 2) for i in range(stick.get_numaxes())]
                hats = [stick.get_hat(i) for i in range(stick.get_numhats())]
                line = f"buttons {down} axes {axes} hats {hats}"
                if last.get((sid, "line")) != line:
                    last[(sid, "line")] = line
                    print(f"[{sid}] {line}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 120.0)
