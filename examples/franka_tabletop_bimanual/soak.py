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

"""Right-arm reliability test: planned and straight moves for a few minutes, no gripper.

~/dimos-franka/.venv/bin/python soak.py [minutes]
"""

import math
import sys
import time

from handoff import READY, Arms, Failed

OUT = list(READY)
OUT[0] -= math.radians(20)  # turned away from the left arm
ABOVE = [
    math.radians(d) for d in (-14.3, -13.2, -15.2, -134.8, -4.9, 120.8, 49.4)
]  # over its own side


def main() -> None:
    minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    arms = Arms(dry=False)
    moves, t0 = 0, time.monotonic()
    try:
        arms.add_table()
        while time.monotonic() - t0 < minutes * 60:
            arms.move("right", OUT, "out")
            arms.move("right", READY, "back")
            arms.move("right", ABOVE, "over the table")
            arms.straight("right", -0.10, "down")
            arms.straight("right", 0.10, "up")
            arms.move("right", READY, "to ready")
            moves += 6
            print(
                f"--- {moves} moves, {(time.monotonic() - t0) / 60:.1f} min, no fault ---",
                flush=True,
            )
    except Failed as e:
        print(
            f"FAULT after {moves} moves, {(time.monotonic() - t0) / 60:.1f} min, at {time.strftime('%H:%M:%S')}: {e}"
        )
        print("right arm status:", arms.app.get_module("right_arm").status().last_error)
    finally:
        arms.close()


if __name__ == "__main__":
    main()
