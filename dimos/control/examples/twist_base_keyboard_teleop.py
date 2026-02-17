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

"""Keyboard teleop for twist base via ControlCoordinator.

Runs a mock holonomic twist base with pygame keyboard control.
WASD keys publish Twist → coordinator's twist_command port → virtual joints
→ tick loop → MockTwistBaseAdapter.

Controls:
    W/S: Forward/backward (linear.x)
    Q/E: Strafe left/right (linear.y)
    A/D: Turn left/right (angular.z)
    Shift: 2x boost
    Ctrl: 0.5x slow
    Space: Emergency stop
    ESC: Quit

Usage:
    python -m dimos.control.examples.twist_base_keyboard_teleop
"""

from __future__ import annotations

from dimos.control.blueprints import coordinator_mock_twist_base
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Twist
from dimos.robot.unitree.keyboard_teleop import keyboard_teleop


def main() -> None:
    """Run mock twist base + keyboard teleop."""
    # Build coordinator with mock twist base
    coord = coordinator_mock_twist_base.build()

    # Build keyboard teleop with LCM transport on /cmd_vel
    teleop = (
        keyboard_teleop()
        .transports(
            {
                ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            }
        )
        .build()
    )

    print("Starting mock twist base coordinator + keyboard teleop...")
    print("Coordinator tick loop: 100Hz")
    print("Keyboard teleop: 50Hz on /cmd_vel")
    print()

    coord.start()
    teleop.start()

    # Block until coordinator stops (or Ctrl+C)
    try:
        coord.loop()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()
        coord.stop()
        print("Stopped.")


if __name__ == "__main__":
    main()
