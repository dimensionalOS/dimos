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
The same bindings as `KeyboardTeleop` (arrows + WASD, Alt+A/D or Alt+←/→ strafe)
publish Twist → coordinator's twist_command port → virtual joints
→ tick loop → MockTwistBaseAdapter.

Controls:
    W/S and Up/Down: Forward/backward (linear.x)
    A/D and Left/Right: Turn (angular.z)
    Alt+A/D or Alt+←/→: Strafe left/right (linear.y)
    Shift: 2x boost
    Ctrl: 0.5x slow
    Space: Emergency stop
    ESC: Quit

Usage:
    python -m dimos.control.examples.twist_base_keyboard_teleop
"""

from __future__ import annotations

from dimos.control.blueprints.mobile import coordinator_mock_twist_base
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop


def main() -> None:
    """Run mock twist base + keyboard teleop."""
    coord = ModuleCoordinator.build(coordinator_mock_twist_base)
    teleop = ModuleCoordinator.build(KeyboardTeleop.blueprint())

    print("Starting mock twist base coordinator + keyboard teleop...")
    print("Coordinator tick loop: 100Hz")
    print("Keyboard teleop: 50Hz on /cmd_vel")
    print()

    coord.start()
    teleop.start()

    # Block until Ctrl+C — loop() handles KeyboardInterrupt and calls stop()
    coord.loop()
    teleop.stop()


if __name__ == "__main__":
    main()
