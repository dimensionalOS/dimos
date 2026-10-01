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

"""Exercise keyboard command and release events without a display or robot."""

from dimos_generated.geometry_msgs.msg import Twist, TwistStamped
from dimos_generated.std_msgs.msg import Float32
import pygame

from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


def main() -> None:
    module = KeyboardTeleopModule()
    commands: list[TwistStamped] = []
    gripper: list[Float32] = []
    subscriptions = [
        module.ee_twist_command.subscribe(
            lambda msg: commands.append(TwistStamped.decode(msg.encode()))
        ),
        module.gripper_command.subscribe(lambda msg: gripper.append(Float32.decode(msg.encode()))),
    ]
    try:
        held = {pygame.K_w, pygame.K_a}
        module._handle_pygame_event(pygame.event.Event(pygame.KEYUP, key=pygame.K_w), held)
        assert commands[-1].twist.linear.y == 0.05
        print("Release W while A remains held → CDR velocity: left 0.05 m/s")
        module._handle_pygame_event(pygame.event.Event(pygame.KEYUP, key=pygame.K_a), held)
        assert commands[-1].twist == Twist()
        print("Release final motion key → CDR velocity: all six components zero")
        for key in [pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET]:
            module._handle_pygame_event(pygame.event.Event(pygame.KEYDOWN, key=key), held)
        assert [msg.data for msg in gripper] == [1, 0]
        print("Gripper [ then ] → generated CDR Float32: open 1.0, closed 0.0")
        print("Synthetic pygame events; no window, keyboard capture, or hardware motion.")
    finally:
        for unsubscribe in subscriptions:
            unsubscribe()
        module.stop()


if __name__ == "__main__":
    main()
