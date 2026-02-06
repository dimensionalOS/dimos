#!/usr/bin/env python3
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

"""Quest controller types with nice API for parsing Joy messages."""

from dataclasses import dataclass, field

from dimos.msgs.sensor_msgs import Joy
from dimos.msgs.std_msgs import UInt32


@dataclass
class ThumbstickState:
    """State of a thumbstick with X/Y axes."""

    x: float = 0.0
    y: float = 0.0


@dataclass
class QuestController:
    """Parsed Quest controller state from Joy message.

    Quest controller button indices:
        0: trigger, 1: grip, 2: touchpad, 3: thumbstick,
        4: X/A, 5: Y/B, 6: menu

    Quest controller axes:
        0: thumbstick X, 1: thumbstick Y
    """

    is_left: bool = True

    # Analog values (0.0-1.0)
    trigger: float = 0.0
    grip: float = 0.0

    # Digital buttons
    touchpad: bool = False
    thumbstick_press: bool = False
    primary: bool = False  # X on left, A on right
    secondary: bool = False  # Y on left, B on right
    menu: bool = False

    # Thumbstick axes
    thumbstick: ThumbstickState = field(default_factory=ThumbstickState)

    @classmethod
    def from_joy(cls, joy: Joy, is_left: bool = True) -> "QuestController":
        """Create QuestController from Joy message."""
        controller = cls(is_left=is_left)

        buttons = joy.buttons or []
        axes = joy.axes or []

        if len(buttons) >= 7:
            controller.trigger = float(buttons[0])
            controller.grip = float(buttons[1])
            controller.touchpad = bool(buttons[2])
            controller.thumbstick_press = bool(buttons[3])
            controller.primary = bool(buttons[4])
            controller.secondary = bool(buttons[5])
            controller.menu = bool(buttons[6])

        if len(axes) >= 2:
            controller.thumbstick = ThumbstickState(
                x=float(axes[0]),
                y=float(axes[1]),
            )

        return controller

    # Convenience aliases
    @property
    def x(self) -> bool:
        return self.primary
    @property
    def y(self) -> bool:
        return self.secondary
    @property
    def a(self) -> bool:
        return self.primary
    @property
    def b(self) -> bool:
        return self.secondary



class QuestButtons(UInt32):
    """Packed button states for both Quest controllers in a single UInt32.

    Bit layout:
        Left (bits 0-6): trigger, grip, touchpad, thumbstick, X, Y, menu
        Right (bits 8-14): trigger, grip, touchpad, thumbstick, A, B, menu
    """

    # Bit positions
    BITS = {
        "left_trigger": 0,
        "left_grip": 1,
        "left_touchpad": 2,
        "left_thumbstick": 3,
        "left_x": 4,
        "left_y": 5,
        "left_menu": 6,
        "right_trigger": 8,
        "right_grip": 9,
        "right_touchpad": 10,
        "right_thumbstick": 11,
        "right_a": 12,
        "right_b": 13,
        "right_menu": 14,
    }

    def __getattr__(self, name: str) -> bool:
        if name in QuestButtons.BITS:
            return bool(self.data & (1 << QuestButtons.BITS[name]))
        raise AttributeError(f"'{type(self).__name__}' has no attribute '{name}'")

    def __setattr__(self, name: str, value: bool) -> None:
        if name in QuestButtons.BITS:
            if value:
                self.data |= 1 << QuestButtons.BITS[name]
            else:
                self.data &= ~(1 << QuestButtons.BITS[name])
        else:
            super().__setattr__(name, value)

    @classmethod
    def from_controllers(
        cls,
        left: "QuestController | None",
        right: "QuestController | None",
    ) -> "QuestButtons":
        """Create QuestButtons from two QuestController instances."""
        buttons = cls()

        if left:
            buttons.left_trigger = left.trigger > 0.5
            buttons.left_grip = left.grip > 0.5
            buttons.left_touchpad = left.touchpad
            buttons.left_thumbstick = left.thumbstick_press
            buttons.left_x = left.primary
            buttons.left_y = left.secondary
            buttons.left_menu = left.menu

        if right:
            buttons.right_trigger = right.trigger > 0.5
            buttons.right_grip = right.grip > 0.5
            buttons.right_touchpad = right.touchpad
            buttons.right_thumbstick = right.thumbstick_press
            buttons.right_a = right.primary
            buttons.right_b = right.secondary
            buttons.right_menu = right.menu

        return buttons


__all__ = ["QuestController", "QuestButtons", "ThumbstickState"]
