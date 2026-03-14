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
from typing import ClassVar

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.UInt32 import UInt32


@dataclass
class ThumbstickState:
    """State of a thumbstick with X/Y axes."""

    x: float = 0.0
    y: float = 0.0


@dataclass
class QuestControllerState:
    """Parsed Quest controller state from Joy message with no data loss.

    Preserves full-fidelity analog values (trigger, grip as floats, thumbstick axes)
    from the raw Joy message in a readable format. Use this when you need analog
    precision (e.g., proportional grip control). Subclasses can publish this
    alongside Buttons for float access.

    Axes layout:
        0: thumbstick X, 1: thumbstick Y, 2: trigger (analog), 3: grip (analog)
    Button indices (digital, 0 or 1):
        0: trigger, 1: grip, 2: touchpad, 3: thumbstick,
        4: X/A, 5: Y/B, 6: menu
    """

    EXPECTED_AXES: ClassVar[int] = 4
    EXPECTED_BUTTONS: ClassVar[int] = 7

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
    def from_joy(cls, joy: Joy, is_left: bool = True) -> "QuestControllerState":
        """Create QuestControllerState from Joy message.
        Expected axes: [thumbstick_x, thumbstick_y, trigger_analog, grip_analog]
        Expected buttons: [trigger, grip, touchpad, thumbstick, X/A, Y/B, menu]
        Raises:
            ValueError: If Joy message doesn't have expected Quest controller format.
        """
        buttons = joy.buttons or []
        axes = joy.axes or []

        if len(buttons) < cls.EXPECTED_BUTTONS:
            raise ValueError(f"Expected {cls.EXPECTED_BUTTONS} buttons, got {len(buttons)}")
        if len(axes) < cls.EXPECTED_AXES:
            raise ValueError(f"Expected {cls.EXPECTED_AXES} axes, got {len(axes)}")

        return cls(
            is_left=is_left,
            trigger=float(axes[2]),
            grip=float(axes[3]),
            touchpad=buttons[2] > 0.5,
            thumbstick_press=buttons[3] > 0.5,
            primary=buttons[4] > 0.5,
            secondary=buttons[5] > 0.5,
            menu=buttons[6] > 0.5,
            thumbstick=ThumbstickState(x=float(axes[0]), y=float(axes[1])),
        )


class Buttons(UInt32):
    """Packed button states for both controllers in a single UInt32.

    Digital buttons are collapsed to bools. Analog trigger values are packed
    as 7-bit integers in the upper 16 bits for proportional gripper control.

    Bit layout:
        Left  (bits 0-6):   trigger, grip, touchpad, thumbstick, primary, secondary, menu
        Right (bits 8-14):  trigger, grip, touchpad, thumbstick, primary, secondary, menu
        Bit 7, 15:          reserved
        Bits 16-22:         left trigger analog (7-bit, 0=0.0 … 127=1.0)
        Bits 23-29:         right trigger analog (7-bit, 0=0.0 … 127=1.0)
        Bits 30-31:         unused (kept clear so LCM signed int32 never overflows)
    """

    # Bit positions for digital buttons
    BITS = {
        "left_trigger": 0,
        "left_grip": 1,
        "left_touchpad": 2,
        "left_thumbstick": 3,
        "left_primary": 4,
        "left_secondary": 5,
        "left_menu": 6,
        "right_trigger": 8,
        "right_grip": 9,
        "right_touchpad": 10,
        "right_thumbstick": 11,
        "right_primary": 12,
        "right_secondary": 13,
        "right_menu": 14,
    }

    # Analog trigger packing constants
    _LEFT_TRIGGER_SHIFT: int = 16
    _RIGHT_TRIGGER_SHIFT: int = 23
    _ANALOG_MASK: int = 0x7F
    _ANALOG_MAX: int = 127

    @property
    def left_trigger_analog(self) -> float:
        """Left trigger analog value [0.0, 1.0], decoded from bits 16-22."""
        return ((self.data >> self._LEFT_TRIGGER_SHIFT) & self._ANALOG_MASK) / self._ANALOG_MAX

    @property
    def right_trigger_analog(self) -> float:
        """Right trigger analog value [0.0, 1.0], decoded from bits 23-29."""
        return ((self.data >> self._RIGHT_TRIGGER_SHIFT) & self._ANALOG_MASK) / self._ANALOG_MAX

    def pack_analog_triggers(self, left: float, right: float) -> None:
        """Pack analog trigger values [0.0, 1.0] into bits 16-22 and 23-29."""
        left_u7 = round(max(0.0, min(1.0, left)) * self._ANALOG_MAX)
        right_u7 = round(max(0.0, min(1.0, right)) * self._ANALOG_MAX)
        self.data = (
            (self.data & 0x0000FFFF)  # clear bits 16-31
            | (left_u7 << self._LEFT_TRIGGER_SHIFT)
            | (right_u7 << self._RIGHT_TRIGGER_SHIFT)
        )

    def __getattr__(self, name: str) -> bool:
        if name in Buttons.BITS:
            return bool(self.data & (1 << Buttons.BITS[name]))
        raise AttributeError(f"'{type(self).__name__}' has no attribute '{name}'")

    def __setattr__(self, name: str, value: bool) -> None:
        if name in Buttons.BITS:
            if value:
                self.data |= 1 << Buttons.BITS[name]
            else:
                self.data &= ~(1 << Buttons.BITS[name])
        else:
            super().__setattr__(name, value)

    @classmethod
    def from_controllers(
        cls,
        left: "QuestControllerState | None",
        right: "QuestControllerState | None",
    ) -> "Buttons":
        """Create Buttons from two QuestControllerState instances."""
        # Safe: cls() calls UInt32.__init__ which sets self.data = 0 before bit ops.
        buttons = cls()

        if left:
            buttons.left_trigger = left.trigger > 0.5
            buttons.left_grip = left.grip > 0.5
            buttons.left_touchpad = left.touchpad
            buttons.left_thumbstick = left.thumbstick_press
            buttons.left_primary = left.primary
            buttons.left_secondary = left.secondary
            buttons.left_menu = left.menu

        if right:
            buttons.right_trigger = right.trigger > 0.5
            buttons.right_grip = right.grip > 0.5
            buttons.right_touchpad = right.touchpad
            buttons.right_thumbstick = right.thumbstick_press
            buttons.right_primary = right.primary
            buttons.right_secondary = right.secondary
            buttons.right_menu = right.menu

        return buttons


__all__ = ["Buttons", "QuestControllerState", "ThumbstickState"]
