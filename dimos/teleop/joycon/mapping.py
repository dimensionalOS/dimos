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

"""Joy-Con readings to the teleop streams dimos already speaks."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Literal

from dimos.teleop.webxr.controller_types import Buttons

JoyConSide = Literal["left", "right"]
JOYCON_SIDES: tuple[JoyConSide, ...] = ("left", "right")
GripperMode = Literal["hold", "toggle"]

# Joy-Con button labels, per side, as printed on the controller.
LEFT_JOYCON_BUTTONS = (
    "up",
    "down",
    "left",
    "right",
    "L",
    "ZL",
    "SL",
    "SR",
    "minus",
    "capture",
    "stick",
)
RIGHT_JOYCON_BUTTONS = ("A", "B", "X", "Y", "R", "ZR", "SL", "SR", "plus", "home", "stick")

# Joy-Con label to the dimos Buttons bit it lands on. The right Joy-Con's X and
# Y go to the left-hand bits so the X and Y aliases the episode monitor uses
# keep their meaning; ZL and ZR are the triggers the grippers follow.
BUTTONS_BIT_BY_LABEL: dict[JoyConSide, dict[str, str]] = {
    "left": {
        "ZL": "left_trigger",
        "L": "left_grip",
        "minus": "left_menu",
        "stick": "left_thumbstick",
    },
    "right": {
        "ZR": "right_trigger",
        "R": "right_grip",
        "A": "right_primary",
        "B": "right_secondary",
        "X": "left_primary",
        "Y": "left_secondary",
        "plus": "right_menu",
        "stick": "right_thumbstick",
    },
}
GRIPPER_LABEL: dict[JoyConSide, str] = {"left": "ZL", "right": "ZR"}

# Input event key codes the kernel hid-nintendo driver reports per button (BTN_TL2
# for ZL, BTN_SOUTH for B, ...), read straight from the driver's event device.
# Confirm with `python -m dimos.teleop.joycon.probe` and override per side in the
# module config when a driver version differs.
DEFAULT_BUTTON_INDEX: dict[JoyConSide, dict[str, int]] = {
    "left": {
        "capture": 309,
        "L": 310,
        "SL": 311,
        "ZL": 312,
        "SR": 313,
        "minus": 314,
        "stick": 317,
        "up": 544,
        "down": 545,
        "left": 546,
        "right": 547,
    },
    "right": {
        "B": 304,
        "A": 305,
        "X": 307,
        "Y": 308,
        "SL": 310,
        "R": 311,
        "SR": 312,
        "ZR": 313,
        "plus": 315,
        "home": 316,
        "stick": 318,
    },
}
# Absolute axis codes of each Joy-Con's stick: ABS_X/ABS_Y on the left, ABS_RX/ABS_RY
# on the right, each -32767..32767.
DEFAULT_AXIS_INDEX: dict[JoyConSide, dict[str, int]] = {
    "left": {"x": 0, "y": 1},
    "right": {"x": 3, "y": 4},
}
STICK_FULL_SCALE = 32767.0


@dataclass(frozen=True)
class JoyConReading:
    """One Joy-Con sampled once: pressed labels, stick in -1..1, stick pressed."""

    pressed: frozenset[str] = frozenset()
    stick: tuple[float, float] = (0.0, 0.0)

    @property
    def stick_pressed(self) -> bool:
        return "stick" in self.pressed


@dataclass
class GripperState:
    """Per-side gripper state the module carries between ticks."""

    opening: float = 1.0
    closed: bool = False
    trigger_was_down: bool = False


@dataclass
class JoyConState:
    """Everything the module carries between ticks."""

    grippers: dict[JoyConSide, GripperState] = field(
        default_factory=lambda: {side: GripperState() for side in JOYCON_SIDES}
    )


def buttons_message(left: JoyConReading | None, right: JoyConReading | None) -> Buttons:
    """Pack both Joy-Cons into the WebXR Buttons bitfield, triggers as 0 or 1."""
    buttons = Buttons()
    readings: dict[JoyConSide, JoyConReading | None] = {"left": left, "right": right}
    for side in JOYCON_SIDES:
        reading = readings[side]
        if reading is None:
            continue
        for label, bit in BUTTONS_BIT_BY_LABEL[side].items():
            if label in reading.pressed:
                buttons.set_attribute(bit, True)
    buttons.pack_analog_triggers(
        1.0 if left is not None and "ZL" in left.pressed else 0.0,
        1.0 if right is not None and "ZR" in right.pressed else 0.0,
    )
    return buttons


def step_gripper(
    state: GripperState,
    trigger_down: bool,
    *,
    mode: GripperMode,
    slew_per_s: float,
    dt: float,
) -> float:
    """Advance one side's opening toward closed or open and return it (1 = open).

    hold: the jaw is closed while the trigger is held. toggle: a press flips it.
    The opening moves at most slew_per_s * dt per tick so the jaw never slams.
    """
    if slew_per_s <= 0.0:
        raise ValueError("slew_per_s must be positive")
    if mode == "hold":
        state.closed = trigger_down
    elif trigger_down and not state.trigger_was_down:
        state.closed = not state.closed
    state.trigger_was_down = trigger_down
    target = 0.0 if state.closed else 1.0
    step = slew_per_s * max(0.0, dt)
    state.opening += max(-step, min(step, target - state.opening))
    return state.opening


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zero inside the deadzone, then rescaled so full deflection stays 1."""
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be in [0, 1)")
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return min(1.0, scaled) if value > 0 else -min(1.0, scaled)


def stick_to_base_velocity(
    reading: JoyConReading | None,
    *,
    deadzone: float,
    max_linear_m_s: float,
    max_angular_rad_s: float,
) -> tuple[float, float, float]:
    """Left stick to (vx, vy, wz): forward and left, or yaw while the stick is pressed.

    The driver gives +x to the right and +y down, so forward is -y and left is -x.
    """
    if reading is None:
        return (0.0, 0.0, 0.0)
    x = apply_deadzone(reading.stick[0], deadzone)
    y = apply_deadzone(reading.stick[1], deadzone)
    if reading.stick_pressed:
        return (0.0, 0.0, -x * max_angular_rad_s)
    return (-y * max_linear_m_s, -x * max_linear_m_s, 0.0)


def pressed_labels(
    button_down: Mapping[int, bool], index_by_label: Mapping[str, int]
) -> frozenset[str]:
    """Translate the key codes that are down into Joy-Con labels."""
    return frozenset(
        label for label, index in index_by_label.items() if button_down.get(index, False)
    )
