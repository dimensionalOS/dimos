#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

# Copyright 2025 Dimensional Inc.

"""Internal B1 command structure for UDP communication."""

from pydantic import BaseModel, Field
from typing import Optional
import struct


class B1Command(BaseModel):
    """Internal B1 robot command matching UDP packet structure.

    This is an internal type - external interfaces use standard Twist messages.
    """

    # Direct joystick values matching C++ NetworkJoystickCmd struct
    lx: float = Field(default=0.0, ge=-1.0, le=1.0)  # Turn velocity (left stick X)
    ly: float = Field(default=0.0, ge=-1.0, le=1.0)  # Forward/back velocity (left stick Y)
    rx: float = Field(default=0.0, ge=-1.0, le=1.0)  # Strafe velocity (right stick X)
    ry: float = Field(default=0.0, ge=-1.0, le=1.0)  # Pitch/height adjustment (right stick Y)
    buttons: int = Field(default=0, ge=0, le=65535)  # Button states (uint16)
    mode: int = Field(
        default=0, ge=0, le=255
    )  # Control mode (uint8): 0=idle, 1=stand, 2=walk, 6=recovery

    @classmethod
    def from_twist(cls, twist, mode: int = 2):
        """Create B1Command from standard ROS Twist message.

        This is the key integration point for navigation and planning.

        Args:
            twist: ROS Twist message with linear and angular velocities
            mode: Robot mode (default is walk mode for navigation)

        Returns:
            B1Command configured for the given Twist
        """
        if mode == 2:  # WALK mode - velocity control
            return cls(
                lx=-twist.angular.z,  # ROS yaw → B1 turn (negated for correct direction)
                ly=twist.linear.x,  # ROS forward → B1 forward
                rx=-twist.linear.y,  # ROS lateral → B1 strafe (negated for correct direction)
                ry=0.0,  # No pitch control in walk mode
                mode=mode,
            )
        elif mode == 1:  # STAND mode - body pose control
            # Map Twist pose controls to B1 joystick axes
            # G1 cpp server maps: ly→bodyHeight, lx→euler[2], rx→euler[0], ry→euler[1]
            return cls(
                lx=-twist.angular.z,  # ROS yaw → B1 yaw (euler[2])
                ly=twist.linear.z,  # ROS height → B1 bodyHeight
                rx=-twist.angular.x,  # ROS roll → B1 roll (euler[0])
                ry=twist.angular.y,  # ROS pitch → B1 pitch (euler[1])
                mode=mode,
            )
        else:
            # IDLE mode - no controls
            return cls(mode=mode)

    def to_bytes(self) -> bytes:
        """Pack to 19-byte UDP packet matching C++ struct.

        Format: 4 floats + uint16 + uint8 = 19 bytes (little-endian)
        """
        return struct.pack("<ffffHB", self.lx, self.ly, self.rx, self.ry, self.buttons, self.mode)

    def __str__(self) -> str:
        """Human-readable representation."""
        mode_names = {0: "IDLE", 1: "STAND", 2: "WALK", 6: "RECOVERY"}
        mode_str = mode_names.get(self.mode, f"MODE_{self.mode}")

        if self.lx != 0 or self.ly != 0 or self.rx != 0 or self.ry != 0:
            return f"B1Cmd[{mode_str}] LX:{self.lx:+.2f} LY:{self.ly:+.2f} RX:{self.rx:+.2f} RY:{self.ry:+.2f}"
        else:
            return f"B1Cmd[{mode_str}] (idle)"
