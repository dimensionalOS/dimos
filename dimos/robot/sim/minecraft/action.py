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

import numpy as np
from numpy.typing import NDArray


class Action:
    """Minecraft action with convenient properties for setting movement and camera controls.

    The action space has 8 dimensions:
    - [0] forward/back: 0=noop, 1=forward, 2=back
    - [1] left/right: 0=noop, 1=left, 2=right
    - [2] jump/sneak/sprint: 0=noop, 1=jump, 2=sneak, 3=sprint
    - [3] camera_pitch: 0-24 (maps to -180 to 180 degrees)
    - [4] camera_yaw: 0-24 (maps to -180 to 180 degrees)
    - [5] functional: 0=noop, 1=use, 2=drop, 3=attack, 4=craft, 5=equip, 6=place, 7=destroy
    - [6] craft_arg: 0-243 (which item to craft)
    - [7] slot_arg: 0-35 (which inventory slot for equip/place/destroy)
    """

    def __init__(self, action_array: NDArray[np.int32] = None):
        """Initialize action from existing array or create a no-op action.

        Args:
            action_array: Optional 8-element action array. If None, creates no-op.
        """
        if action_array is None:
            self._action = np.zeros(8, dtype=np.int32)
            # Set camera to neutral position (no movement)
            self._action[3] = 12  # pitch
            self._action[4] = 12  # yaw
        else:
            self._action = np.array(action_array, dtype=np.int32)
            assert self._action.shape == (8,), (
                f"Action must have 8 elements, got {self._action.shape}"
            )

    @classmethod
    def noop(cls) -> "Action":
        """Create a no-op action (all zeros)."""
        return cls()

    @property
    def array(self) -> NDArray[np.int32]:
        """Get the raw action array."""
        return self._action

    # Movement properties
    @property
    def forward(self) -> bool:
        """Check if moving forward."""
        return self._action[0] == 1

    @forward.setter
    def forward(self, value: bool):
        """Set forward movement."""
        if value:
            self._action[0] = 1
        else:
            self._action[0] = 0

    @property
    def backward(self) -> bool:
        """Check if moving backward."""
        return self._action[0] == 2

    @backward.setter
    def backward(self, value: bool):
        """Set backward movement."""
        if value:
            self._action[0] = 2
        else:
            self._action[0] = 0

    @property
    def left(self) -> bool:
        """Check if moving left."""
        return self._action[1] == 1

    @left.setter
    def left(self, value: bool):
        """Set left movement."""
        if value:
            self._action[1] = 1
        else:
            self._action[1] = 0

    @property
    def right(self) -> bool:
        """Check if moving right."""
        return self._action[1] == 2

    @right.setter
    def right(self, value: bool):
        """Set right movement."""
        if value:
            self._action[1] = 2
        else:
            self._action[1] = 0

    @property
    def jump(self) -> bool:
        """Check if jumping."""
        return self._action[2] == 1

    @jump.setter
    def jump(self, value: bool):
        """Set jump action."""
        if value:
            self._action[2] = 1

    @property
    def sneak(self) -> bool:
        """Check if sneaking."""
        return self._action[2] == 2

    @sneak.setter
    def sneak(self, value: bool):
        """Set sneak action."""
        if value:
            self._action[2] = 2

    @property
    def sprint(self) -> bool:
        """Check if sprinting."""
        return self._action[2] == 3

    @sprint.setter
    def sprint(self, value: bool):
        """Set sprint action."""
        if value:
            self._action[2] = 3

    # Camera properties
    @property
    def camera_pitch(self) -> float:
        """Get camera pitch delta in degrees (-180 to 180)."""
        return (self._action[3] - 12) * 15.0  # Map 0-24 to -180 to 180

    @camera_pitch.setter
    def camera_pitch(self, degrees: float):
        """Set camera pitch delta in degrees (-180 to 180)."""
        # Clamp to valid range and convert to action space
        degrees = np.clip(degrees, -180, 180)
        self._action[3] = int(np.round((degrees / 15.0) + 12))
        self._action[3] = np.clip(self._action[3], 0, 24)

    @property
    def camera_yaw(self) -> float:
        """Get camera yaw delta in degrees (-180 to 180)."""
        return (self._action[4] - 12) * 15.0  # Map 0-24 to -180 to 180

    @camera_yaw.setter
    def camera_yaw(self, degrees: float):
        """Set camera yaw delta in degrees (-180 to 180)."""
        # Clamp to valid range and convert to action space
        degrees = np.clip(degrees, -180, 180)
        self._action[4] = int(np.round((degrees / 15.0) + 12))
        self._action[4] = np.clip(self._action[4], 0, 24)

    # Functional actions
    @property
    def use(self) -> bool:
        """Check if using (right click)."""
        return self._action[5] == 1

    @use.setter
    def use(self, value: bool):
        """Set use action."""
        if value:
            self._action[5] = 1

    @property
    def attack(self) -> bool:
        """Check if attacking (left click)."""
        return self._action[5] == 3

    @attack.setter
    def attack(self, value: bool):
        """Set attack action."""
        if value:
            self._action[5] = 3

    @property
    def drop(self) -> bool:
        """Check if dropping item."""
        return self._action[5] == 2

    @drop.setter
    def drop(self, value: bool):
        """Set drop action."""
        if value:
            self._action[5] = 2

    def craft(self, item_id: int):
        """Set craft action with item ID (0-243)."""
        self._action[5] = 4
        self._action[6] = np.clip(item_id, 0, 243)

    def equip(self, slot: int):
        """Set equip action with inventory slot (0-35)."""
        self._action[5] = 5
        self._action[7] = np.clip(slot, 0, 35)

    def place(self, slot: int):
        """Set place action with inventory slot (0-35)."""
        self._action[5] = 6
        self._action[7] = np.clip(slot, 0, 35)

    def destroy(self, slot: int):
        """Set destroy action with inventory slot (0-35)."""
        self._action[5] = 7
        self._action[7] = np.clip(slot, 0, 35)

    # Vector-like movement setters
    @property
    def x(self) -> int:
        """Get forward/back movement as x-axis: -1=back, 0=noop, 1=forward."""
        if self._action[0] == 1:
            return 1  # forward
        elif self._action[0] == 2:
            return -1  # back
        return 0

    @x.setter
    def x(self, value: int):
        """Set forward/back movement as x-axis: -1=back, 0=noop, 1=forward."""
        if value > 0:
            self._action[0] = 1  # forward
        elif value < 0:
            self._action[0] = 2  # back
        else:
            self._action[0] = 0  # noop

    @property
    def y(self) -> int:
        """Get left/right movement as y-axis: -1=left, 0=noop, 1=right."""
        if self._action[1] == 1:
            return -1  # left (negative y)
        elif self._action[1] == 2:
            return 1  # right (positive y)
        return 0

    @y.setter
    def y(self, value: int):
        """Set left/right movement as y-axis: -1=left, 0=noop, 1=right."""
        if value < 0:
            self._action[1] = 1  # left
        elif value > 0:
            self._action[1] = 2  # right
        else:
            self._action[1] = 0  # noop

    @property
    def z(self) -> int:
        """Get vertical movement: 1=jump, -1=sneak, 0=noop."""
        if self._action[2] == 1:
            return 1  # jump (up)
        elif self._action[2] == 2:
            return -1  # sneak (down/crouch)
        return 0

    @z.setter
    def z(self, value: int):
        """Set vertical movement: 1=jump, -1=sneak, 0=noop."""
        if value > 0:
            self._action[2] = 1  # jump
        elif value < 0:
            self._action[2] = 2  # sneak
        else:
            self._action[2] = 0  # noop

    def __str__(self) -> str:
        """Create readable string representation."""
        parts = []

        # Movement
        if self.forward:
            parts.append("forward")
        elif self.backward:
            parts.append("back")

        if self.left:
            parts.append("left")
        elif self.right:
            parts.append("right")

        if self.jump:
            parts.append("jump")
        elif self.sneak:
            parts.append("sneak")
        elif self.sprint:
            parts.append("sprint")

        # Camera
        if abs(self.camera_pitch) > 0.1:
            parts.append(f"pitch={self.camera_pitch:.1f}°")
        if abs(self.camera_yaw) > 0.1:
            parts.append(f"yaw={self.camera_yaw:.1f}°")

        # Functional
        if self.use:
            parts.append("use")
        elif self.attack:
            parts.append("attack")
        elif self.drop:
            parts.append("drop")
        elif self._action[5] == 4:
            parts.append(f"craft[{self._action[6]}]")
        elif self._action[5] == 5:
            parts.append(f"equip[{self._action[7]}]")
        elif self._action[5] == 6:
            parts.append(f"place[{self._action[7]}]")
        elif self._action[5] == 7:
            parts.append(f"destroy[{self._action[7]}]")

        return f"Action({', '.join(parts) if parts else 'noop'})"
