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

from typing import Tuple, TypedDict

import numpy as np
from numpy.typing import NDArray


class DamageSource(TypedDict):
    damage_amount: NDArray[np.float32]
    damage_distance: NDArray[np.float32]
    damage_pitch: NDArray[np.float32]
    damage_yaw: NDArray[np.float32]
    hunger_damage: NDArray[np.float32]
    is_explosion: np.bool_
    is_fire_damage: np.bool_
    is_magic_damage: np.bool_
    is_projectile: np.bool_
    is_unblockable: np.bool_


class DeltaInv(TypedDict):
    dec_name_by_craft: NDArray[np.str_]
    dec_name_by_other: NDArray[np.str_]
    dec_quantity_by_craft: NDArray[np.float32]
    dec_quantity_by_other: NDArray[np.float32]
    inc_name_by_craft: NDArray[np.str_]
    inc_name_by_other: NDArray[np.str_]
    inc_quantity_by_craft: NDArray[np.float32]
    inc_quantity_by_other: NDArray[np.float32]


class Equipment(TypedDict):
    cur_durability: NDArray[np.float32]
    max_durability: NDArray[np.float32]
    name: NDArray[np.str_]
    quantity: NDArray[np.float32]
    variant: NDArray[np.int32]


class Inventory(TypedDict):
    cur_durability: NDArray[np.float32]
    max_durability: NDArray[np.float32]
    name: NDArray[np.str_]
    quantity: NDArray[np.float32]
    variant: NDArray[np.int32]


class LifeStats(TypedDict):
    armor: NDArray[np.float32]
    food: NDArray[np.float32]
    is_sleeping: np.bool_
    life: NDArray[np.float32]
    oxygen: NDArray[np.float32]
    saturation: NDArray[np.float32]
    xp: NDArray[np.float32]


class LocationStats(TypedDict):
    biome_id: np.int32
    can_see_sky: np.bool_
    is_raining: np.bool_
    light_level: NDArray[np.float32]
    pitch: NDArray[np.float32]
    pos: NDArray[np.float32]  # Shape: (3,)
    rainfall: NDArray[np.float32]
    sea_level: NDArray[np.float32]
    sky_light_level: NDArray[np.float32]
    sun_brightness: NDArray[np.float32]
    temperature: NDArray[np.float32]
    yaw: NDArray[np.float32]


class Masks(TypedDict):
    action_arg: NDArray[np.bool_]  # Shape: (8, 1)
    action_type: NDArray[np.bool_]  # Shape: (8,)
    craft_smelt: NDArray[np.bool_]  # Shape: (244,)
    destroy: NDArray[np.bool_]  # Shape: (36,)
    equip: NDArray[np.bool_]  # Shape: (36,)
    place: NDArray[np.bool_]  # Shape: (36,)


class NearbyTools(TypedDict):
    furnace: np.bool_
    table: np.bool_


class Voxels(TypedDict):
    block_meta: NDArray[np.int32]  # Shape: (X, 5, Z) - typically 21x5x21
    block_name: NDArray[np.str_]  # Shape: (X, 5, Z)
    blocks_light: NDArray[np.bool_]  # Shape: (X, 5, Z)
    blocks_movement: NDArray[np.bool_]  # Shape: (X, 5, Z)
    can_burn: NDArray[np.bool_]  # Shape: (X, 5, Z)
    cos_look_vec_angle: NDArray[np.float32]  # Shape: (X, 5, Z)
    is_collidable: NDArray[np.bool_]  # Shape: (X, 5, Z)
    is_liquid: NDArray[np.bool_]  # Shape: (X, 5, Z)
    is_solid: NDArray[np.bool_]  # Shape: (X, 5, Z)
    is_tool_not_required: NDArray[np.bool_]  # Shape: (X, 5, Z)


class ObservationDict(TypedDict):
    damage_source: DamageSource
    delta_inv: DeltaInv
    equipment: Equipment
    inventory: Inventory
    life_stats: LifeStats
    location_stats: LocationStats
    masks: Masks
    nearby_tools: NearbyTools
    rgb: NDArray[np.uint8]  # Shape: (3, H, W) - CHW format
    voxels: Voxels


class Output:
    """Complete Minecraft output containing all state information."""

    def __init__(self, obs_tuple: Tuple):
        """Initialize from the output tuple returned by Minecraft environment.

        Args:
            obs_tuple: Tuple containing (obs_dict, reward, done, truncated, info)
        """
        self.obs_dict: ObservationDict = obs_tuple[0]
        self.reward: float = obs_tuple[1]
        self.done: bool = obs_tuple[2]
        self.truncated: bool = obs_tuple[3]
        self.info: dict = obs_tuple[4]

    @property
    def damage_source(self) -> DamageSource:
        return self.obs_dict["damage_source"]

    @property
    def delta_inv(self) -> DeltaInv:
        return self.obs_dict["delta_inv"]

    @property
    def equipment(self) -> Equipment:
        return self.obs_dict["equipment"]

    @property
    def inventory(self) -> Inventory:
        return self.obs_dict["inventory"]

    @property
    def life_stats(self) -> LifeStats:
        return self.obs_dict["life_stats"]

    @property
    def location_stats(self) -> LocationStats:
        return self.obs_dict["location_stats"]

    @property
    def masks(self) -> Masks:
        return self.obs_dict["masks"]

    @property
    def nearby_tools(self) -> NearbyTools:
        return self.obs_dict["nearby_tools"]

    @property
    def rgb(self) -> NDArray[np.uint8]:
        return self.obs_dict["rgb"]

    @property
    def voxels(self) -> Voxels:
        return self.obs_dict["voxels"]

    @property
    def position(self) -> NDArray[np.float32]:
        """Get agent position as [x, y, z] array."""
        return self.location_stats["pos"]

    @property
    def health(self) -> float:
        """Get agent health value."""
        return float(self.life_stats["life"][0])

    @property
    def hunger(self) -> float:
        """Get agent hunger/food value."""
        return float(self.life_stats["food"][0])

    @property
    def yaw(self) -> float:
        """Get agent yaw rotation in degrees."""
        return float(self.location_stats["yaw"][0])

    @property
    def pitch(self) -> float:
        """Get agent pitch rotation in degrees."""
        return float(self.location_stats["pitch"][0])

    def __str__(self) -> str:
        """Create a readable string representation of the Minecraft output."""
        lines = [
            f"  Position: [{self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f}]",
            f"  Rotation: yaw={self.yaw:.1f}°, pitch={self.pitch:.1f}°",
            f"  Health: {self.health:.1f}/20",
            f"  Hunger: {self.hunger:.1f}/20",
            f"  Biome: {self.info.get('biome_name', 'Unknown')} (id={self.location_stats['biome_id']})",
            f"  RGB Image: {self.rgb.shape[1]}x{self.rgb.shape[2]} ({self.rgb.shape[0]} channels)",
            f"  Voxel Grid: {self.voxels['block_name'].shape}",
            f"  Reward: {self.reward}",
            f"  Done: {self.done}, Truncated: {self.truncated}",
        ]

        # Add inventory summary if player has items
        equipped_items = []
        for i, name in enumerate(self.equipment["name"]):
            if name != "air":
                equipped_items.append(f"{name} x{int(self.equipment['quantity'][i])}")
        if equipped_items:
            lines.append(f"  Equipment: {', '.join(equipped_items)}")

        inventory_items = []
        for i, name in enumerate(self.inventory["name"]):
            if name != "air":
                inventory_items.append(f"{name} x{int(self.inventory['quantity'][i])}")
        if inventory_items:
            lines.append(
                f"  Inventory: {', '.join(inventory_items[:5])}"
                + (" ..." if len(inventory_items) > 5 else "")
            )

        # Add nearby tools
        tools = []
        if self.nearby_tools["table"]:
            tools.append("crafting table")
        if self.nearby_tools["furnace"]:
            tools.append("furnace")
        if tools:
            lines.append(f"  Nearby Tools: {', '.join(tools)}")

        return "\n".join(lines)


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
