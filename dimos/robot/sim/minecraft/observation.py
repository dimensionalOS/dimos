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
