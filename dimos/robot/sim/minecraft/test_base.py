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

import pickle
from pathlib import Path
from typing import Any, Tuple

import pytest

from dimos.robot.sim import minecraft
from dimos.utils.data import get_data


@pytest.fixture
def minecraft_output(
    name: str = "output.pkl",
) -> Tuple[minecraft.ObservationDict, float, bool, bool, dict]:
    with open(get_data("minecraft") / name, "rb") as f:
        obs_tuple = pickle.load(f)
    return obs_tuple


def test_minecraft_output_structure(minecraft_output):
    """Test that MinecraftOutput can be created and has valid structure."""
    obs = minecraft.Output(minecraft_output)

    # Basic type checks
    assert isinstance(obs.obs_dict, dict), "First element should be observation dict"
    assert isinstance(obs.reward, (int, float)), f"Reward should be numeric, got {type(obs.reward)}"
    assert isinstance(obs.done, bool), f"Done should be bool, got {type(obs.done)}"
    assert isinstance(obs.truncated, bool), f"Truncated should be bool, got {type(obs.truncated)}"
    assert isinstance(obs.info, dict), f"Info should be dict, got {type(obs.info)}"

    # Check required keys in observation dict
    required_keys = {
        "damage_source",
        "delta_inv",
        "equipment",
        "inventory",
        "life_stats",
        "location_stats",
        "masks",
        "nearby_tools",
        "rgb",
        "voxels",
    }
    obs_keys = set(obs.obs_dict.keys())
    missing_keys = required_keys - obs_keys
    assert not missing_keys, f"Missing required keys: {missing_keys}"


def test_minecraft_output_arrays(minecraft_output):
    """Test array shapes and properties."""
    obs = minecraft.Output(minecraft_output)

    # RGB array
    assert obs.rgb.ndim == 3, f"RGB should be 3D array, got shape {obs.rgb.shape}"
    # RGB can be either CHW (3, H, W) or HWC (H, W, 3) format
    assert obs.rgb.shape[0] == 3 or obs.rgb.shape[2] == 3, (
        f"RGB should have 3 channels, got shape {obs.rgb.shape}"
    )

    # Position
    assert obs.position.shape == (3,), (
        f"Position should be 3D vector, got shape {obs.position.shape}"
    )

    # Voxel grid
    voxel_shape = obs.voxels["block_name"].shape
    assert len(voxel_shape) == 3, f"Voxel grid should be 3D, got shape {voxel_shape}"
    assert voxel_shape[1] == 5, f"Voxel grid should have 5 vertical layers, got {voxel_shape}"


def test_minecraft_output_properties(minecraft_output):
    """Test convenience properties."""
    obs = minecraft.Output(minecraft_output)

    # Test all properties return expected types
    assert isinstance(obs.health, float)
    assert isinstance(obs.hunger, float)
    assert isinstance(obs.yaw, float)
    assert isinstance(obs.pitch, float)

    # Test value ranges
    assert 0 <= obs.health <= 20, f"Health should be 0-20, got {obs.health}"
    assert 0 <= obs.hunger <= 20, f"Hunger should be 0-20, got {obs.hunger}"
    assert -180 <= obs.yaw <= 180, f"Yaw should be -180 to 180, got {obs.yaw}"
    assert -90 <= obs.pitch <= 90, f"Pitch should be -90 to 90, got {obs.pitch}"


def test_print_data(minecraft_output):
    print("\n", minecraft.Output(minecraft_output))
