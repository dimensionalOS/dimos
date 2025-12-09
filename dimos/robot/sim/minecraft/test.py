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
import time
from contextlib import contextmanager


@contextmanager
def create_env(**kwargs):
    import minedojo

    env = minedojo.make(**kwargs)
    try:
        yield env
    finally:
        env.close()


def capture_obs():
    # Use open-ended creative mode with custom voxel size
    with create_env(
        task_id="creative:1",  # Simple creative mode
        image_size=(800, 1280),
        world_seed="dimensional",
        use_voxel=True,
        voxel_size=dict(xmin=-5, ymin=-2, zmin=-5, xmax=5, ymax=2, zmax=5),
    ) as env:
        obs = env.reset()
        act = env.action_space.no_op()
        obs, reward, done, info = env.step(act)
        with open("observation.pkl", "wb") as f:
            pickle.dump(obs, f)
        print("Observation saved to observation.pkl")
        for i in range(50):
            print(i)
            obs, reward, done, info = env.step(act)
            time.sleep(0.05)
        return obs


def read_obs():
    with open("observation.pkl", "rb") as f:
        obs = pickle.load(f)

    # Validate we have data
    print("=== Observation Data Validation ===")
    print(f"Observation keys: {list(obs.keys())}")
    print(f"Number of keys: {len(obs.keys())}")

    # Check RGB image
    if "rgb" in obs:
        print(f"\nRGB shape: {obs['rgb'].shape}")
        print(f"RGB dtype: {obs['rgb'].dtype}")

    # Validate voxels
    print("\n=== Voxel Data Validation ===")
    if "voxels" in obs:
        print("✓ Voxels found in observation")
        voxel_data = obs["voxels"]
        print(f"Voxel keys: {list(voxel_data.keys())}")

        print(voxel_data["blocks_movement"])
        # Check block names
        if "block_name" in voxel_data:
            print(f"\nBlock name shape: {voxel_data['block_name'].shape}")
            print(f"Sample blocks: {voxel_data['block_name'].flatten()[:5]}")

        # Check voxel properties
        for key in ["is_collidable", "is_solid", "blocks_movement"]:
            if key in voxel_data:
                print(f"{key} shape: {voxel_data[key].shape}")
    else:
        print("✗ No voxels found in observation")

    return obs


def loop():
    with create_env(
        task_id="harvest_wool_with_shears_and_sheep",
        image_size=(160 * 5, 256 * 5),
    ) as env:
        for i in range(10_000):
            act = env.action_space.no_op()
            # act[0] = 1    # forward/backward
            print(i)
            if i % 100 == 0:
                act[2] = 1  # jump
            obs, reward, done, info = env.step(act)
            time.sleep(0.05)


if __name__ == "__main__":
    # Run capture_obs to save a new observation
    # capture_obs()

    # Then read and validate it
    read_obs()
