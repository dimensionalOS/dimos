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

"""Example usage of the Path class."""

from dimos.msgs.nav_msgs import Path
from dimos.msgs.geometry_msgs import PoseStamped

# Create a path with some waypoints
path = Path(frame_id="map")

# Add poses using mutable push
for i in range(5):
    pose = PoseStamped(
        frame_id="map",  # Will be overridden by path's frame_id
        position=[i * 2.0, i * 1.0, 0.0],
        orientation=[0, 0, 0, 1],  # Identity quaternion
    )
    path.push_mut(pose)

print(f"Path has {len(path)} poses")
print(f"First pose: pos=({path.head().x}, {path.head().y}, {path.head().z})")
print(f"Last pose: pos=({path.last().x}, {path.last().y}, {path.last().z})")

# Create a new path with immutable operations
path2 = path.slice(1, 4)  # Get poses 1, 2, 3
path3 = path2.reverse()  # Reverse the order

print(f"\nSliced path has {len(path2)} poses")
print(f"Reversed path first pose: pos=({path3.head().x}, {path3.head().y}, {path3.head().z})")

# Iterate over poses
print("\nIterating over original path:")
for i, pose in enumerate(path):
    print(f"  Pose {i}: pos=({pose.x}, {pose.y}, {pose.z})")

# LCM encoding/decoding
encoded = path.lcm_encode()
decoded = Path.lcm_decode(encoded)
print(f"\nEncoded and decoded path has {len(decoded)} poses")
print(f"All poses have frame_id: '{decoded.frame_id}'")
