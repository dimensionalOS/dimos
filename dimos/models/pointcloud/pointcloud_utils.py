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

import random


# Distance calculations
def human_like_distance(distance_meters) -> str:
    # Define the choices with units included, focusing on the 0.1 to 10 meters range
    if distance_meters < 1:  # For distances less than 1 meter
        choices = [
            (
                round(distance_meters * 100, 2),
                "centimeters",
                0.2,
            ),  # Centimeters for very small distances
            (
                round(distance_meters * 39.3701, 2),
                "inches",
                0.8,
            ),  # Inches for the majority of cases under 1 meter
        ]
    elif distance_meters < 3:  # For distances less than 3 meters
        choices = [
            (round(distance_meters, 2), "meters", 0.5),
            (
                round(distance_meters * 3.28084, 2),
                "feet",
                0.5,
            ),  # Feet as a common unit within indoor spaces
        ]
    else:  # For distances from 3 up to 10 meters
        choices = [
            (
                round(distance_meters, 2),
                "meters",
                0.7,
            ),  # Meters for clarity and international understanding
            (
                round(distance_meters * 3.28084, 2),
                "feet",
                0.3,
            ),  # Feet for additional context
        ]

    # Normalize probabilities and make a selection
    total_probability = sum(prob for _, _, prob in choices)
    cumulative_distribution = []
    cumulative_sum = 0
    for value, unit, probability in choices:
        cumulative_sum += probability / total_probability  # Normalize probabilities
        cumulative_distribution.append((cumulative_sum, value, unit))

    # Randomly choose based on the cumulative distribution
    r = random.random()
    for cumulative_prob, value, unit in cumulative_distribution:
        if r < cumulative_prob:
            return f"{value} {unit}"

    # Fallback to the last choice if something goes wrong
    return f"{choices[-1][0]} {choices[-1][1]}"




def get_bounding_box_height(pcd):
    """
    Compute the height of the bounding box for a given point cloud.

    Parameters:
    pcd (open3d.geometry.PointCloud): The input point cloud.

    Returns:
    float: The height of the bounding box.
    """
    aabb = pcd.get_axis_aligned_bounding_box()
    return aabb.get_extent()[1]  # Assuming the Y-axis is the up-direction


