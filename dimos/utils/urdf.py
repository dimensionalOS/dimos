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

"""URDF generation utilities."""

from __future__ import annotations


def box_urdf(
    width: float,
    height: float,
    depth: float,
    name: str = "box_robot",
    mass: float = 1.0,
    rgba: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.5),
) -> str:
    """Generate a simple URDF with a box as the base_link.

    Args:
        width: Box size in X direction (meters)
        height: Box size in Y direction (meters)
        depth: Box size in Z direction (meters)
        name: Robot name
        mass: Mass of the box (kg)
        rgba: Color as (red, green, blue, alpha), default red with 0.5 transparency

    Returns:
        URDF XML string
    """
    # Simple box inertia (solid cuboid)
    ixx = (mass / 12.0) * (height**2 + depth**2)
    iyy = (mass / 12.0) * (width**2 + depth**2)
    izz = (mass / 12.0) * (width**2 + height**2)

    r, g, b, a = rgba
    return f"""<?xml version="1.0"?>
<robot name="{name}">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="{width} {height} {depth}"/>
      </geometry>
      <material name="box_material">
        <color rgba="{r} {g} {b} {a}"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="{width} {height} {depth}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="{mass}"/>
      <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>
    </inertial>
  </link>
</robot>
"""
