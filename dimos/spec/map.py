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

"""Structural protocols decoupling map producers from consumers.

Robotic applications require different map representations for different tasks:
3D geometry for manipulation, 2D occupancy for obstacle detection, and costmaps
for path planning. This module provides structural protocols that allow you to
swap mapping backends without changing downstream code.

A SLAM system, map accumulator, or simulator can satisfy these protocols by declaring
the appropriate output stream.

Choose
- `Global3DMap` for detailed 3D geometry (manipulation, dense reconstruction),
- `GlobalMap` for basic obstacle detection,
- and `GlobalCostmap` for path planning
"""

from typing import Annotated, Protocol

from annotated_doc import Doc

from dimos.core import Out
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2


class Global3DMap(Protocol):
    """Protocol for modules providing a global 3D point cloud map.

    This protocol defines the interface for modules that accumulate and publish
    a 3D point cloud representation of the environment. Unlike instantaneous
    sensor scans, the global point cloud represents accumulated spatial knowledge
    built over time (e.g., from SLAM or incremental mapping).

    The protocol enables decoupling between map producers and consumers, allowing
    different mapping backends (ROS2 SLAM, custom mappers, simulation) to be
    substituted without changing downstream code.

    Example:
        Implementing the protocol::

            class CustomMapper(Module):
                global_pointcloud: Out[PointCloud2] = None  # Satisfies Global3DMap

                def publish_map(self, points: np.ndarray):
                    pc = PointCloud2.from_numpy(points, frame_id="map", timestamp=time.time())
                    self.global_pointcloud.publish(pc)

    Notes:
        - Implementations in `dimos/navigation/rosnav.py` (ROSNav module)
        - The protocol specifies the interface but not publish frequency, point
          density, or how the map is accumulated
        - Coordinate frame should be a fixed world/map frame, not a moving robot frame
    """

    global_pointcloud: Annotated[
        Out[PointCloud2],
        Doc(
            """Output stream publishing accumulated 3D point cloud data. The `PointCloud2`
            messages contain Open3D point clouds with spatial coordinates in a fixed
            world/map frame (typically `frame_id="map"`)."""
        ),
    ]


class GlobalMap(Protocol):
    """Protocol for modules providing a global 2D occupancy grid map.

    This protocol defines the interface for modules that publish a 2D occupancy
    grid representing obstacles and free space in the environment. The occupancy
    grid uses the ROS `nav_msgs/OccupancyGrid` convention for cell values.

    Abstracts over different mapping implementations (SLAM, static maps, simulation).

    Example:
        Implementing the protocol::

            class MapProducer(Module):
                global_map: Out[OccupancyGrid] = None  # Satisfies GlobalMap

                def publish_occupancy(self, pointcloud: LidarMessage):
                    grid = OccupancyGrid.from_pointcloud(
                        pointcloud, resolution=0.05, min_height=0.0, max_height=2.0
                    )
                    self.global_map.publish(grid)

    Notes:
        - For 2D occupancy grids used in navigation, `GlobalCostmap` is more commonly
          consumed as it supports cost gradients for path planning
    """

    global_map: Annotated[
        Out[OccupancyGrid],
        Doc(
            """Output stream publishing 2D occupancy grids. Cell values follow ROS
            conventions: `-1` (unknown/unexplored), `0` (free space), `100` (occupied),
            and `1-99` (intermediate occupancy/cost values, implementation-dependent)."""
        ),
    ]


class GlobalCostmap(Protocol):
    """Protocol for modules providing a global 2D costmap for navigation.

    This protocol defines the interface for modules that publish a 2D costmap
    representation designed specifically for path planning. Unlike `GlobalMap`
    (which represents raw occupancy), costmaps typically include safety margins
    around obstacles and cost gradients to encourage paths that maintain clearance.

    Example:
        Implementing the protocol::

            class CostmapBuilder(Module):
                global_costmap: Out[OccupancyGrid] = None  # Satisfies GlobalCostmap

                def publish_costmap(self, occupancy: OccupancyGrid):
                    # Apply inflation and gradient for navigation
                    costmap = occupancy.inflate(radius=0.2).gradient(max_distance=1.5)
                    self.global_costmap.publish(costmap)

    Notes:
        - Example of an implementation: `dimos/robot/unitree_webrtc/type/map.py`
    """

    global_costmap: Annotated[
        Out[OccupancyGrid],
        Doc(
            """Output stream publishing 2D costmaps. Cell values follow ROS conventions:
            `-1` (unknown/unexplored), `0` (free space with no cost), `1-99` (increasing
            traversal cost from gradient/inflation), and `100` (lethal obstacle, impassable)."""
        ),
    ]
