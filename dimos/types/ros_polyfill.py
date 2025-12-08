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

try:
    from geometry_msgs.msg import Vector3  # type: ignore[attr-defined, import-untyped]
except ImportError:
    from dimos.msgs.geometry_msgs import Vector3

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        Point,
        Pose,
        Quaternion,
        Twist,
    )
    from nav_msgs.msg import OccupancyGrid, Odometry  # type: ignore[attr-defined, import-untyped]
    from std_msgs.msg import Header  # type: ignore[attr-defined, import-untyped]
except ImportError:
    from dimos_lcm.geometry_msgs import (  # type: ignore[import-untyped, no-redef]
        Point,
        Pose,
        Quaternion,
        Twist,
    )
    from dimos_lcm.nav_msgs import OccupancyGrid, Odometry  # type: ignore[import-untyped, no-redef]
    from dimos_lcm.std_msgs import Header  # type: ignore[import-untyped, no-redef]

__all__ = [
    "Header",
    "OccupancyGrid",
    "Odometry",
    "Point",
    "Pose",
    "Quaternion",
    "Twist",
    "Vector3",
]
