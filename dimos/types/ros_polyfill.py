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
    from geometry_msgs.msg import Vector3
except ImportError:

    class Vector3:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

        def __repr__(self) -> str:
            return f"Vector3(x={self.x}, y={self.y}, z={self.z})"


try:
    from nav_msgs.msg import OccupancyGrid, Odometry
    from geometry_msgs.msg import Pose, Point, Quaternion, Twist
    from std_msgs.msg import Header
except ImportError:

    class Header:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class Point:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

        def __repr__(self) -> str:
            return f"Point(x={self.x}, y={self.y}, z={self.z})"

    class Quaternion:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.w = float(w)

        def __repr__(self) -> str:
            return f"Quaternion(x={self.x}, y={self.y}, z={self.z}, w={self.w})"

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()

        def __repr__(self) -> str:
            return f"Pose(position={self.position}, orientation={self.orientation})"

    class MapMetaData:
        def __init__(self):
            self.map_load_time = None
            self.resolution = 0.05
            self.width = 0
            self.height = 0
            self.origin = Pose()

        def __repr__(self) -> str:
            return f"MapMetaData(resolution={self.resolution}, width={self.width}, height={self.height}, origin={self.origin})"

    class Twist:
        def __init__(self):
            self.linear = Vector3()
            self.angular = Vector3()

        def __repr__(self) -> str:
            return f"Twist(linear={self.linear}, angular={self.angular})"

    class OccupancyGrid:
        def __init__(self):
            self.header = Header()
            self.info = MapMetaData()
            self.data = []

        def __repr__(self) -> str:
            return f"OccupancyGrid(info={self.info}, data_length={len(self.data)})"

    class Odometry:
        def __init__(self):
            self.header = Header()
            self.child_frame_id = ""
            self.pose = Pose()
            self.twist = Twist()

        def __repr__(self) -> str:
            return f"Odometry(pose={self.pose}, twist={self.twist})"
