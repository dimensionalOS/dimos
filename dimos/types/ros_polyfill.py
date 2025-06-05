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
