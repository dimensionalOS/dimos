# Copyright 2026 Dimensional Inc.
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

from dimos.core.rpc_client import RPCClient
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3

client = RPCClient(None, PickAndPlaceModule)
# Table surface in the robot world frame.  Adjust the z offset to match your setup.
# The obstacle top should sit at the real table surface height (typically z ≈ -0.10).
# Note: the xarm_perception blueprint enforces min_tcp_z=0.05, which prevents the real
# gripper fingers from descending below the table even when the URDF collision geometry
# is too simplified to detect it. This floor obstacle still helps the motion planner
# route trajectories above the table.
obstacle_id = client.add_obstacle(
    name="floor",
    pose=Pose(Vector3(0.7, 0.0, -0.120), Quaternion(0.0, 0.0, 0.0, 1.0)),
    shape="box",
    dimensions=[0.7, 1.0, 0.2],
)
print("obstacle_id:", obstacle_id)
client.stop_rpc_client()
