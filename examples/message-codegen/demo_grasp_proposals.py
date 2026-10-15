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

"""Generate a top-down grasp from CDR cloud data, without a robot or GPU."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import GraspCandidateArray
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.msgs.geometry import pose_matrix
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import to_nanoseconds


def main() -> None:
    points = np.array(
        [[0.4, -0.02, 0.1], [0.4, 0.02, 0.1], [0.6, -0.02, 0.2], [0.6, 0.02, 0.2]],
        dtype=np.float32,
    )
    cloud = pointcloud_from_xyz(
        points,
        header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789)),
    )
    provider = HeuristicGraspModule()
    try:
        proposals = provider.propose_grasps(PointCloud2.decode(cloud.encode()))
        payload = proposals.encode()
        received = GraspCandidateArray.decode(payload)
        assert received.header == cloud.header
        assert len(received.candidates) == 1
        candidate = received.candidates[0]
        matrix = pose_matrix(candidate.pose)
        np.testing.assert_allclose(matrix[:3, 3], [0.5, 0.0, 0.15], atol=1e-7)
        np.testing.assert_allclose(matrix[:3, 2], [0.0, 0.0, -1.0], atol=1e-12)
        print(f"Input: {cloud.width} points in {cloud.header.frame_id}; CDR decoded")
        print(f"Output: dimos_msgs/msg/GraspCandidateArray; {len(payload)} CDR bytes")
        print(f"Source timestamp: {to_nanoseconds(received.header.stamp)} ns")
        print(f"Rank 0: position={matrix[:3, 3].round(6).tolist()}; score={candidate.score}")
        print(f"Approach axis: {matrix[:3, 2].round(6).tolist()}")
        print("PASS: generated CDR proposals preserve the cloud header and point down")
    finally:
        provider.stop()


if __name__ == "__main__":
    main()
