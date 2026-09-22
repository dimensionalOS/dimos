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

"""Inspect a Go2 device pose and generated CDR TF chain without connecting to a robot."""

from dimos_generated.tf2_msgs.msg import TFMessage

from dimos.msgs.geometry import compose_transforms
from dimos.msgs.time import to_nanoseconds
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.type.odometry import pose_from_webrtc_odometry, raw_odometry_msg_sample


def main() -> None:
    pose = pose_from_webrtc_odometry(raw_odometry_msg_sample)
    edges = GO2Connection._odom_to_tf(pose, prefix="robot0")
    decoded = TFMessage.decode(TFMessage(transforms=edges).encode())
    assert len(decoded.transforms) == 3
    print(f"Device pose: frame={pose.header.frame_id}, stamp={to_nanoseconds(pose.header.stamp)}")
    for edge in decoded.transforms:
        assert edge.header.stamp == pose.header.stamp
        translation = edge.transform.translation
        print(
            f"  {edge.header.frame_id} → {edge.child_frame_id}: "
            f"xyz=({translation.x:.6f}, {translation.y:.6f}, {translation.z:.6f})"
        )
    base, camera, optical = decoded.transforms
    result = compose_transforms(compose_transforms(base, camera), optical)
    assert result.header == pose.header
    assert result.child_frame_id == "robot0/camera_optical"
    print(f"Composed camera frame: {result.header.frame_id} → {result.child_frame_id}")
    print(f"Exact camera stamp: {to_nanoseconds(result.header.stamp)}")
    print("PASS: original ROS header, namespaced local frames, generated CDR round trip")


if __name__ == "__main__":
    main()
