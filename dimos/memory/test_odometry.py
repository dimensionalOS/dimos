# Copyright 2025-2026 Dimensional Inc.
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

from pathlib import Path

from dimos.memory.odometry import as_odometry
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry

POSES = [
    PoseStamped(ts=1.0, frame_id="world", position=[1.0, 2.0, 0.0], orientation=[0, 0, 0, 1]),
    PoseStamped(ts=2.0, frame_id="world", position=[3.0, 1.0, 0.5], orientation=[0, 0, 1, 0]),
]


def test_old_and_new_recordings_give_the_same_poses(tmp_path: Path) -> None:
    with SqliteStore(path=str(tmp_path / "old.db")) as store:
        old = store.stream("odom", PoseStamped)
        for pose in POSES:
            old.append(pose, ts=pose.ts)
    with SqliteStore(path=str(tmp_path / "new.db")) as store:
        new = store.stream("odom", Odometry)
        for pose in POSES:
            odometry = Odometry(
                ts=pose.ts,
                frame_id=pose.frame_id,
                child_frame_id="base_link",
                pose=Pose(pose.position, pose.orientation),
                twist=Twist([0.3, 0.0, 0.0], [0.0, 0.0, 0.1]),
            )
            new.append(odometry, ts=pose.ts)

    for name in ("old.db", "new.db"):
        with SqliteStore(path=str(tmp_path / name)) as store:
            read = [as_odometry(obs.data) for obs in store.stream("odom")]
        assert all(isinstance(odom, Odometry) for odom in read)
        assert [odom.to_pose_stamped() for odom in read] == POSES
