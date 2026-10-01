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

from unittest.mock import MagicMock

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.geometry import quaternion_from_euler
from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer
from dimos.simulation.mujoco.person_on_track import PersonTrackPublisher


@pytest.mark.parametrize(
    "heading,expected_linear,expected_angular", [(0.0, 0.8, 0.0), (1.0, 0.0, -1.5)]
)
def test_pursuit_generated_pose_preserves_steering_and_stop(
    heading: float, expected_linear: float, expected_angular: float, monkeypatch: pytest.MonkeyPatch
) -> None:
    explorer = DirectCmdVelExplorer()
    output = MagicMock()
    explorer._cmd_vel = output
    poses = iter(
        [
            PoseStamped(
                header=Header(frame_id="world"),
                pose=Pose(position=Point(), orientation=quaternion_from_euler(0.0, 0.0, heading)),
            ),
            PoseStamped(
                header=Header(frame_id="world"),
                pose=Pose(position=Point(x=1.0), orientation=Quaternion(w=1.0)),
            ),
        ]
    )
    monkeypatch.setattr(explorer, "_wait_for_pose", lambda: next(poses))
    explorer._drive_to(1.0, 0.0)
    commands = [Twist.decode(call.args[1].encode()) for call in output.broadcast.call_args_list]
    assert len(commands) == 2
    assert commands[0].linear.x == expected_linear
    assert commands[0].angular.z == expected_angular
    assert commands[1].linear.x == 0.0
    assert commands[1].angular.z == 0.0


def test_person_track_generated_pose_preserves_position_and_heading(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = MagicMock()
    monkeypatch.setattr(
        "dimos.simulation.mujoco.person_on_track.make_transport", lambda channel, message: output
    )
    publisher = PersonTrackPublisher([(1.0, 2.0), (1.0, 3.0), (2.0, 3.0)])
    publisher.tick()
    pose = Pose.decode(output.broadcast.call_args.args[1].encode())
    assert (pose.position.x, pose.position.y, pose.position.z) == (1.0, 2.0, 0.0)
    assert np.isclose(pose.orientation.z, np.sin(np.pi / 4))
    assert np.isclose(pose.orientation.w, np.cos(np.pi / 4))
