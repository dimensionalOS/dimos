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

from dimos_generated.builtin_interfaces.msg import Duration, Time
from dimos_generated.dimos_msgs.msg import TrajectoryStatus
from dimos_generated.std_msgs.msg import Header
from dimos_generated.trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pytest

from dimos.msgs.time import duration_from_seconds, to_nanoseconds
from dimos.msgs.trajectory import TrajectoryState, sample_trajectory, trajectory_duration


@pytest.mark.parametrize(
    "seconds,expected", [(-1.0, [1.0, 2.0]), (0.5, [2.0, 1.0]), (3.0, [3.0, 0.0])]
)
@pytest.mark.parametrize("with_velocities", [False, True])
def test_cdr_trajectory_sampling_and_copy_isolation(seconds, expected, with_velocities):
    trajectory = JointTrajectory(
        header=Header(frame_id="robot", stamp=Time(sec=1700000000, nanosec=123456789)),
        joint_names=["a", "b"],
        points=[
            JointTrajectoryPoint(
                positions=[1.0, 2.0], velocities=[0.0, 1.0] if with_velocities else []
            ),
            JointTrajectoryPoint(
                positions=[3.0, 0.0],
                velocities=[2.0, 3.0] if with_velocities else [],
                time_from_start=duration_from_seconds(1.0),
            ),
        ],
    )
    decoded = JointTrajectory.decode(trajectory.encode())
    positions, velocities = sample_trajectory(decoded, seconds)
    assert positions == expected
    alpha = max(0.0, min(seconds, 1.0))
    assert velocities == ([2 * alpha, 1 + 2 * alpha] if with_velocities else [0.0, 0.0])
    positions[0] = 999.0
    assert list(decoded.points[0].positions) == [1.0, 2.0]
    assert trajectory_duration(decoded) == 1.0
    assert to_nanoseconds(decoded.header.stamp) == 1700000000123456789


def test_empty_trajectory_sampling_and_invalid_sample_time():
    assert trajectory_duration(JointTrajectory()) == 0.0
    assert sample_trajectory(JointTrajectory(), 0.0) == ([], [])
    with pytest.raises(ValueError, match="finite"):
        sample_trajectory(JointTrajectory(), float("nan"))


def test_generated_status_roundtrips_duration_fields_and_state_constants():
    status = TrajectoryStatus(
        header=Header(stamp=Time(sec=0, nanosec=7)),
        state=TrajectoryState.EXECUTING,
        progress=0.25,
        time_elapsed=Duration(sec=1, nanosec=123456789),
        time_remaining=Duration(sec=3, nanosec=987654321),
    )
    decoded = TrajectoryStatus.decode(status.encode())
    assert decoded == status
    assert decoded.state == TrajectoryStatus.EXECUTING
    assert TrajectoryState(decoded.state).name == "EXECUTING"
    assert to_nanoseconds(decoded.time_elapsed) == 1123456789
    assert to_nanoseconds(decoded.time_remaining) == 3987654321
