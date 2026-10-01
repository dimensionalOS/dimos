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

"""Project generated coordinator feedback into a selected planning group."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.std_msgs.msg import Header

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.utils import (
    filter_joint_state_to_selected_joints,
    normalize_joint_target,
)
from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.msgs.time import to_nanoseconds


def main() -> None:
    group = PlanningGroup("left_arm", ("left/j1", "left/j2"), "base", "tool")
    source = JointState(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="robot"),
        name=["right/j1", "left/j2", "left/j1"],
        position=[0.9, 0.3, -0.2],
    )
    decoded = JointState.decode(source.encode())
    selected = filter_joint_state_to_selected_joints(decoded, group.joint_names)
    target = normalize_joint_target(
        group, JointState(header=source.header, name=["left/j2", "left/j1"], position=[0.5, 0.1])
    )
    space = JointSpace(
        tuple(
            JointCoordinate(
                name=name,
                mechanism_type="revolute",
                topology=CoordinateTopology.INTERVAL,
                lower=-1.0,
                upper=1.0,
                max_velocity=1.0,
                max_acceleration=2.0,
            )
            for name in group.joint_names
        )
    )
    positions = space.from_joint_state(selected)
    assert positions.tolist() == [-0.2, 0.3]
    assert space.from_joint_state(target).tolist() == [0.1, 0.5]
    assert selected.header == target.header == source.header
    assert JointState.decode(target.encode()) == target
    print(f"Coordinator input: names={list(source.name)}, q={list(source.position)}")
    print(f"Selected {group.id}: names={list(selected.name)}, q={positions.tolist()}")
    print(f"Normalized target: q={list(target.position)}; within declared joint limits")
    print(
        f"Source stamp preserved: {to_nanoseconds(selected.header.stamp)}, frame={selected.header.frame_id}"
    )
    print("PASS: generated CDR feedback feeds planning selection and joint-space validation")


if __name__ == "__main__":
    main()
