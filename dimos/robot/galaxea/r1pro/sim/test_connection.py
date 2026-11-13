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

import math
import xml.etree.ElementTree as ET

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.geometry_msgs import Pose, Quaternion, Twist, Vector3
from dimos_lcm.sim_msgs import BodyState, WorldState
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES
from dimos.robot.galaxea.r1pro.joylo import JAWS, UPPER
from dimos.robot.galaxea.r1pro.sim.connection import R1ProSimConnection


@pytest.fixture
def connection(tmp_path, mocker):
    root = ET.Element("robot", name="fixture")
    ET.SubElement(root, "link", name="root")
    for i, name in enumerate((*UPPER, *JAWS, *R1PRO_BODY_AXES)):
        child = f"link{i}"
        ET.SubElement(root, "link", name=child)
        joint = ET.SubElement(root, "joint", name=name, type="prismatic")
        ET.SubElement(joint, "parent", link="root")
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "axis", xyz="1 0 0")
        lo, hi = (0, 0.1) if name in JAWS else (-2, 2)
        ET.SubElement(joint, "limit", lower=str(lo), upper=str(hi), effort="30", velocity="2")
    path = tmp_path / "robot.urdf"
    path.write_text(ET.tostring(root, encoding="unicode"))
    module = R1ProSimConnection(model=RobotModel.from_file(path))
    ports = {name: mocker.patch.object(getattr(module, name), "publish") for name in module.outputs}
    yield module, ports
    module.dispose()


@pytest.mark.asyncio
async def test_opposing_fingers_become_canonical_jaw_openings_and_halt_clamps(connection):
    module, ports = connection
    positions = dict.fromkeys(UPPER, 0.25)
    positions[UPPER[0]] = 2.01  # Physics can overshoot a joint limit slightly.
    positions.update(
        {
            "r1pro/left_gripper_finger_joint1": 0.025,
            "r1pro/left_gripper_finger_joint2": -0.025,
            "r1pro/right_gripper_finger_joint1": 0.05,
            "r1pro/right_gripper_finger_joint2": -0.05,
        }
    )
    await module.handle_encoder(
        JointState(ts=1, name=list(positions), position=list(positions.values()))
    )
    feedback = ports["joint_state"].call_args.args[0]
    measured = dict(zip(feedback.name, feedback.position, strict=True))
    assert (measured[JAWS[0]], measured[JAWS[1]]) == pytest.approx((0.05, 0.1))
    module.halt()
    hold = ports["upper_command"].call_args.args[0]
    assert hold.position[0] == 2
    assert ports["servo_left_gripper"].call_args.args[0].data == pytest.approx(0.5)
    assert ports["servo_right_gripper"].call_args.args[0].data == pytest.approx(1)
    velocity = ports["servo_velocity"].call_args.args[0]
    assert (velocity.linear.x, velocity.linear.y, velocity.angular.z) == (0, 0, 0)


@pytest.mark.asyncio
async def test_world_velocity_is_reported_in_the_rotated_body_frame(connection):
    module, ports = connection
    body = BodyState(
        body_id="r1pro/base_link",
        pose=Pose(orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5))),
        twist=Twist(linear=Vector3(x=0.2, y=0.4), angular=Vector3(z=0.3)),
    )
    await module.handle_observations(
        WorldState(header=SampleHeader(time_ns=2_000_000_000), bodies=[body])
    )
    sample = ports["joint_state"].call_args.args[0]
    assert sample.name == list(R1PRO_BODY_AXES)
    assert sample.position == []
    assert sample.velocity == pytest.approx([0.4, -0.2, 0.3])
    assert sample.ts == 2


def test_canonical_commands_convert_to_servo_units(connection):
    module, ports = connection
    module.write(
        {
            f"{UPPER[0]}/position": 0.5,
            f"{JAWS[0]}/position": 0.025,
            f"{R1PRO_BODY_AXES[0]}/velocity": 0.12,
            f"{R1PRO_BODY_AXES[2]}/velocity": -0.2,
        }
    )
    arm = ports["upper_command"].call_args.args[0]
    assert arm.name == [UPPER[0]] and arm.position == [0.5]
    assert ports["servo_left_gripper"].call_args.args[0].data == pytest.approx(0.25)
    ports["servo_right_gripper"].assert_not_called()
    velocity = ports["servo_velocity"].call_args.args[0]
    assert (velocity.linear.x, velocity.linear.y, velocity.angular.z) == (0.12, 0, -0.2)
