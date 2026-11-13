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

"""R1 leader routing on the shared Feetech connection."""

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import _operator_blueprint
from dimos.robot.galaxea.r1pro.joylo import ARMS
from dimos.robot.galaxea.r1pro.leader import R1ProLeaderConnection


@pytest.mark.parametrize("left_only", [False, True])
def test_operator_ports_and_joint_routes_select_the_requested_arms(left_only, mocker):
    parsed = BlueprintConfigParser(_operator_blueprint(left_only)).parse(
        [
            "--r1proleaderconnection.left-serial-port=left-port",
            "--r1proleaderconnection.right-serial-port=right-port",
        ],
        environ={},
    )
    module = R1ProLeaderConnection(**parsed.module_kwargs("r1proleaderconnection"))
    try:
        joints = mocker.patch.object(module.joint_state, "publish")
        left_jaw = mocker.patch.object(module.left_gripper_command, "publish")
        right_jaw = mocker.patch.object(module.right_gripper_command, "publish")
        readings = {
            side: {f"joint_{i}": sign * i / 10 for i in range(1, 8)}
            for side, sign in (("left", 1), ("right", -1))
        }
        module.publish_reading(readings)
        target = joints.call_args.args[0]
        assert target.name == list(ARMS[:7] if left_only else ARMS)
        assert target.position == pytest.approx(
            [i / 10 for i in range(1, 8)] + ([] if left_only else [-i / 10 for i in range(1, 8)])
        )
        assert module.serial_ports() == (
            {"left": "left-port"} if left_only else {"left": "left-port", "right": "right-port"}
        )
        left_jaw.assert_not_called()
        right_jaw.assert_not_called()
    finally:
        module.stop()
