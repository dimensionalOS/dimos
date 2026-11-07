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

import json
from pathlib import Path

import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.sim.spatial import R1ProSpatialTeleop
from dimos.teleop.spatial.model import SpatialGroup
from dimos.teleop.webxr.controller_types import Hand


@pytest.fixture
def station(mocker):
    # Geometry is not loaded when exercising the operator protocol.
    module = R1ProSpatialTeleop(
        robot_model=RobotModelConfig(
            model=RobotModel.from_file(Path("/unused.urdf")), joint_names=["shoulder"]
        ),
        groups={
            "left": SpatialGroup(
                label="Arm", joint_names=("shoulder",), tip_frame="tool", base_frame="base"
            )
        },
        base_drive=True,
    )
    for port in (
        module.cmd_vel,
        module.teleop_buttons,
        module.button_pressed,
        module.button_released,
    ):
        mocker.patch.object(port, "publish")
    clock = mocker.patch("dimos.teleop.spatial.module.time.monotonic", return_value=10.0)
    module._feedback = {
        "ts": 1000,
        "source_ts": 0.1,
        "joints": {"shoulder": 0},
        "arms": {"left": {"grasp": [0, 0, 1], "orientation": [0, 0, 0, 1], "opening": 0}},
    }
    module._feedback_received = 10.0
    module._operator_paused = False
    try:
        yield module, clock
    finally:
        module.stop()


def packet(module, sequence=0, drive=(1, 0, 0), hands=None):
    return json.dumps(
        dict(
            type="spatial_input",
            session=module._operator_session,
            sequence=sequence,
            control_epoch=module._control_epoch,
            observed_at=1000,
            hands=hands or {},
            drive=drive,
        )
    )


def velocity(module):
    command = module.cmd_vel.publish.call_args.args[0]
    return command.linear.x, command.linear.y, command.angular.z


def test_drive_expires_and_queued_packet_cannot_revive_it(station):
    module, clock = station
    old_packet = packet(module)
    assert module._dispatch_text_message(old_packet)
    assert velocity(module) == (0.2, 0, 0)
    clock.return_value = 10.31
    module._expire_stale_state(clock.return_value)
    assert velocity(module) == (0, 0, 0)
    assert not module._dispatch_text_message(old_packet)
    assert velocity(module) == (0, 0, 0)


@pytest.mark.parametrize("drive", [(2, 0, 0), (float("nan"), 0, 0)])
def test_invalid_drive_stops_motion(station, drive):
    module, _ = station
    assert module._dispatch_text_message(packet(module))
    assert not module._dispatch_text_message(packet(module, 1, drive))
    assert velocity(module) == (0, 0, 0)


def test_sim_clock_does_not_make_fresh_feedback_stale(station):
    module, clock = station
    assert module._joint_feedback_ready(1794000000)
    clock.return_value = 10.51
    assert not module._joint_feedback_ready(1794000000)


def test_arm_requires_release_before_grab_and_disconnect_releases(station):
    module, _ = station
    hand = dict(
        position=[0, 0, 1], orientation=[0, 0, 0, 1], pinched=True, action="move", arm="left"
    )
    assert module._dispatch_text_message(packet(module, hands={"left": hand}))
    assert module._owner(Hand.LEFT, "move") is None
    assert module._dispatch_text_message(
        packet(module, 1, hands={"left": {**hand, "pinched": False}})
    )
    assert module._dispatch_text_message(packet(module, 2, hands={"left": hand}))
    assert module._owner(Hand.LEFT, "move") == Hand.LEFT
    module._reset_controller_state()
    assert module._owner(Hand.LEFT, "move") is None
    assert velocity(module) == (0, 0, 0)
