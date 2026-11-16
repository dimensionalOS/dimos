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

import asyncio
import math
import threading
from types import SimpleNamespace

import pytest
from scipy.spatial.transform import Rotation

from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro._connection_test_fakes import LocalTransport
from dimos.robot.galaxea.r1pro.joylo import (
    BASE_FRAME,
    JAWS,
    TORSO,
    UPPER,
    HandsOutput,
    JoyloController,
    JoyloOutput,
)
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo
from dimos.robot.galaxea.r1pro.motion import motion_groups
from dimos.teleop.joycon.mapping import JoyConReading
from dimos.teleop.joycon.raw import encode_readings


@pytest.fixture
def module(mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.time", return_value=100)
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.monotonic", return_value=10)
    instance = R1ProJoylo()
    coordinator = R1ProJoyloCoordinator(motion_limits=motion_groups())
    coordinator.add_task(
        JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=list(UPPER), velocity_limits=dict.fromkeys(UPPER, 3.0)
            )
        )
    )
    instance._motion_limits = coordinator
    instance._controller = JoyloController(dict.fromkeys(UPPER, (-3, 3)), instance.config)
    yield instance
    instance.dispose()
    coordinator.dispose()


@pytest.mark.parametrize("stamp", [99.7, 100.1, math.nan, math.inf])
def test_queued_old_future_or_nonfinite_samples_do_not_renew_input_authority(module, stamp):
    assert not module._fresh("leader", stamp)
    assert module._fresh("leader", 100)


def test_a_headset_on_another_clock_still_feeds_the_hands_and_a_repeated_pose_does_not(module):
    module._controller.set_arm_source("hands")
    # The wall clock here reads 100. The headset's reads 99.6: not this host's clock.
    for side in ("left", "right"):
        module._on_hand(
            side, PoseStamped(ts=99.6, position=[0.3, 0.2, 1.0], orientation=[0, 0, 0, 1])
        )
    assert module._controller.hand_receipts == [10, 10]
    assert module._controller.hands[0] == ((0.3, 0.2, 1.0), (0.0, 0.0, 0.0, 1.0))
    # The same pose sent again, or an older one, does not renew the hand.
    module._controller.hand_receipts = [0, 0]
    module._on_hand(
        "left", PoseStamped(ts=99.6, position=[0.3, 0.2, 1.0], orientation=[0, 0, 0, 1])
    )
    module._on_hand(
        "left", PoseStamped(ts=99.5, position=[0.3, 0.2, 1.0], orientation=[0, 0, 0, 1])
    )
    module._on_hand(
        "left", PoseStamped(ts=math.nan, position=[0.3, 0.2, 1.0], orientation=[0, 0, 0, 1])
    )
    assert module._controller.hand_receipts == [0, 0]
    module._on_hand(
        "left", PoseStamped(ts=99.62, position=[0.3, 0.2, 1.0], orientation=[0, 0, 0, 1])
    )
    # A stamp far behind is a new headset session, not an old sample.
    module._on_hand(
        "right", PoseStamped(ts=40.0, position=[0.3, -0.2, 1.0], orientation=[0, 0, 0, 1])
    )
    assert module._controller.hand_receipts == [10, 10]
    # With the leader arms as the source, hand poses are not taken.
    module._controller.set_arm_source("leader")
    module._on_hand(
        "left", PoseStamped(ts=99.7, position=[0.3, 0.2, 1.0], orientation=[0, 0, 0, 1])
    )
    assert module._controller.hands == [None, None]


async def test_the_operators_head_is_taken_in_the_frame_of_the_hands(module):
    module._controller.set_arm_source("hands")
    # The headset's axes: x right, y up, z back. The head is 1.5 m up and faces left.
    left = Rotation.from_euler("Y", 90, degrees=True)
    await module.handle_webxr_head(
        PoseStamped(ts=99.6, position=[0.1, 1.5, -0.2], orientation=list(left.as_quat()))
    )
    position, heading = module._controller.head
    assert position == pytest.approx((0.2, -0.1, 1.5))
    assert heading == pytest.approx(math.pi / 2)
    assert module._controller.head_receipt == 10
    # Looking down at the hands, with the head tilted, is the same heading. So is straight down.
    for stamp, pitch, tilt in ((99.7, -60, 15), (99.75, -90, 0)):
        down = left * Rotation.from_euler("XZ", [pitch, tilt], degrees=True)
        await module.handle_webxr_head(
            PoseStamped(ts=stamp, position=[0.1, 1.5, -0.2], orientation=list(down.as_quat()))
        )
        assert module._controller.head[1] == pytest.approx(math.pi / 2, abs=1e-6)
    # Facing the way the headset's space lies is a heading of zero. A repeated pose is not taken.
    await module.handle_webxr_head(
        PoseStamped(ts=99.8, position=[0, 1.5, 0], orientation=[0, 0, 0, 1])
    )
    assert module._controller.head == ((0, 0, 1.5), 0)
    module._controller.head_receipt = 0
    await module.handle_webxr_head(
        PoseStamped(ts=99.8, position=[0, 1.5, 0], orientation=[0, 0, 0, 1])
    )
    assert module._controller.head_receipt == 0


def test_hand_targets_go_out_in_the_robot_base_frame(module):
    sent = []
    module.hand_left_pose.publish = sent.append
    module.hand_right_pose.publish = sent.append
    module.hand_buttons.publish = lambda _: None
    pose = ((0.3, 0.2, 1.1), (0.0, 0.0, 0.0, 1.0))
    module._publish_hands(HandsOutput(True, (pose, pose)))
    assert [message.frame_id for message in sent] == [BASE_FRAME, BASE_FRAME]
    assert sent[0].position.z == pytest.approx(1.1)


def _host(module, left, right, stamp):
    sample = encode_readings(left, right)
    sample.ts = stamp
    asyncio.run(module.handle_joycon_sample(sample))


def _headset(module, stamp, *down, axes=(0.0, 0.0, 0.0, 0.0)):
    buttons = [int(i in down) for i in range(18)]
    sample = Joy(ts=stamp, frame_id="standard", axes=list(axes), buttons=buttons)
    asyncio.run(module.handle_webxr_gamepad(sample))


def test_a_joycon_pair_on_the_headset_is_the_pair_when_this_host_has_none(module, mocker):
    c = module._controller
    # The reader on this host runs and finds no Joy-Con: both sides are missing.
    _host(module, None, None, 99.90)
    assert c.readings == (None, None)
    # The headset's page sends the pair: X down, left stick forward.
    _headset(module, 5.0, 3, axes=(0.0, -1.0, 0.0, 0.0))
    assert c.readings[1].pressed == {"X"} and c.readings[0].stick == (0.0, -1.0)
    assert module.status()["joycon_source"] == ["headset", "headset"]
    # The host's empty samples go on and take nothing away.
    _host(module, None, None, 99.92)
    assert c.readings[1].pressed == {"X"} and c.receipts["joycon"] == 10
    # A Joy-Con this host reads is that side's Joy-Con, whatever the headset says of it.
    _host(module, JoyConReading(frozenset({"ZL"})), None, 99.94)
    assert c.readings[0].pressed == {"ZL"} and c.readings[1].pressed == {"X"}
    assert module.status()["joycon_source"] == ["host", "headset"]
    # A pad in a layout that is not known is no Joy-Con.
    asyncio.run(
        module.handle_webxr_gamepad(Joy(ts=5.1, frame_id="", axes=[0.0] * 4, buttons=[0] * 18))
    )
    assert c.readings == (JoyConReading(frozenset({"ZL"})), None)


def test_a_late_headset_pad_holds_and_a_lost_one_is_gone(module, mocker):
    c = module._controller
    monotonic = mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.monotonic")
    monotonic.return_value = 10.0
    _headset(module, 5.0, 3)
    assert c.readings[1].pressed == {"X"} and c.receipts["joycon"] == 10.0
    # 0.4 s with no pad from the headset: the host's empty samples do not say "no Joy-Con" yet.
    # The controller is given nothing, so its own watchdog holds for the late input.
    monotonic.return_value = 10.4
    _host(module, None, None, 99.90)
    assert c.readings[1].pressed == {"X"} and c.receipts["joycon"] == 10.0
    # The pad is back inside the recovery window.
    _headset(module, 5.5)
    assert c.readings[1].pressed == frozenset() and c.receipts["joycon"] == 10.4
    # More than the recovery window with no pad: the Joy-Cons are gone.
    monotonic.return_value = 11.6
    _host(module, None, None, 99.92)
    assert c.readings == (None, None) and c.receipts["joycon"] == 11.6
    assert module.status()["joycon_source"] == [None, None]


def test_duplicates_and_reordering_are_rejected_per_input_source(module):
    assert module._fresh("leader", 99.9)
    assert not module._fresh("leader", 99.9)
    assert not module._fresh("leader", 99.8)
    assert module._fresh("joycon", 99.8)
    assert module._fresh("leader", 100)


def test_separate_hardware_feedback_packets_form_complete_measured_hold(module):
    module._on_robot_feedback(JointState(ts=99.9, name=list(UPPER), position=[0.2] * len(UPPER)))
    assert module._controller.step(10, 0.01) is None
    module._on_robot_feedback(JointState(ts=100, name=list(JAWS), position=[0.03, 0.07]))
    output = module._controller.step(10, 0.01)
    assert output.positions == dict.fromkeys(UPPER, 0.2)
    assert output.openings == pytest.approx((0.3, 0.7))
    assert output.velocity == (0, 0, 0)
    assert not module.status()["active"]
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.1)


@pytest.mark.parametrize("stale", [UPPER, JAWS])
def test_fresh_other_joints_cannot_renew_stale_feedback(module, mocker, stale):
    names = [*UPPER, *JAWS]
    module._on_robot_feedback(JointState(ts=100, name=names, position=[0.1] * len(names)))
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.time", return_value=100.3)
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.monotonic", return_value=10.3)
    fresh = [name for name in names if name not in stale]
    module._on_robot_feedback(JointState(ts=100.3, name=fresh, position=[0.2] * len(fresh)))
    assert module._controller.step(10.3, 0.01) is None
    assert "robot" in module.status()["reason"]
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.3)


def test_delayed_feedback_does_not_overwrite_new_pose_or_refresh_its_age(module):
    names = [*UPPER, *JAWS]
    module._on_robot_feedback(JointState(ts=99.95, name=names, position=[0.1] * len(names)))
    module._on_robot_feedback(JointState(ts=99.9, name=names, position=[0.9] * len(names)))
    output = module._controller.step(10, 0.01)
    assert output.positions == dict.fromkeys(UPPER, 0.1)
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.05)


@pytest.mark.asyncio
async def test_desktop_speed_input_changes_controller_without_enabling_it(module):
    await module.handle_arm_tracking_speed(Float32(data=0.75))
    await module.handle_drive_speed(Float32(data=0.3))
    assert module._motion_limits.get_motion_limits()["arm_rate"]["rate"] == 0.75
    assert module._controller.config.base_speed == 0.3
    assert not module.status()["active"]


@pytest.mark.parametrize("bad", [-1, math.nan, math.inf, 4])
def test_invalid_speed_request_changes_nothing(module, bad):
    before = module.set_motion_speeds()
    with pytest.raises(ValueError):
        module.set_motion_speeds(base_speed=0.3, arm_rate=bad)
    assert module.set_motion_speeds() == before


def test_deployment_speed_ceiling_and_zero_rate(module):
    with pytest.raises(ValueError):
        module.set_motion_speeds(arm_rate=1.1)
    assert module.set_motion_speeds(arm_rate=0)["arm_rate"] == 0


def test_speed_readback_describes_live_values_and_deployment_bounds(module):
    module.set_motion_speeds(base_speed=0.4, yaw_speed=0.8)
    settings = module.get_motion_speeds()
    assert settings["settings"][settings["drive"]["linear"]]["value"] == 0.4
    assert settings["settings"][settings["drive"]["angular"]]["value"] == 0.8
    assert settings["settings"]["arm_rate"]["max"] == 1.0
    assert not module.status()["active"]


def test_feedback_burst_keeps_arm_and_jaw_updates_while_async_loop_is_busy(module, mocker):
    model = SimpleNamespace(
        joints=[SimpleNamespace(name=name, lower=-3.0, upper=3.0) for name in UPPER]
    )
    mocker.patch.object(type(module.config.model), "load", return_value=model)
    mocker.patch.object(module, "_run")  # Only exercise feedback; no periodic command production.
    ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
    for name, transport in ports.items():
        module.set_transport(name, transport)
    module.start()
    entered, release = threading.Event(), threading.Event()

    def busy_loop():
        entered.set()
        release.wait(timeout=5)

    try:
        module._loop.call_soon_threadsafe(busy_loop)
        assert entered.wait(timeout=2)
        ports["robot_feedback"].publish(
            JointState(ts=99.9, name=list(UPPER), position=[0.2] * len(UPPER))
        )
        ports["robot_feedback"].publish(JointState(ts=100, name=list(JAWS), position=[0.03, 0.07]))
        # A chassis packet follows on the same topic, with velocity and no position.
        ports["robot_feedback"].publish(JointState(ts=100, name=["base"], velocity=[0]))
        output = module._controller.step(10, 0.01)
        assert output is not None
        assert output.positions == dict.fromkeys(UPPER, 0.2)
        assert output.openings == pytest.approx((0.3, 0.7))
        assert module.status()["input_age_s"]["robot"] == pytest.approx(0.1)
    finally:
        release.set()
        module.stop()


def test_hand_mode_publishes_explicit_torso_targets_without_leader_arm_targets(module, mocker):
    output = JoyloOutput(
        dict.fromkeys(UPPER, 0.1), (0.5, 0.5), (0.0, 0.0, 0.0), HandsOutput(False, None)
    )
    mocker.patch.object(module._controller, "step", return_value=output)
    sent = {name: mocker.patch.object(getattr(module, name), "publish") for name in module.outputs}
    module.tick(10, 0.01)
    target = sent["leader_joint_command"].call_args.args[0]
    assert target.name == list(TORSO)
    assert target.position == [0.1] * 4
    assert target.frame_id == "dimos/guarded-joint"
