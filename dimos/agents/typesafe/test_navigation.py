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
from collections.abc import Iterator
import threading
import time

from dimos_lcm.std_msgs import Bool
import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.agents.typesafe.navigation import TypeSafeNavigationAgent
from dimos.agents.typesafe.test_drive import answers
from dimos.agents.typesafe.test_world_state import det2d, det3d
from dimos.agents.typesafe.types import Answers, Question
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry


class FakeSystemOne:
    def __init__(self) -> None:
        self.answers = answers()
        self.calls = 0
        self.fail = False
        self.gate = threading.Event()
        self.gate.set()

    def __call__(self, state: object, questions: dict[str, Question]) -> Answers:
        self.calls += 1
        self.gate.wait(5)
        if self.fail:
            raise RuntimeError("boom")
        return self.answers


Rig = tuple[TypeSafeNavigationAgent, FakeSystemOne, list[Twist]]


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch) -> Iterator[Rig]:
    scheduler = ThreadPoolScheduler(max_workers=2)
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    a = TypeSafeNavigationAgent(
        max_hz=None, linear_accel=100.0, angular_accel=100.0, stale_s=5.0, give_up_s=0.3
    )
    a.odom.transport = LCMTransport("/test_typesafe/odom", PoseStamped)
    a.cmd_vel.transport = LCMTransport("/test_typesafe/cmd_vel", Twist)
    a.finished.transport = LCMTransport("/test_typesafe/finished", Bool)
    for name in (
        "odometry",
        "detections_3d",
        "detections_2d",
        "lidar",
        "human_input",
        "agent",
        "agent_idle",
        "choices",
        "scores",
        "nouls",
    ):
        getattr(a, name).transport = pLCMTransport(f"/test_typesafe/{name}")
    twists: list[Twist] = []
    unsub = a.cmd_vel.transport.subscribe(twists.append)
    fake = FakeSystemOne()
    a._post = fake  # type: ignore[method-assign]
    a.start()
    yield a, fake, twists
    unsub()
    a.stop()
    scheduler.executor.shutdown(wait=True)


def odom(a: TypeSafeNavigationAgent, x: float = 0.0) -> None:
    a.odom.transport.publish(PoseStamped(position=(x, 0, 0.4), frame_id="world"))


def scene(a: TypeSafeNavigationAgent, robot_x: float = 0.0) -> None:
    """Chair at (3, 0); robot on the x axis facing it. Odom is the trigger, so it goes last."""
    a.detections_3d.transport.publish(det3d("chair", 3.0, 0.0))
    time.sleep(0.05)
    odom(a, robot_x)
    time.sleep(0.1)


def until(pred: object, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():  # type: ignore[operator]
            return True
        time.sleep(0.02)
    return False


def moving(twists: list[Twist]) -> bool:
    return any(t.linear.x > 0.4 for t in twists)


def test_idle_without_goal(rig: Rig) -> None:
    a, fake, twists = rig
    scene(a)
    time.sleep(0.3)
    assert fake.calls == 0 and not moving(twists)


def test_holds_without_detections(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    odom(a)
    time.sleep(0.3)
    assert fake.calls == 0 and not moving(twists)


def test_drives_then_stops_and_clears(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    scene(a)
    assert until(lambda: moving(twists))
    fake.answers = answers(stop=0.95)
    assert until(lambda: (odom(a), a.current_goal() is None)[1])
    assert twists[-1].is_zero()


def test_arrival_by_distance(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    scene(a, robot_x=2.8)  # 0.2 m from the chair
    assert until(lambda: (odom(a, 2.8), a.current_goal() is None)[1])
    assert not moving(twists)


def test_odometry_feeds_pose(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    a.detections_3d.transport.publish(det3d("chair", 3.0, 0.0))
    time.sleep(0.05)
    a.odometry.transport.publish(
        Odometry(ts=time.time(), frame_id="world", pose=Pose(position=(2.8, 0, 0.4)))
    )
    assert until(
        lambda: (
            a.odometry.transport.publish(
                Odometry(ts=time.time(), frame_id="world", pose=Pose(position=(2.8, 0, 0.4)))
            ),
            a.current_goal() is None,
        )[1]
    )
    assert not moving(twists)


def test_inference_failure_zeroes_target(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    scene(a)
    assert until(lambda: moving(twists))
    fake.fail = True
    odom(a)
    assert until(lambda: twists[-1].is_zero(), timeout=3.0)


def test_deadman_zeroes_when_inference_stalls(rig: Rig) -> None:
    a, fake, twists = rig
    a.config.deadman_s = 0.3
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    scene(a)
    assert until(lambda: moving(twists))
    fake.gate.clear()
    odom(a)
    assert until(lambda: twists[-1].is_zero(), timeout=3.0)
    fake.gate.set()


def test_goal_change_during_inference_drops_the_stale_answer(rig: Rig) -> None:
    """A reply for the old goal must not move the robot or leak its position into the new goal."""
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    fake.gate.clear()  # the chair request hangs in flight
    scene(a)
    assert until(lambda: fake.calls == 1)
    a.set_goal("go to the door")
    fake.gate.set()  # the chair reply lands after the goal changed
    time.sleep(0.3)
    assert not moving(twists)
    assert a._robot == {"motion": "idle"}


def test_finished_answer_publishes_and_clears(rig: Rig) -> None:
    a, fake, _ = rig
    done: list[Bool] = []
    unsub = a.finished.transport.subscribe(done.append)
    fake.answers = answers(x="forward", task="finished")
    a.set_goal("briefing line\ngo to the chair")
    assert a.current_goal() == "go to the chair"
    scene(a, robot_x=2.8)
    assert until(lambda: (odom(a, 2.8), a.current_goal() is None)[1])
    assert until(lambda: bool(done)) and done[0].data is True
    unsub()


def test_lost_detections_give_up(rig: Rig) -> None:
    """2D detections latch no goal point: when they stop, holding ends with the goal cleared."""
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    a.detections_2d.transport.publish(det2d("chair", 640.0, 200, 200))
    time.sleep(0.05)
    odom(a)
    assert until(lambda: moving(twists))
    a.config.stale_s = 0.0  # every detection is now stale
    assert until(lambda: (odom(a), a.current_goal() is None)[1])
    assert twists[-1].is_zero()


def test_timeout_clears_only_the_goal_that_timed_out(rig: Rig) -> None:
    a, _fake, _twists = rig
    a.set_goal("go to the chair")
    old = a._goal_gen
    a._set_target((0.0, 0.0, 0.0))
    time.sleep(0.4)  # past give_up_s: the chair goal has stood still long enough
    a.set_goal("go to the door")
    assert a._gave_up(old) is False  # the door goal accepted meanwhile is left alone
    assert a.current_goal() == "go to the door"
    assert a._gave_up(a._goal_gen) is False  # the door goal has not stood still yet


def test_2d_target_filling_the_view_counts_as_arrival(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    a.detections_2d.transport.publish(det2d("chair", 640.0, 1000, 600))  # 65% of the image
    time.sleep(0.05)
    odom(a)
    assert until(lambda: fake.calls > 0)
    assert until(lambda: (odom(a), a.current_goal() is None)[1])
    assert not moving(twists)


def test_idle_follows_goal_changes_in_order(rig: Rig) -> None:
    a, _fake, _twists = rig
    idle: list[bool] = []
    unsub = a.agent_idle.transport.subscribe(idle.append)
    a.set_goal("go to the chair")
    old = a._goal_gen
    a._set_target((0.0, 0.0, 0.0))
    time.sleep(0.4)
    a.set_goal("go to the door")
    assert a._gave_up(old) is False
    assert until(lambda: idle[-2:] == [False, False])  # chair, then door; no idle in between
    unsub()
