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

import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.agents.typesafe.navigation import TypeSafeNavigationAgent
from dimos.agents.typesafe.test_drive import answers
from dimos.agents.typesafe.test_world_state import det3d
from dimos.agents.typesafe.types import Answers, Question
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
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
    a.goal.transport = LCMTransport("/test_typesafe/goal", PointStamped)
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


def test_goal_point_published_and_used_when_target_leaves_view(rig: Rig) -> None:
    a, fake, twists = rig
    goals: list[PointStamped] = []
    unsub = a.goal.transport.subscribe(goals.append)
    fake.answers = answers(x="forward")
    a.set_goal("go to the chair")
    scene(a)
    assert until(lambda: moving(twists) and bool(goals))
    assert (goals[0].x, goals[0].y, goals[0].frame_id) == (3.0, 0.0, "world")
    # detections go stale, odom keeps coming: the latched goal point keeps it driving
    a.config.stale_s = 0.2
    time.sleep(0.3)
    calls = fake.calls
    for _ in range(6):
        odom(a, 0.5)
        time.sleep(0.15)
    assert fake.calls > calls
    assert a.current_goal() == "go to the chair" and twists[-1].linear.x > 0.4
    unsub()


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
    assert a._goal_xy is None
