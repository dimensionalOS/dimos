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
from collections.abc import Iterator, Mapping
import threading
import time

from dimos_lcm.std_msgs import Bool
from dimos_lcm.vision_msgs import (
    BoundingBox3D,
    Detection3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
import pytest

from dimos.agents.typesafe.agent import TypeSafeAgent
from dimos.agents.typesafe.client import API_KEY_ENV, Answer, Answers, Question
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


def _choice(label: str, *options: str) -> Answer:
    return {
        "type": "choice",
        "choice": label,
        "confidence": 1.0,
        "probabilities": {o: float(o == label) for o in options},
    }


def _answers(x: str = "none", stop: float = 0.0, task: str = "continue") -> Answers:
    return {
        "task": _choice(task, "finished", "continue"),
        "target": _choice("chair", "chair", "none"),
        "drive.x": _choice(x, "forward", "none", "backward"),
        "drive.y": _choice("none", "left", "none", "right"),
        "drive.yaw": _choice("none", "turn_left", "none", "turn_right"),
        "stop": {"type": "noul", "noul": stop},
    }


class FakeSystemOne:
    def __init__(self) -> None:
        self.answers = _answers()
        self.calls = 0
        self.fail = False
        self.block = threading.Event()
        self.block.set()

    def __call__(self, state: object, questions: Mapping[str, Question]) -> Answers:
        self.calls += 1
        self.block.wait(5)
        if self.fail:
            raise RuntimeError("boom")
        return self.answers

    def close(self) -> None:
        pass


Rig = tuple[TypeSafeAgent, FakeSystemOne, list[Twist]]


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch) -> Iterator[Rig]:
    monkeypatch.setenv(API_KEY_ENV, "test-key")
    a = TypeSafeAgent(
        rate_hz=10.0, linear_accel=100.0, angular_accel=100.0, stale_s=5.0, give_up_s=0.3
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
    ):
        getattr(a, name).transport = pLCMTransport(f"/test_typesafe/{name}")
    twists: list[Twist] = []
    unsub = a.cmd_vel.transport.subscribe(twists.append)
    a.start()
    fake = FakeSystemOne()
    a._client = fake  # type: ignore[assignment]
    yield a, fake, twists
    unsub()
    a.stop()


def _scene(a: TypeSafeAgent, robot_x: float = 0.0) -> None:
    a.odom.transport.publish(PoseStamped(position=(robot_x, 0, 0.4), frame_id="world"))
    d = Detection3D()
    d.header = Header(1.0, "world")
    d.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id="chair", score=0.9))]
    d.results_length = 1
    d.bbox = BoundingBox3D(center=Pose(position=(3.0, 0.0, 0.3)), size=Vector3(0.5, 0.5, 0.6))
    a.detections_3d.transport.publish(
        Detection3DArray(detections_length=1, header=Header(1.0, "world"), detections=[d])
    )
    time.sleep(0.1)


def _wait(pred: object, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():  # type: ignore[operator]
            return True
        time.sleep(0.02)
    return False


def _moving(twists: list[Twist]) -> bool:
    return any(t.linear.x > 0.4 for t in twists)


def test_idle_without_goal(rig: Rig) -> None:
    _, fake, twists = rig
    time.sleep(0.3)
    assert fake.calls == 0 and not _moving(twists)


def test_holds_without_odom_or_detections(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = _answers(x="forward")
    a.set_goal("go to the chair")
    time.sleep(0.3)
    assert fake.calls == 0 and not _moving(twists)


def test_goal_drives_then_stops_and_clears(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = _answers(x="forward")
    _scene(a)
    a.set_goal("go to the chair")
    assert _wait(lambda: _moving(twists))
    fake.answers = _answers(stop=0.95)
    assert _wait(lambda: a.goal() is None)
    assert twists[-1].is_zero()


def test_finished_answer_publishes_and_clears(rig: Rig) -> None:
    a, fake, _ = rig
    done: list[Bool] = []
    a.finished.transport.subscribe(done.append)
    fake.answers = _answers(x="forward", task="finished")
    _scene(a, robot_x=2.8)
    a.set_goal("briefing line\ngo to the chair")
    assert a.goal() in (None, "go to the chair")
    assert _wait(lambda: a.goal() is None)
    assert _wait(lambda: bool(done)) and done[0].data is True


def test_arrival_by_distance(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = _answers(x="forward")
    _scene(a, robot_x=2.8)  # 0.2 m from the chair
    a.set_goal("go to the chair")
    assert _wait(lambda: a.goal() is None)
    assert not _moving(twists)


def test_inference_failure_zeroes_target(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = _answers(x="forward")
    _scene(a)
    a.set_goal("go to the chair")
    assert _wait(lambda: _moving(twists))
    fake.fail = True
    assert _wait(lambda: twists[-1].is_zero())


def test_deadman_zeroes_when_inference_stalls(rig: Rig) -> None:
    a, fake, twists = rig
    a.config.deadman_s = 0.3
    fake.answers = _answers(x="forward")
    _scene(a)
    a.set_goal("go to the chair")
    assert _wait(lambda: _moving(twists))
    fake.block.clear()
    assert _wait(lambda: twists[-1].is_zero(), timeout=3.0)
    fake.block.set()


def test_odometry_feeds_pose(rig: Rig) -> None:
    a, fake, twists = rig
    fake.answers = _answers(x="forward")
    _scene(a)
    a.odometry.transport.publish(
        Odometry(ts=1.0, frame_id="world", pose=Pose(position=(2.8, 0, 0.4)))
    )
    time.sleep(0.1)
    a.set_goal("go to the chair")
    assert _wait(lambda: a.goal() is None)  # 0.2 m from the chair: arrival, not driving
    assert not _moving(twists)
