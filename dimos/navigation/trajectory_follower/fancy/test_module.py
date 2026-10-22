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


import pytest

from dimos.core.module import ModuleConfig
from dimos.message_utils.time import time_from_seconds
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.tf_pose import TfPose
from dimos.navigation.trajectory_follower.fancy.laws import seed
from dimos.navigation.trajectory_follower.fancy.module import (
    GoalLatch,
    TrajectoryFollower,
    TrajectoryFollowerConfig,
)
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNativeConfig
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.tf.tf import MultiTBuffer


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


# The real constructor stands up an LCM RPC transport per instance; the fixture stops them.
_BUILT: list[TrajectoryFollower] = []


@pytest.fixture(autouse=True)
def _stop_modules():
    yield
    while _BUILT:
        _BUILT.pop().stop()


def test_goal_latch_fires_once_then_holds():
    latch = GoalLatch(tolerance=0.2)
    latch.set_goal((1.0, 0.0))
    assert not latch.arrive((0.0, 0.0))
    assert latch.arrive((0.95, 0.0))
    assert latch.reached
    assert not latch.arrive((0.95, 0.0))


def test_goal_latch_ignores_sub_tolerance_goal_moves():
    latch = GoalLatch(tolerance=0.2)
    latch.set_goal((1.0, 0.0))
    assert latch.arrive((1.0, 0.0))
    latch.set_goal((1.05, 0.0))  # replan grid snap, same goal
    assert latch.reached
    latch.set_goal((3.0, 0.0))  # a new task
    assert not latch.reached


def _follower(**config):
    follower = TrajectoryFollower(rpc_transport=MemoryRPC, **{"embodiment": GO2, **config})
    _BUILT.append(follower)
    return follower


def _base_at(z: float) -> PoseStamped:
    """The tf-resolved base pose; the ground sits emb.base_height under it."""
    return PoseStamped(
        header=Header(frame_id="odom", stamp=time_from_seconds(0.0)),
        pose=Pose(position=Point(x=0.0, y=0.0, z=z), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
    )


def _straight_path(end_x: float = 0.5) -> Path:
    first = _base_at(0.0)
    last = _base_at(0.0)
    last.pose.position.x = end_x
    return Path(header=Header(frame_id="odom", stamp=time_from_seconds(0.0)), poses=[first, last])


class _Heard(list):
    """The module logger does not propagate, so caplog cannot see it."""

    def warning(self, msg, **kw):
        self.append((msg, kw))

    def info(self, msg, **kw):
        pass


@pytest.fixture
def heard(monkeypatch):
    said = _Heard()
    monkeypatch.setattr("dimos.navigation.trajectory_follower.fancy.module.logger", said)
    return said


# --- the deadman and preemption (follower.rs is the twin)


def _driven(follower: TrajectoryFollower) -> list:
    out: list = []
    follower.nav_cmd_vel.subscribe(out.append)
    return out


def _reached(follower: TrajectoryFollower) -> list:
    out: list = []
    follower.goal_reached.subscribe(out.append)
    return out


def test_a_stale_path_zeroes_the_twist():
    follower = _follower()
    out = _driven(follower)
    path, pose = _straight_path(5.0), _base_at(0.01)
    follower._on_path(path)
    follower.step(pose, path, age=follower.config.max_path_age_s + 0.1)
    assert (out[-1].linear.x, out[-1].linear.y, out[-1].angular.z) == (0.0, 0.0, 0.0)
    # the boundary is exclusive: a path exactly at the limit drives
    follower.step(pose, path, age=follower.config.max_path_age_s)
    assert out[-1].linear.x > 0.0


def test_a_stale_path_outranks_an_arrival():
    follower = _follower()
    out, reached = _driven(follower), _reached(follower)
    path = _straight_path(0.5)
    follower._on_path(path)
    at_goal = _base_at(0.01)
    at_goal.pose.position.x = 0.5
    follower.step(at_goal, path, age=9.0)
    assert out[-1].linear.x == 0.0
    assert not reached
    # and the same tick against a fresh path is the arrival
    follower.step(at_goal, path, age=0.0)
    assert [m.data for m in reached] == [True]


class _Clock:
    t = 100.0

    def __call__(self) -> float:
        return self.t


def _on_tf(follower: TrajectoryFollower, frame: str = "odom") -> tuple[MultiTBuffer, _Clock]:
    """A tf buffer and a clock in the follower's hands, with one live edge on it."""
    tf, clock = MultiTBuffer(), _Clock()
    follower._pose_src = TfPose(tf, "base_link", follower.config.max_path_age_s, clock=clock)
    tf.receive_transform(
        TransformStamped(
            header=Header(frame_id=frame, stamp=time_from_seconds(5.0)),
            child_frame_id="base_link",
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.01), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        )
    )
    return tf, clock


def test_the_pose_comes_off_tf_in_the_path_frame(heard):
    follower = _follower()
    out = _driven(follower)
    _, clock = _on_tf(follower, frame="map")
    follower._on_path(_straight_path(5.0))  # an odom plan; the edge on tf is map -> base_link
    follower.tick()
    assert (out[-1].linear.x, out[-1].linear.y, out[-1].angular.z) == (0.0, 0.0, 0.0)
    follower._on_path(
        Path(
            header=Header(frame_id="map", stamp=time_from_seconds(0.0)),
            poses=_straight_path(5.0).poses,
        )
    )
    clock.t += TfPose.RETRY_PERIOD_S  # the miss parked lookups for a period
    follower.tick()
    assert out[-1].linear.x > 0.0


def test_a_stale_pose_zeroes_the_twist(heard):
    follower = _follower()
    out = _driven(follower)
    _, clock = _on_tf(follower)
    follower._on_path(_straight_path(5.0))
    follower.tick()
    assert out[-1].linear.x > 0.0
    clock.t += follower.config.max_path_age_s + 0.1
    follower.tick()
    assert (out[-1].linear.x, out[-1].linear.y, out[-1].angular.z) == (0.0, 0.0, 0.0)


def test_an_empty_path_drops_the_plan_and_zeroes_the_twist():
    follower = _follower()
    out = _driven(follower)
    path, pose = _straight_path(5.0), _base_at(0.01)
    follower._on_path(path)
    follower.step(pose, path, age=0.0)
    assert out[-1].linear.x > 0.0
    follower._on_path(Path(header=path.header, poses=[]))
    assert (out[-1].linear.x, out[-1].linear.y, out[-1].angular.z) == (0.0, 0.0, 0.0)
    assert follower._path is None


def test_the_controller_override_names_a_law():
    follower = _follower(controller=seed.make)
    assert type(follower._controller).__name__ == "PursuitController"


def test_native_twin_shares_the_python_defaults():
    native, py = TrajectoryFollowerNativeConfig.model_fields, TrajectoryFollowerConfig.model_fields
    shared = set(native) & set(py) - set(ModuleConfig.model_fields)
    assert {f: native[f].default for f in shared} == {f: py[f].default for f in shared}
