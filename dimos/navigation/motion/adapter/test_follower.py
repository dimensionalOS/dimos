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


import numpy as np
import pytest

from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.motion.adapter.follower import (
    GoalLatch,
    TrajectoryFollower,
    TrajectoryFollowerConfig,
)
from dimos.navigation.motion.adapter.follower_native import TrajectoryFollowerNativeConfig
from dimos.navigation.motion.control.laws import seed
from dimos.navigation.motion.obstacles import path_clearance
from dimos.navigation.tf_pose import TfPose
from dimos.protocol.tf.tf import MultiTBuffer

# Followers built by the helper below. The real constructor stands up the module's
# LCM RPC transport (a run_forever + _lcm_loop daemon pair per instance); these
# tests exercise pure geometry on top of it, so the fixture hands them back.
_BUILT: list[TrajectoryFollower] = []


@pytest.fixture(autouse=True)
def _stop_modules():
    yield
    while _BUILT:
        _BUILT.pop().stop()


def test_clearance_is_obstacle_distance_minus_half_width():
    xy = np.array([[0.0, 0.0]])
    points = np.array([[1.0, 0.0, 0.2]])
    clr = path_clearance(xy, points, half_width=0.155)
    assert abs(clr[0] - (1.0 - 0.155)) < 1e-6


def test_clearance_reads_every_point_it_is_handed_whatever_its_z():
    # the model already decided; a floor or ceiling z arriving here means the
    # model KEPT it, and re-judging it would price a world nobody planned
    xy = np.array([[0.0, 0.0]])
    for z in (-0.5, 0.01, 0.2, 0.46, 1.0):
        points = np.array([[0.1, 0.0, z]])
        assert abs(path_clearance(xy, points, half_width=0.0)[0] - 0.1) < 1e-6


def test_clearance_empty_path_or_no_obstacles():
    assert path_clearance(np.zeros((0, 2)), np.zeros((0, 3)), 0.1).shape == (0,)
    assert np.isinf(path_clearance(np.zeros((1, 2)), np.zeros((0, 3)), 0.1)[0])


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


# --- the module shell


def _follower(**config):
    follower = TrajectoryFollower(**config)
    _BUILT.append(follower)
    return follower


def _base_at(z: float) -> PoseStamped:
    """The tf-resolved base pose; the ground sits emb.base_height under it."""
    return PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.0, 0.0, z))


def _straight_path(end_x: float = 0.5) -> Path:
    return Path(
        ts=0.0,
        frame_id="odom",
        poses=[
            PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.0, 0.0, 0.0)),
            PoseStamped(ts=0.0, frame_id="odom", position=Vector3(end_x, 0.0, 0.0)),
        ],
    )


class _Heard(list):
    """The module logger does not propagate, so caplog cannot see it."""

    def warning(self, msg, **kw):
        self.append((msg, kw))

    def info(self, msg, **kw):
        pass


@pytest.fixture
def heard(monkeypatch):
    said = _Heard()
    monkeypatch.setattr("dimos.navigation.motion.adapter.follower.logger", said)
    monkeypatch.setattr("dimos.navigation.motion.adapter.diagnostics.logger", said)
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
    at_goal = PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.5, 0.0, 0.01))
    follower.step(at_goal, path, age=9.0)
    assert out[-1].linear.x == 0.0
    assert not reached
    # ...and the same tick against a fresh path IS the arrival
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
        Transform(
            translation=Vector3(0.0, 0.0, 0.01),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=frame,
            child_frame_id="base_link",
            ts=5.0,
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
    assert any("pose" in str(m) for m in heard)
    follower._on_path(Path(ts=0.0, frame_id="map", poses=_straight_path(5.0).poses))
    clock.t += TfPose.RETRY_PERIOD_S  # the miss parked lookups for a period
    follower.tick()
    assert out[-1].linear.x > 0.0


def test_a_stale_pose_zeroes_the_twist_and_names_itself(heard):
    follower = _follower()
    out = _driven(follower)
    _, clock = _on_tf(follower)
    follower._on_path(_straight_path(5.0))
    follower.tick()
    assert out[-1].linear.x > 0.0
    clock.t += follower.config.max_path_age_s + 0.1
    follower.tick()
    assert (out[-1].linear.x, out[-1].linear.y, out[-1].angular.z) == (0.0, 0.0, 0.0)
    assert any("pose" in str(m) for m in heard)


def test_stop_movement_drops_the_plan_and_zeroes_the_twist():
    follower = _follower()
    out = _driven(follower)
    path, pose = _straight_path(5.0), _base_at(0.01)
    follower._on_path(path)
    follower.step(pose, path, age=0.0)
    assert out[-1].linear.x > 0.0
    follower._on_stop(Bool(True))
    assert (out[-1].linear.x, out[-1].linear.y, out[-1].angular.z) == (0.0, 0.0, 0.0)
    assert follower._path is None


def test_the_controller_override_names_a_law():
    follower = _follower(controller=seed.make)
    assert type(follower._controller).__name__ == "PursuitController"


def test_native_twin_shares_the_python_defaults():
    native, py = TrajectoryFollowerNativeConfig.model_fields, TrajectoryFollowerConfig.model_fields
    shared = set(native) & set(py) - set(ModuleConfig.model_fields)
    assert {f: native[f].default for f in shared} == {f: py[f].default for f in shared}
