# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for the RPP nav follower: lazy artifact load, set_path arming, the
feedforward/profile calibration, replan reset, and an end-to-end drive."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from dimos.control.benchmarking.paths import circle, straight_line
from dimos.control.benchmarking.plant import (
    FopdtChannelParams,
    TwistBasePlantParams,
    TwistBasePlantSim,
)
from dimos.control.benchmarking.tuning import (
    FeedforwardDC,
    FopdtChannelDC,
    PlantModelDC,
    Provenance,
    RecommendedControllerDC,
    TuningConfig,
    VelocityProfileDC,
)
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.rpp_path_follower_task.rpp_path_follower_task import (
    RPPPathFollowerTask,
    create_task,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

_JOINTS = ["go2/vx", "go2/vy", "go2/wz"]


@pytest.fixture
def artifact_path(tmp_path):
    """A small on-disk artifact (plant gain != 1, real lag) for the task to load."""
    chan = lambda K: FopdtChannelDC(K=K, tau=0.3, L=0.1)  # noqa: E731
    art = TuningConfig(
        provenance=Provenance(robot_id="go2", git_sha="test", sim_or_hw="hw"),
        plant=PlantModelDC(vx=chan(0.8), vy=chan(0.8), wz=chan(0.9)),
        feedforward=FeedforwardDC(K_vx=1.25, K_vy=1.25, K_wz=1.11),  # 1/K (unused by RPP)
        velocity_profile=VelocityProfileDC(
            max_linear_speed=0.8,
            max_angular_speed=1.18,
            max_centripetal_accel=1.0,
            max_linear_accel=2.5,
            max_linear_decel=5.0,
            min_speed=0.2,
        ),
        recommended_controller=RecommendedControllerDC(),
    )
    p = tmp_path / "art.json"
    art.to_json(p)
    return str(p), art


def _task(artifact_path: str, **params) -> RPPPathFollowerTask:
    cfg = SimpleNamespace(
        name="rpp_follower",
        joint_names=_JOINTS,
        priority=10,
        params={"artifact_path": artifact_path, **params},
    )
    return create_task(cfg, None)


def _odom(x=0.0, y=0.0, yaw=0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0), orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
    )


def test_artifact_loaded_lazily_not_at_construction(artifact_path):
    path, _ = artifact_path
    task = _task(path)
    assert not task._artifact_loaded
    assert task._ff is None and task._profile_cap is None


def test_set_path_arms_and_loads_calibration(artifact_path):
    path, art = artifact_path
    task = _task(path, speed=0.7)
    task.set_path(circle(radius=1.0), _odom())
    assert task._artifact_loaded
    assert task.is_active()
    assert task.get_state() == "path_following"
    # Feedforward built from plant.K (commanded == achieved), not the 1/K block.
    assert task._ff.cfg.K_vx == art.plant.vx.K
    assert task._ff.cfg.K_wz == art.plant.wz.K
    # Curvature profile + yaw clamp from the artifact.
    assert task._profile_cap.cfg.max_centripetal_accel == art.velocity_profile.max_centripetal_accel
    assert task._profile_cap.cfg.min_speed == art.velocity_profile.min_speed
    assert task._config.max_yaw_rate == art.velocity_profile.max_angular_speed


def test_profile_speed_capped_to_cruise(artifact_path):
    path, _ = artifact_path
    # cruise 0.5 below the artifact's 0.8 max -> profile uses the cruise.
    task = _task(path, speed=0.5)
    task.set_path(straight_line(length=3.0), _odom())
    assert task._profile_cap.cfg.max_linear_speed == pytest.approx(0.5)


def test_set_speed_rebuilds_cap_after_load(artifact_path):
    """The coordinator's `speed` port -> set_speed() retunes the follower: it
    updates the pursuit speed and rebuilds the curvature cap to the new top
    speed (still using the artifact's a_lat / min_speed). Valid only while idle,
    so the run is cancelled before retuning (as between benchmark runs)."""
    path, art = artifact_path
    task = _task(path, speed=0.7)
    task.set_path(straight_line(length=3.0), _odom())  # loads the artifact + arms
    task.cancel()  # back to idle so a new speed can be set (as between runs)
    task.set_speed(0.4)
    assert task._config.speed == pytest.approx(0.4)
    assert task._controller._speed == pytest.approx(0.4)
    assert task._profile_cap.cfg.max_linear_speed == pytest.approx(0.4)
    # regulation constants stay from the artifact
    assert task._profile_cap.cfg.max_centripetal_accel == art.velocity_profile.max_centripetal_accel
    assert task._profile_cap.cfg.min_speed == art.velocity_profile.min_speed


def test_set_speed_clamps_to_artifact_vmax(artifact_path):
    """set_speed above the artifact's measured top speed clamps to it — the cap
    never exceeds what the plant can hold."""
    path, art = artifact_path  # artifact max_linear_speed = 0.8
    task = _task(path, speed=0.5)
    task.set_path(straight_line(length=3.0), _odom())
    task.cancel()
    task.set_speed(2.0)
    assert task._profile_cap.cfg.max_linear_speed == pytest.approx(
        art.velocity_profile.max_linear_speed
    )


def test_set_speed_before_first_path_is_picked_up(artifact_path):
    """set_speed before any path (artifact not yet loaded) stashes the speed; the
    lazy artifact load on the first path picks it up."""
    path, _ = artifact_path
    task = _task(path, speed=0.7)
    assert not task._artifact_loaded
    task.set_speed(0.4)
    assert task._profile_cap is None  # nothing to rebuild yet
    task.set_path(straight_line(length=3.0), _odom())
    assert task._profile_cap.cfg.max_linear_speed == pytest.approx(0.4)


def test_set_speed_ignored_while_active(artifact_path):
    """A mid-run set_speed is ignored (a discontinuous cap jump); the next path
    picks up the new speed cleanly."""
    path, _ = artifact_path
    task = _task(path, speed=0.7)
    task.set_path(straight_line(length=3.0), _odom())
    assert task.is_active()
    task.set_speed(0.3)
    assert task._config.speed == pytest.approx(0.7)  # unchanged while active


def test_set_path_resets_cleanly_on_replan(artifact_path):
    path, _ = artifact_path
    task = _task(path)
    task.set_path(straight_line(length=3.0), _odom())
    # Drive a couple ticks so progress advances.
    for k in range(5):
        task.compute(_state(0.5 * k, 0.0, 0.0))
    advanced = task._max_progress_idx
    assert advanced > 0
    # A replan (new path via set_path) resets progress to 0 — no stale carrot.
    task.set_path(straight_line(length=3.0), _odom())
    assert task._max_progress_idx == 0
    assert task.get_state() == "path_following"


def _state(x, y, yaw, t=0.0):
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={_JOINTS[0]: x, _JOINTS[1]: y, _JOINTS[2]: yaw},
            joint_velocities={_JOINTS[0]: 0.0, _JOINTS[1]: 0.0, _JOINTS[2]: 0.0},
        ),
        t_now=t,
        dt=0.1,
    )


def test_drives_to_arrival_forward_only(artifact_path):
    """End-to-end: drive a straight against the FOPDT plant; arrives, and the
    forward-only contract holds (vy == 0, vx >= 0) every tick."""
    path, art = artifact_path
    from dimos.core.global_config import global_config as gc  # noqa: F401 (task uses _gc)

    task = _task(path, speed=0.7)
    plant = TwistBasePlantSim(
        TwistBasePlantParams(
            vx=FopdtChannelParams(art.plant.vx.K, art.plant.vx.tau, art.plant.vx.L),
            vy=FopdtChannelParams(art.plant.vy.K, art.plant.vy.tau, art.plant.vy.L),
            wz=FopdtChannelParams(art.plant.wz.K, art.plant.wz.tau, art.plant.wz.L),
        )
    )
    p = straight_line(length=3.0)
    plant.reset(0.0, 0.0, 0.0, 0.1)
    task.set_path(p, _odom())
    arrived = False
    for k in range(400):
        out = task.compute(_state(plant.x, plant.y, plant.yaw, t=k * 0.1))
        cvx, cvy, cwz = out.velocities if out is not None else (0.0, 0.0, 0.0)
        assert cvy == 0.0
        assert cvx >= 0.0
        if task.get_state() == "arrived":
            arrived = True
            break
        plant.step(cvx, cvy, cwz, 0.1)
    assert arrived


# --- tangent-heading synthesis (for position-only planners e.g. MLS) -----------

import math

from dimos.control.tasks.rpp_path_follower_task.rpp_path_follower_task import (
    _with_tangent_headings,
)
from dimos.msgs.nav_msgs.Path import Path


def _ident_path(points):
    """Path with identity orientation on every pose (what MLSPlannerNative emits)."""
    return Path(
        ts=0.0,
        frame_id="odom",
        poses=[
            PoseStamped(
                ts=0.0, position=Vector3(x, y, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
            for x, y in points
        ],
    )


def test_degenerate_path_gets_tangent_headings():
    # Straight +x then turn +y: first segments face 0 rad, the corner faces +pi/2.
    out = _with_tangent_headings(_ident_path([(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]))
    yaws = [p.orientation.euler[2] for p in out.poses]
    assert math.isclose(yaws[0], 0.0, abs_tol=1e-6)
    assert math.isclose(yaws[1], 0.0, abs_tol=1e-6)
    assert math.isclose(yaws[2], math.pi / 2, abs_tol=1e-6)  # tangent into the turn
    assert math.isclose(yaws[3], math.pi / 2, abs_tol=1e-6)
    # Last pose inherits the approach heading (no next segment).
    assert math.isclose(yaws[4], yaws[3], abs_tol=1e-6)


def test_positions_unchanged_by_synthesis():
    pts = [(0, 0), (1, 0), (1, 1)]
    out = _with_tangent_headings(_ident_path(pts))
    assert [(round(p.position.x, 6), round(p.position.y, 6)) for p in out.poses] == pts


def test_path_with_real_headings_is_untouched():
    # A planner that already provides per-pose orientations must be left alone.
    oriented = Path(
        ts=0.0,
        frame_id="odom",
        poses=[
            PoseStamped(
                ts=0.0,
                position=Vector3(0, 0, 0),
                orientation=Quaternion.from_euler(Vector3(0, 0, 0.3)),
            ),
            PoseStamped(
                ts=0.0,
                position=Vector3(1, 0, 0),
                orientation=Quaternion.from_euler(Vector3(0, 0, 1.4)),
            ),
        ],
    )
    out = _with_tangent_headings(oriented)
    assert out is oriented  # returned unchanged (identity), not rebuilt


def test_short_path_returned_as_is():
    one = _ident_path([(0, 0)])
    assert _with_tangent_headings(one) is one
