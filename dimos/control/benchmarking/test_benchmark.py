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

"""Benchmark pub/sub pieces: odom recorder, odom-based completion, the flat
per-run recording round-trip, and an end-to-end in-process loop (RPP follower
driven against the FOPDT sim plant -> benchmark records odom -> completion fires
-> recording -> offline scoring)."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from dimos.control.benchmarking.benchmark import (
    CompletionMonitor,
    OdomRecorder,
    RunRecording,
    path_set,
    shift_path_to_start_at_pose,
)
from dimos.control.benchmarking.paths import circle, straight_line
from dimos.control.benchmarking.plant import (
    FopdtChannelParams,
    TwistBasePlantParams,
    TwistBasePlantSim,
)
from dimos.control.benchmarking.score import score_dir
from dimos.control.benchmarking.tuning import TuningConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.rpp_path_follower_task.rpp_path_follower_task import (
    DEFAULT_ARTIFACT_PATH,
    create_task,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

_JOINTS = ["go2/vx", "go2/vy", "go2/wz"]


def _pose(x=0.0, y=0.0, yaw=0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0), orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
    )


# ---------------------------------------------------------------------------
# Recording round-trip
# ---------------------------------------------------------------------------


def test_recording_round_trip(tmp_path):
    ref = straight_line(length=2.0)
    rec = RunRecording.from_path(
        robot="go2",
        path_name="straight_line",
        speed=0.5,
        reference=ref,
        goal_tolerance=0.25,
        velocity_threshold=0.05,
        timeout=60.0,
    )
    rec.arrived = True
    rec.reason = "goal+stop"
    rec.ticks = [[i * 0.1, i * 0.1, 0.0, 0.0, 0.5, 0.0, 0.0] for i in range(20)]
    out = tmp_path / "run.json"
    rec.to_json(out)

    back = RunRecording.from_json(out)
    assert back.robot == "go2"
    assert back.path == "straight_line"
    assert back.speed == 0.5
    assert back.arrived is True
    assert back.reason == "goal+stop"
    assert len(back.ticks) == 20
    # reference rebuilds into a Path matching the original pose count
    assert len(back.reference_path().poses) == len(ref.poses)


def test_path_set_is_the_full_battery():
    names = set(path_set())
    assert names == {
        "straight_line",
        "single_corner",
        "smooth_corner",
        "square",
        "rounded_square",
        "circle",
    }


def test_anchor_shifts_path_to_pose():
    ref = straight_line(length=2.0)
    anchored = shift_path_to_start_at_pose(ref, _pose(5.0, 3.0, 0.0))
    p0 = anchored.poses[0].position
    assert p0.x == 5.0 and p0.y == 3.0


# ---------------------------------------------------------------------------
# Odom recorder
# ---------------------------------------------------------------------------


def test_odom_recorder_differentiates_forward_velocity():
    rec = OdomRecorder(alpha=1.0)  # no smoothing -> exact diff
    rec.on_odom(_pose(0.0, 0.0, 0.0), now=0.0)
    rec.on_odom(_pose(0.1, 0.0, 0.0), now=0.1)  # 1 m/s forward
    lin, ang = rec.body_speed()
    assert lin == pytest.approx(1.0, abs=1e-6)
    assert ang == pytest.approx(0.0, abs=1e-6)
    assert len(rec.snapshot()) == 2


def test_odom_recorder_reset_clears_ticks():
    rec = OdomRecorder()
    rec.on_odom(_pose(0.0), now=0.0)
    rec.on_odom(_pose(0.1), now=0.1)
    assert rec.snapshot()
    rec.reset()
    assert rec.snapshot() == []
    assert rec.latest_pose() is not None  # latest pose is retained for warm-up


# ---------------------------------------------------------------------------
# Completion monitor (odom-only)
# ---------------------------------------------------------------------------


def _monitor(ref, dwell_s=0.3):
    return CompletionMonitor(
        ref,
        goal_tolerance=0.25,
        velocity_threshold=0.05,
        angular_threshold=0.1,
        dwell_s=dwell_s,
    )


def test_completion_fires_on_goal_and_stop():
    ref = straight_line(length=2.0)
    mon = _monitor(ref)
    t = 0.0
    done = False
    # drive to the goal (moving)
    for i in range(40):
        x = min(2.0, i * 0.1)
        done = mon.update(x, 0.0, 1.0, 0.0, t)  # moving -> not done
        t += 0.1
    assert not done
    # sit at goal, stopped, for the dwell
    for _ in range(10):
        done = mon.update(2.0, 0.0, 0.0, 0.0, t)
        t += 0.1
        if done:
            break
    assert done


def test_completion_does_not_fire_prematurely_on_closed_path():
    """A closed path (circle: goal == start) must not trip arrival at the start
    just because the robot is near the last pose and momentarily slow."""
    ref = circle(radius=1.0)
    mon = _monitor(ref)
    # robot sits at the start (== goal) and is stopped, but hasn't covered the
    # path yet -> not complete.
    done = False
    for i in range(10):
        done = mon.update(ref.poses[0].position.x, ref.poses[0].position.y, 0.0, 0.0, i * 0.1)
    assert not done


def test_completion_never_fires_when_robot_never_reaches_goal():
    ref = straight_line(length=2.0)
    mon = _monitor(ref)
    done = False
    for i in range(60):  # robot stuck at start, stopped
        done = mon.update(0.0, 0.0, 0.0, 0.0, i * 0.1)
    assert not done  # never covered the path -> would hit the per-run timeout


# ---------------------------------------------------------------------------
# End-to-end: RPP follower vs sim plant -> benchmark record -> score
# ---------------------------------------------------------------------------


def _state(x, y, yaw, t):
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={_JOINTS[0]: x, _JOINTS[1]: y, _JOINTS[2]: yaw},
            joint_velocities={_JOINTS[0]: 0.0, _JOINTS[1]: 0.0, _JOINTS[2]: 0.0},
        ),
        t_now=t,
        dt=0.1,
    )


def test_end_to_end_controller_benchmark_scoring(tmp_path):
    """Prove the full decoupled chain in-process against the in-repo FOPDT sim:
    the benchmark publishes a path+speed (here driven directly), the RPP follower
    tracks it against TwistBasePlantSim, the benchmark records the executed odom,
    odom-based completion fires, a run is written, and the offline scorer emits
    metrics."""
    art = TuningConfig.from_json(DEFAULT_ARTIFACT_PATH)
    task = create_task(
        SimpleNamespace(
            name="rpp_follower", joint_names=_JOINTS, priority=10, params={"speed": 0.5}
        ),
        None,
    )
    plant = TwistBasePlantSim(
        TwistBasePlantParams(
            vx=FopdtChannelParams(art.plant.vx.K, art.plant.vx.tau, art.plant.vx.L),
            vy=FopdtChannelParams(art.plant.vy.K, art.plant.vy.tau, art.plant.vy.L),
            wz=FopdtChannelParams(art.plant.wz.K, art.plant.wz.tau, art.plant.wz.L),
        )
    )
    plant.reset(0.0, 0.0, 0.0, 0.1)

    # Benchmark anchors the path to the robot's first odom (here the origin).
    ref = shift_path_to_start_at_pose(straight_line(length=2.0), _pose(plant.x, plant.y, plant.yaw))
    task.set_path(ref, _pose(plant.x, plant.y, plant.yaw))

    recorder = OdomRecorder()
    monitor = _monitor(ref, dwell_s=0.3)
    done = False
    for k in range(800):
        out = task.compute(_state(plant.x, plant.y, plant.yaw, t=k * 0.1))
        cvx, cvy, cwz = out.velocities if out is not None else (0.0, 0.0, 0.0)
        # Benchmark records what it sees over the transport: cmd_vel + odom.
        recorder.on_cmd_vel(Twist(linear=Vector3(cvx, cvy, 0.0), angular=Vector3(0.0, 0.0, cwz)))
        recorder.on_odom(_pose(plant.x, plant.y, plant.yaw), now=k * 0.1)
        lin, ang = recorder.body_speed()
        if monitor.update(plant.x, plant.y, lin, ang, k * 0.1):
            done = True
            break
        plant.step(cvx, cvy, cwz, 0.1)
    assert done, "odom-based completion never fired"

    # Write the run, then score it offline.
    rec = RunRecording.from_path(
        robot="go2",
        path_name="straight_line",
        speed=0.5,
        reference=ref,
        goal_tolerance=0.25,
        velocity_threshold=0.05,
        timeout=60.0,
    )
    rec.arrived = True
    rec.reason = "goal+stop"
    rec.ticks = recorder.snapshot()
    rec.to_json(tmp_path / "go2_straight_line_v0.50_000.json")

    opm = score_dir(tmp_path, tolerances_cm=[10, 15], plots=False)
    assert len(opm.points) == 1
    pt = opm.points[0]
    assert pt.arrived
    assert pt.path == "straight_line"
    # tracked a straight line reasonably well
    assert pt.cte_max < 0.3
    assert len(opm.tolerance_inversion) == 2
