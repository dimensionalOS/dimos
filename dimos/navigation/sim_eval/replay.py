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

"""An episode's recording as a rerun file, with its case's scene, goal and score drawn in.

The runner writes one per episode next to the recording. Open it with ``rerun episode.rrd``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sim_msgs.Contacts import Contacts
from dimos.navigation.global_planner.viz import robot_body_box
from dimos.navigation.sim_eval.runner import RECORDING_FILE, RUN_FILE, SCORE_FILE
from dimos.navigation.sim_eval.suite import Case, Manifest
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_LENGTH, ROBOT_WIDTH
from dimos.simulation.go2_sim.world import ODOM_FRAME_ID, scene_edges

REPLAY_FILE = "episode.rrd"
TIMELINE = "time"
LOCAL_PATH_COLOR = (255, 160, 0)
TRAJECTORY_COLOR = (200, 200, 200)
GOAL_COLOR = (255, 60, 60)


def episode_case(episode_dir: Path) -> Case:
    """The case an episode directory ran, from its run's manifest."""
    run = json.loads((episode_dir.parents[2] / RUN_FILE).read_text())
    manifest = Manifest.load(Path(run["suite"]))
    return next(c for c in manifest.cases if c.id == episode_dir.parent.name)


def write_rrd(episode_dir: Path, out: Path | None = None) -> Path:
    """Write the episode's rerun file and return its path."""
    import rerun as rr

    case = episode_case(episode_dir)
    out = out or episode_dir / REPLAY_FILE
    rec = rr.RecordingStream(application_id="dimos-sim-eval", recording_id=case.id)
    rec.save(str(out))
    rec.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    scene = LineSegments3D(ts=0.0, frame_id=ODOM_FRAME_ID, segments=scene_edges(case.scene()))
    rec.log("world/scene", scene.to_rerun(radii=0.01), static=True)
    rec.log(
        "world/goal",
        rr.Points3D(positions=[case.goal], radii=0.1, colors=[GOAL_COLOR]),
        static=True,
    )
    rec.log(
        "world/robot/body",
        robot_body_box(ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT),
        static=True,
    )
    store = SqliteStore(path=str(episode_dir / RECORDING_FILE))
    store.start()
    try:
        names = set(store.list_streams())
        trajectory = []
        if "ground_truth" in names:
            for pose in store.stream("ground_truth", PoseStamped).to_list():
                trajectory.append(tuple(pose.data.position))
                rec.set_time(TIMELINE, timestamp=float(pose.ts))
                rec.log("world/robot", pose.data.to_rerun())
        if trajectory:
            rec.log(
                "world/trajectory",
                rr.LineStrips3D([trajectory], colors=[TRAJECTORY_COLOR], radii=0.01),
                static=True,
            )
        for name, color in (("planner_path", (0, 255, 128)), ("path", LOCAL_PATH_COLOR)):
            if name in names:
                for path in store.stream(name, PathMsg).to_list():
                    rec.set_time(TIMELINE, timestamp=float(path.ts))
                    rec.log(f"world/{name}", path.data.to_rerun(color=color, z_offset=0.05))
        if "goal" in names:
            for goal in store.stream("goal", PointStamped).to_list():
                if math.isfinite(goal.data.x):
                    rec.set_time(TIMELINE, timestamp=float(goal.ts))
                    rec.log("world/goal_sent", goal.data.to_rerun())
        if "cmd_vel" in names:
            for command in store.stream("cmd_vel", Twist).to_list():
                rec.set_time(TIMELINE, timestamp=float(command.ts))
                rec.log("cmd_vel/vx", rr.Scalars(command.data.linear.x))
                rec.log("cmd_vel/wz", rr.Scalars(command.data.angular.z))
        if "contacts" in names:
            for contacts in store.stream("contacts", Contacts).to_list():
                rec.set_time(TIMELINE, timestamp=float(contacts.ts))
                text = ", ".join(f"{c.part} on {c.kind}" for c in contacts.data.contacts) or "none"
                rec.log("contacts", rr.TextLog(text))
    finally:
        store.stop()
    _log_score(rec, episode_dir)
    rec.flush()
    return out


def _log_score(rec: object, episode_dir: Path) -> None:
    import rerun as rr

    assert isinstance(rec, rr.RecordingStream)
    score_file = episode_dir / SCORE_FILE
    if not score_file.exists():
        return
    score = json.loads(score_file.read_text())
    t0, t1 = score["window"]
    rec.set_time(TIMELINE, timestamp=float(t0))
    rec.log("episode", rr.TextLog("goal echoed, episode clock starts"))
    rec.set_time(TIMELINE, timestamp=float(t1))
    metrics = {k: v for k, v in score.items() if isinstance(v, (int, float)) and k != "window"}
    summary = ", ".join(
        f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in metrics.items()
    )
    rec.log("episode", rr.TextLog(f"{score['outcome']}: {summary}"))
