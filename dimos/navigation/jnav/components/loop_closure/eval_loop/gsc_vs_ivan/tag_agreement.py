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

"""AprilTag agreement of a gsc_run run: every raw_april_tags sighting is put in the world through the odometry, then
through GSC's correction, and each tag's visits (sightings split by 20 s gaps, one median position per visit) should
land on one spot. spread = per tag, RMS distance of its visit positions to their centroid, mean over tags seen on 2+
visits (m, lower = tighter).
  final  = every visit through the finished graph
  live   = at each revisit, that visit and all earlier ones through the graph GSC had right then (does the robot's map
           agree with itself when it gets back somewhere), averaged over revisits
  belief = each visit through the correction GSC had at that moment (earlier visits keep their old, uncorrected
           belief, so this mixes map versions and reads worse than live) The camera mount is the recording's static tf from the odometry's frame to camera_optical.
usage: tag_agreement.py <run dir> <db> <odom stream> <odom frame> [--moving ID ...]   (run from the dimos worktree)
  -> prints and writes <run dir>/tags.json"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

from dimos.memory.store.sqlite import SqliteStore

VISIT_GAP_S = 20.0
parser = argparse.ArgumentParser()
parser.add_argument("run", type=Path)
parser.add_argument("db")
parser.add_argument("odom")
parser.add_argument("odom_frame")
parser.add_argument("--camera-frame", default="camera_optical")
parser.add_argument("--moving", type=int, nargs="*", default=[])
args = parser.parse_args()


def static_edges(store):
    """Every tf edge's first sample as (rotation, translation), parent -> child."""
    edges = {}
    for message in store.stream("tf").order_by("ts"):
        for transform in message.data.transforms:
            key = (transform.frame_id, transform.child_frame_id)
            if key not in edges:
                t, q = transform.translation, transform.rotation
                edges[key] = (R.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z]))
    return edges


def chain(edges, start, goal):
    """start -> goal through the tf graph (edges usable both ways), breadth first."""
    frontier, seen = [(start, R.identity(), np.zeros(3))], {start}
    while frontier:
        frame, rotation, translation = frontier.pop(0)
        if frame == goal:
            return rotation, translation
        for (parent, child), (edge_rotation, edge_translation) in edges.items():
            for here, there, step_rotation, step_translation in (
                (parent, child, edge_rotation, edge_translation),
                (
                    child,
                    parent,
                    edge_rotation.inv(),
                    -(edge_rotation.inv().apply(edge_translation)),
                ),
            ):
                if here == frame and there not in seen:
                    seen.add(there)
                    frontier.append(
                        (
                            there,
                            rotation * step_rotation,
                            rotation.apply(step_translation) + translation,
                        )
                    )
    raise SystemExit(f"no tf chain {start} -> {goal}")


with SqliteStore(path=args.db) as store:
    mount_rotation, mount_translation = chain(
        static_edges(store), args.odom_frame, args.camera_frame
    )
    odom = []
    for obs in store.stream(args.odom).order_by("ts"):
        p = (
            obs.data.pose
            if hasattr(obs.data, "pose") and hasattr(obs.data.pose, "position")
            else obs.data
        )
        odom.append(
            (
                obs.ts,
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            )
        )
    sightings = [
        (obs.ts, int(obs.tags["marker_id"]), obs.data)
        for obs in store.stream("raw_april_tags").order_by("ts")
    ]
odom = np.array(odom)
odom = odom[np.unique(odom[:, 0], return_index=True)[1]]
odom_slerp = Slerp(odom[:, 0], R.from_quat(odom[:, 4:]))

kf = np.fromfile(args.run / "a.kf", rm.KEYFRAME)
raw_rotation, corrected_rotation = (
    R.from_quat(kf["raw"][:, 3:]),
    R.from_quat(kf["corrected"][:, 3:]),
)
# final correction per keyframe (world <- world), interpolated between keyframes
final_rotation = corrected_rotation * raw_rotation.inv()
final_translation = kf["corrected"][:, :3] - final_rotation.apply(kf["raw"][:, :3])
final_slerp = Slerp(kf["ts"], final_rotation)
# every closure state's graph, as a correction (world <- world) per keyframe it holds
states = []
for ts, record in rm.read_closures(args.run / "a.kf.closures"):
    ids = record["id"].astype(int)
    delta = R.from_quat(record["dq"].astype(float))
    rotation = delta * R.from_quat(record["q"].astype(float)) * raw_rotation[ids].inv()
    translation = (
        delta.apply(record["xyz"].astype(float)) + record["dt"] - rotation.apply(kf["raw"][ids, :3])
    )
    order = np.argsort(kf["ts"][ids])
    states.append((ts, kf["ts"][ids][order], rotation[order], translation[order]))
state_times = np.array([state[0] for state in states])


def through(state, ts, xyz):
    """xyz (world, raw) at time ts through one closure state's graph: its nearest keyframe's correction."""
    _, times, rotation, translation = state
    i = min(int(np.searchsorted(times, ts)), len(times) - 1)
    if i > 0 and ts - times[i - 1] < times[i] - ts:
        i -= 1
    return rotation[i].apply(xyz) + translation[i]


positions = {"raw": {}, "final": {}, "belief": {}}
for ts, marker_id, pose in sightings:
    if marker_id in args.moving or not (
        odom[0, 0] <= ts <= odom[-1, 0] and kf["ts"][0] <= ts <= kf["ts"][-1]
    ):
        continue
    body_rotation = odom_slerp([ts])[0]
    body_translation = np.array([np.interp(ts, odom[:, 0], odom[:, k]) for k in (1, 2, 3)])
    camera_rotation = body_rotation * mount_rotation
    camera_translation = body_rotation.apply(mount_translation) + body_translation
    tag = (
        camera_rotation.apply([pose.position.x, pose.position.y, pose.position.z])
        + camera_translation
    )
    final = final_slerp([ts])[0].apply(tag) + np.array(
        [np.interp(ts, kf["ts"], final_translation[:, k]) for k in range(3)]
    )
    newest = np.searchsorted(state_times, ts) - 1
    belief = tag if newest < 0 else through(states[newest], ts, tag)
    for name, xyz in (("raw", tag), ("final", final), ("belief", belief)):
        positions[name].setdefault(marker_id, []).append((ts, xyz))


def spread(by_tag):
    per_tag = {}
    for marker_id, rows in by_tag.items():
        visits, last = [], None
        for ts, xyz in rows:
            if last is None or ts - last > VISIT_GAP_S:
                visits.append([])
            visits[-1].append(xyz)
            last = ts
        if len(visits) >= 2:
            centers = np.array([np.median(np.array(visit), axis=0) for visit in visits])
            per_tag[marker_id] = (
                len(visits),
                float(np.sqrt(np.mean(np.sum((centers - centers.mean(0)) ** 2, axis=1)))),
            )
    return per_tag


def split(rows):
    visits, last = [], None
    for ts, xyz in rows:
        if last is None or ts - last > VISIT_GAP_S:
            visits.append([])
        visits[-1].append((ts, xyz))
        last = ts
    return visits


def live_spread(by_tag):
    """Per tag: at each revisit, the spread of it and all earlier visits through the graph GSC had at its end."""
    per_tag = {}
    for marker_id, rows in by_tag.items():
        visits = split(rows)
        spreads = []
        for k in range(1, len(visits)):
            newest = np.searchsorted(state_times, visits[k][-1][0]) - 1
            if newest < 0:
                spreads.append(
                    spread({0: [row for visit in visits[: k + 1] for row in visit]})[0][1]
                )
                continue
            centers = np.array(
                [
                    np.median([through(states[newest], ts, xyz) for ts, xyz in visit], axis=0)
                    for visit in visits[: k + 1]
                ]
            )
            spreads.append(
                float(np.sqrt(np.mean(np.sum((centers - centers.mean(0)) ** 2, axis=1))))
            )
        if spreads:
            per_tag[marker_id] = (len(visits), float(np.mean(spreads)))
    return per_tag


out = {}
for name in ("raw", "final", "live", "belief"):
    per_tag = live_spread(positions["raw"]) if name == "live" else spread(positions[name])
    out[name] = dict(
        mean_spread_m=round(float(np.mean([s for _, s in per_tag.values()])), 3)
        if per_tag
        else None,
        tags={
            str(k): dict(visits=v, spread_m=round(s, 3)) for k, (v, s) in sorted(per_tag.items())
        },
    )
(args.run / "tags.json").write_text(json.dumps(out, indent=1))
print(
    args.run.name,
    json.dumps({k: v["mean_spread_m"] for k, v in out.items()}),
    f"tags revisited: {len(out['raw']['tags'])}",
)
