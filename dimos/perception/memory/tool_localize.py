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

"""Query memory for objects, localize them in 3D via depth, render in rerun.

Run: uv run python -m dimos.perception.memory.tool_localize [query ...] [out.rrd]
         [--from <s>] [--duration <s>] [--multi]

Queries share one model load and one .rrd; with --multi they go to
localize() as one list, sharing a single detection pass per frame.
Exit code 0 with a printed
position per verified hit; exit code 1 when no query is verified, with
"no verified detection of ..." per miss - the honest answer that the object
is not there. An ambiguous hit (identical twins in view) is printed with its
ambiguity margin flagged.
"""

import argparse
from pathlib import Path
import sys
from typing import Any, cast

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.memory.transform import throttle
from dimos.perception.memory import gates
from dimos.perception.memory.localize import LocalizeTrace
from dimos.perception.memory.types import Localization
from dimos.utils.data import get_data

REFUSAL_MARGIN = 0.15


def render(
    out: str, store: Any, traces: list[tuple[str, LocalizeTrace]], t0: float, t1: float
) -> None:
    """Write the .rrd - rerun stays an inline import.

    Entity contract (the acceptance color cheat sheet): ``map`` backdrop,
    then one subtree per query - ``detections/<query>/matched/*`` green,
    ``detections/<query>/verified/*`` red, ``detections/<query>/answer``
    always blue.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.visualization.rerun.init import rerun_init

    tf = cast("StreamTF", StreamTF.from_store(store))
    camera_info = store.streams.camera_info.first().data

    rerun_init("memory-localize")
    rr.save(out)
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="/", name="Scene"),
                rrb.Spatial2DView(origin="camera", name="Live"),
                column_shares=[2, 1],
            )
        )
    )

    GREEN, RED, BLUE = [46, 204, 113], [231, 76, 60], [52, 120, 246]
    POINT_SIZE = 0.005

    def at(ts: float) -> None:
        rr.set_time("ts", timestamp=ts)

    # scene backdrop from an answer frame's depth (or the first detection)
    backdrop_ts = next((t.backdrop_ts for _, t in traces if t.backdrop_ts is not None), None)
    if backdrop_ts is None:
        backdrop_ts = next((t.matched[0][0] for _, t in traces if t.matched), None)
    if backdrop_ts is not None:
        try:
            color = store.streams.color_image.at(backdrop_ts, 0.1).first().data
            depth = gates.depth_at(store, backdrop_ts)
            transform = tf.get(
                gates.OPTICAL_FRAME, gates.WORLD_FRAME, backdrop_ts, gates.TF_TOLERANCE
            )
            if depth is not None and transform is not None:
                backdrop = PointCloud2.from_rgbd(
                    color, depth, camera_info, depth_scale=0.001
                ).transform(-transform)
                rr.log(
                    "map",
                    backdrop.voxel_downsample(0.01).to_rerun(voxel_size=POINT_SIZE),
                    static=True,
                )
        except LookupError:
            pass

    # live camera feed + frustum tracking the wrist along the timeline
    rr.log("camera", camera_info.to_rerun(), static=True)
    feed_throttle = 0.1 if (t1 - t0) <= 160 else 0.4
    feed = store.streams.color_image.after(t0).before(t1).transform(throttle(feed_throttle))
    for obs in feed:
        pose = gates.camera_pose(tf, obs.ts)
        if pose is None:
            continue
        at(obs.ts)
        rr.log("camera/image", obs.data.to_rerun())
        rr.log("camera", pose.to_rerun())

    for query, trace in traces:
        root = f"detections/{query.replace(' ', '_')}"

        # marked frames: into the live feed, plus a frozen frustum at the capture pose
        for i, obs in enumerate(trace.detection_frames):
            pose = gates.camera_pose(tf, obs.ts)
            if pose is None:
                continue
            at(obs.ts)
            annotated = obs.data.annotated_image()
            rr.log("camera/image", annotated.to_rerun())
            frame = f"{root}/frames/{i}"
            rr.log(frame, pose.to_rerun())
            rr.log(frame, camera_info.to_rerun())
            rr.log(f"{frame}/image", annotated.to_rerun())

        # 3d detections: green = matched candidates, red = cross-view re-detections
        for tag, entries, rgb in [
            ("matched", trace.matched, GREEN),
            ("verified", trace.verified, RED),
        ]:
            for i, (ts, det) in enumerate(entries):
                at(ts)
                rr.log(
                    f"{root}/{tag}/{i}_{det.name.replace(' ', '_')}",
                    det.pointcloud.to_rerun(voxel_size=POINT_SIZE, colors=rgb),
                )

        # the answer: always blue, whatever the query
        if trace.answer is not None:
            at(trace.answer.ts)
            rr.log(
                f"{root}/answer",
                trace.answer.pointcloud.to_rerun(voxel_size=POINT_SIZE, colors=BLUE),
            )


def report(query: str, hit: Localization | None, lo: float) -> bool:
    """Print one query's outcome; True when it counts as a hit."""
    if hit is None:
        print(f"no verified detection of {query!r}")
        return False
    offset = hit.pose_timestamp - lo
    if hit.position_world_xyz is None:
        print(
            f"hit {query!r} without pose: reason={hit.reason} "
            f"score={hit.semantic_score:.2f} ts_offset={offset:.1f}s"
        )
        return True
    x, y, z = hit.position_world_xyz
    cloud_points = len(hit.point_cloud) if hit.point_cloud is not None else 0
    print(
        f"hit {query!r}: position=({x:.3f}, {y:.3f}, {z:.3f}) frame={hit.frame_id} "
        f"ts_offset={offset:.1f}s points={cloud_points} views={hit.n_views} "
        f"score={hit.semantic_score:.2f} margin={hit.ambiguity_margin:.2f}"
    )
    if hit.ambiguity_margin < REFUSAL_MARGIN:
        print(
            f"ambiguity: margin {hit.ambiguity_margin:.2f} below refusal threshold "
            f"{REFUSAL_MARGIN:.2f} - multiple coexisting matches, this pick is flagged"
        )
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "positionals",
        nargs="+",
        metavar="query",
        help="one or more object queries, optionally followed by an out.rrd to write",
    )
    parser.add_argument("--dataset", type=Path, help="memory recording database")
    parser.add_argument(
        "--from", dest="start", type=float, default=0.0, help="start offset into the recording (s)"
    )
    parser.add_argument("--duration", type=float, default=None, help="how much to parse (s)")
    parser.add_argument(
        "--allow-no-pose",
        action="store_true",
        help="return an RGB-only hit with a null position instead of refusing",
    )
    parser.add_argument(
        "--multi",
        action="store_true",
        help="pass all queries to localize() as one list (one shared detection pass per frame)",
    )
    args = parser.parse_args()

    queries = list(args.positionals)
    out = queries.pop() if queries[-1].endswith(".rrd") else None
    if not queries:
        parser.error("no queries given")

    dataset = args.dataset or get_data(
        "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
        "xarm6_worldbelief_20260729_203624_161992.db"
    )
    store = SqliteStore(path=dataset)
    lo, hi = store.streams.color_image.get_time_range()
    after = lo + args.start
    before = lo + args.start + args.duration if args.duration is not None else hi

    from dimos.perception.memory.dandetect import DanDetector

    traces: list[tuple[str, LocalizeTrace]] = []
    hits = 0
    with DanDetector() as dan:
        index = dan.embed(store, after, before)
        if args.multi:
            qtraces = [LocalizeTrace() for _ in queries]
            results = dan.localize(
                store,
                queries,
                index=index,
                require_pose=not args.allow_no_pose,
                trace=qtraces,
            )
            for query, qtrace, hit in zip(queries, qtraces, results, strict=True):
                if report(query, hit, lo):
                    hits += 1
                    traces.append((query, qtrace))
        else:
            for query in queries:
                trace = LocalizeTrace()
                hit = dan.localize(
                    store,
                    query,
                    index=index,
                    require_pose=not args.allow_no_pose,
                    trace=trace,
                )
                if report(query, hit, lo):
                    hits += 1
                    traces.append((query, trace))

    if not hits:
        return 1

    if out is not None:
        render(out, store, traces, after, before)
        print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
