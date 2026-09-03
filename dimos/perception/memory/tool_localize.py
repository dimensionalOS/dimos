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

"""Query memory for objects, localize them in 3D, render in rerun.

Run: uv run python -m dimos.perception.memory.tool_localize [query ...] [out.rrd]
         [--dataset <db>] [--from <s>] [--duration <s>] [--multi]

The recording's shape decides the rig: an xArm-style store lifts through
aligned depth and tf, a mobile-robot store (Go2/G1 replay) lifts through
registered lidar and stamped poses. Queries share one model load and one
.rrd; with --multi they go to localize() as one list and share one
detection pass over the union of the labels' candidate frames.
Every query prints one
line per verified instance - all the coke bottles, not one winner - each
with the union cloud of every viewpoint that saw it. Exit code 0 when any
query verified; exit code 1 when none did, with "no verified detection
of ..." per miss - the honest answer that the object is not there. An
ambiguous instance (identical twins in view) is printed with its ambiguity
margin flagged.
"""

import argparse
import json
from pathlib import Path
import sys
from typing import Any

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.transform import throttle
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC, lattice_quantum
from dimos.perception.memory.localize import LocalizeTrace
from dimos.perception.memory.rig import Rig
from dimos.perception.memory.types import Localization
from dimos.utils.data import get_data

REFUSAL_MARGIN = 0.15


def render(
    out: str, rig: Rig, traces: list[tuple[str, LocalizeTrace]], t0: float, t1: float
) -> None:
    """Write the .rrd - rerun stays an inline import.

    Entity contract: the ``map`` backdrop is what was NOT detected - its
    cells are carved free of every answer's cells - and each
    ``detections/<query>/answer/<k>`` is one verified instance textured
    with each sighting's own image, wrapped in a labeled wireframe box.
    Mobile rigs show the robot as a translucent box carrying the live
    frustum; stationary rigs keep a frozen frustum per detection frame.
    """
    import numpy as np
    import rerun as rr
    import rerun.blueprint as rrb

    from dimos.memory.vis.color import Color
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, _get_colormap_lut
    from dimos.visualization.rerun.init import rerun_init

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

    answer_parts = [
        det.pointcloud.points_f32()
        for _, trace in traces
        for members in trace.answers
        for det in members
    ]

    def at(ts: float) -> None:
        rr.set_time("ts", timestamp=ts)

    def carve(points: np.ndarray, spacing: float) -> np.ndarray:
        """Drop map cells claimed by a detected object, plus one cell of halo."""
        if not answer_parts:
            return points

        def keys(cells: np.ndarray) -> np.ndarray:
            return (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]

        offset = 1 << 20
        claimed = np.floor(np.vstack(answer_parts) / spacing).astype(np.int64) + offset
        claimed = np.unique(claimed, axis=0)
        steps = np.array([-1, 0, 1])
        neighbors = np.stack(np.meshgrid(steps, steps, steps), -1).reshape(-1, 3)
        dilated = (claimed[:, None, :] + neighbors[None, :, :]).reshape(-1, 3)
        cells = np.floor(points / spacing).astype(np.int64) + offset
        return points[~np.isin(keys(cells), np.unique(keys(dilated)))]

    def height_points(points: np.ndarray, radius: float, blend: float, scale: float) -> Any:
        """Height colormap, grayed by ``blend`` and dimmed by ``scale`` so
        textured detections pop."""
        z = points[:, 2]
        t = (z - z.min()) / (z.max() - z.min() + 1e-8)
        turbo = _get_colormap_lut("turbo")[(t * 255).astype(np.uint8)]
        colors = ((turbo * (1 - blend) + blend * 205) * scale).astype(np.uint8)
        return rr.Points3D(positions=points, colors=colors, radii=radius)

    def image_colors(det: Any) -> np.ndarray:
        """Sample the sighting's own image at each cloud point's reprojection."""
        points = det.pointcloud.points_f32()
        matrix = det.transform.to_matrix()
        cam = points @ matrix[:3, :3].T + matrix[:3, 3]
        pixels = Detection3DPC.project_pixels(cam, rig.cameras[rig.optical_frame])
        cols = np.round(pixels[:, 0]).astype(int)
        rows = np.round(pixels[:, 1]).astype(int)
        rgb = det.image.to_rgb().data
        height, width = rgb.shape[:2]
        return rgb[np.clip(rows, 0, height - 1), np.clip(cols, 0, width - 1)]

    # scene backdrop: stationary depth rigs get the answer frame's RGBD
    # cloud, mobile depth rigs the window's frames merged, pointcloud rigs
    # the window's scans merged; point size follows the source's spacing
    point_size = 0.005 if rig.depth is not None else 0.015
    if rig.depth is not None and rig.mobile:
        parts = []
        for obs in rig.color.after(t0).before(t1).transform(throttle(1.0)):
            cloud = rig.backdrop(obs.ts, depth_trunc=4.0)
            if cloud is not None:
                parts.append(cloud.voxel_downsample(0.03).points_f32())
        if parts:
            merged = PointCloud2.from_numpy(np.vstack(parts), frame_id=rig.world_frame)
            carved = carve(merged.voxel_downsample(0.03).points_f32(), 0.03)
            rr.log("map", height_points(carved, 0.013, 0.7, 0.5), static=True)
    elif rig.depth is not None:
        backdrop_ts = next((t.backdrop_ts for _, t in traces if t.backdrop_ts is not None), None)
        if backdrop_ts is None:
            backdrop_ts = next((t.first_match_ts for _, t in traces if t.first_match_ts), None)
        if backdrop_ts is not None:
            backdrop = rig.backdrop(backdrop_ts)
            if backdrop is not None:
                carved = carve(backdrop.voxel_downsample(0.01).points_f32(), 0.01)
                rr.log("map", height_points(carved, point_size / 2, 0.0, 1.0), static=True)
    else:
        scans = [
            points
            for obs in rig.cloud.after(t0).before(t1).transform(throttle(2.0))
            if (points := rig.registered_scan(obs)) is not None
        ]
        if scans:
            point_size = lattice_quantum(scans[0]) or point_size
            merged = PointCloud2.from_numpy(np.vstack(scans), frame_id=rig.world_frame)
            carved = carve(merged.voxel_downsample(0.05).points_f32(), 0.05)
            rr.log("map", height_points(carved, 0.022, 0.7, 0.5), static=True)

    # live camera feed + frustum tracking the capture pose along the
    # timeline; on mobile rigs a translucent box marks the robot
    rr.log("camera", rig.cameras[rig.optical_frame].to_rerun(), static=True)
    if rig.mobile:
        rr.log(
            "robot",
            rr.Boxes3D(half_sizes=[[0.2, 0.2, 0.2]], colors=[[120, 120, 120, 70]]),
            static=True,
        )
    feed_throttle = 0.1 if (t1 - t0) <= 160 else 0.4
    feed = rig.color.after(t0).before(t1).transform(throttle(feed_throttle))
    for obs in feed:
        pose = rig.camera_pose(obs.ts)
        if pose is None:
            continue
        at(obs.ts)
        rr.log("camera/image", obs.data.to_rerun())
        rr.log("camera", pose.to_rerun())
        if rig.mobile:
            rr.log("robot", pose.to_rerun())

    n_instances = sum(len(trace.answers) for _, trace in traces)
    box_colors = [
        list(Color.from_cmap("turbo", k / max(n_instances - 1, 1)).rgb_u8())
        for k in range(n_instances)
    ]
    box_index = 0

    for query, trace in traces:
        root = f"detections/{query.replace(' ', '_')}"

        # marked frames into the live feed; only stationary rigs also drop
        # a frozen frustum at the capture pose
        for i, obs in enumerate(trace.detection_frames):
            pose = rig.camera_pose(obs.ts)
            if pose is None:
                continue
            at(obs.ts)
            annotated = obs.data.annotated_image()
            rr.log("camera/image", annotated.to_rerun())
            if rig.mobile:
                continue
            frame = f"{root}/frames/{i}"
            rr.log(frame, pose.to_rerun())
            rr.log(frame, rig.cameras[rig.optical_frame].to_rerun())
            rr.log(f"{frame}/image", annotated.to_rerun())

        # the answers: per verified instance, every sighting's cloud colored
        # by its own image, wrapped in a static labeled wireframe box
        for i, members in enumerate(trace.answers):
            at(max(det.ts for det in members))
            positions = np.vstack([det.pointcloud.points_f32() for det in members])
            colors = np.vstack([image_colors(det) for det in members])
            rr.log(
                f"{root}/answer/{i}",
                rr.Points3D(positions=positions, colors=colors, radii=point_size / 2),
            )
            aabb_min, aabb_max = positions.min(axis=0), positions.max(axis=0)
            score = max(det.confidence for det in members)
            rr.log(
                f"{root}/answer/{i}/box",
                rr.Boxes3D(
                    centers=[(aabb_min + aabb_max) / 2],
                    half_sizes=[(aabb_max - aabb_min) / 2],
                    colors=[box_colors[box_index]],
                    labels=[f"{query} {score:.2f}"],
                    fill_mode=rr.components.FillMode.MajorWireframe,
                ),
                static=True,
            )
            box_index += 1


def report(query: str, hits: list[Localization], lo: float) -> bool:
    """Print one query's instances; True when any is verified."""
    if not hits:
        print(f"no verified detection of {query!r}")
        return False
    for hit in hits:
        offset = hit.pose_timestamp - lo
        if hit.position_world_xyz is None:
            print(
                f"hit {query!r} without pose: reason={hit.reason} "
                f"score={hit.semantic_score:.2f} ts_offset={offset:.1f}s"
            )
            continue
        x, y, z = hit.position_world_xyz
        cloud_points = len(hit.point_cloud) if hit.point_cloud is not None else 0
        print(
            f"hit {query!r} [{hit.instance_id}]: position=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"frame={hit.frame_id} ts_offset={offset:.1f}s points={cloud_points} "
            f"views={hit.n_views} score={hit.semantic_score:.2f} "
            f"margin={hit.ambiguity_margin:.2f}"
        )
        if hit.ambiguity_margin < REFUSAL_MARGIN:
            print(
                f"ambiguity: margin {hit.ambiguity_margin:.2f} below refusal threshold "
                f"{REFUSAL_MARGIN:.2f} - coexisting matches, this instance is flagged"
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
    parser.add_argument("--manifest", type=Path, help="rig manifest json (default: <db>.rig.json)")
    parser.add_argument("--color", help="stream name override for the color role")
    parser.add_argument("--depth", help="stream name override for the depth role")
    parser.add_argument("--cloud", help="stream name override for the pointcloud role")
    parser.add_argument("--odom", help="stream name override for the poses role")
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
        help="pass all queries to localize() as one list sharing one detection pass",
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
    manifest = json.loads(args.manifest.read_text()) if args.manifest else None
    overrides = {
        role: name
        for role, name in [
            ("color", args.color),
            ("depth", args.depth),
            ("cloud", args.cloud),
            ("poses", args.odom),
        ]
        if name
    }
    store = SqliteStore(path=dataset)
    rig = Rig.from_store(store, manifest=manifest, overrides=overrides)
    lo, hi = rig.color.get_time_range()
    after = lo + args.start
    before = lo + args.start + args.duration if args.duration is not None else hi

    from dimos.perception.memory.dandetect import DanDetector

    traces: list[tuple[str, LocalizeTrace]] = []
    hits = 0
    with DanDetector() as dan:
        index = dan.embed(store, after, before, rig=rig)
        if args.multi:
            qtraces = [LocalizeTrace() for _ in queries]
            results = dan.localize(
                store,
                queries,
                index=index,
                rig=rig,
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
                    rig=rig,
                    require_pose=not args.allow_no_pose,
                    trace=trace,
                )
                if report(query, hit, lo):
                    hits += 1
                    traces.append((query, trace))

    if not hits:
        return 1

    if out is not None:
        render(out, rig, traces, after, before)
        print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
