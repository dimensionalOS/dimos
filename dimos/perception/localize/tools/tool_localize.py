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

"""Query a recording for objects, localize them in 3D, render in rerun.

Run: uv run python -m dimos.perception.localize.tools.tool_localize
         [query ...] [out.rrd] [--dataset <db>] [--from <s>] [--duration <s>] [--multi]
         [--policy <json>] [--embedder <m>] [--detector <m>] [--segmenter <m>]

The offline bench for a stage combination: try it on a recording and find
its policy before deploying. Each stage takes a dandetect registry name or
a module:attr path; the policy starts at the chosen models' shifts.

Every calibrated camera with colour runs in one pass, lifted through its
aligned depth or, without one, the recording's lidar scans of the window merged.
With --multi the queries go to localize() as one list and share one
detection pass; without it each query runs alone. --policy overrides
LocalizePolicy fields. Every query prints one line per verified instance,
plus where each camera puts it on a multi-camera rig. Exit code 0 when any
query verified; 1 when none did, with "no verified detection of ..." per
miss. An ambiguous instance (identical twins in view) is flagged.
"""

import argparse
from dataclasses import asdict
import heapq
from itertools import combinations
import json
from pathlib import Path
import sys
from typing import Any

from dimos_generated.geometry_msgs.msg import PoseStamped
from dimos_generated.sensor_msgs.msg import Image
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.transform import throttle
from dimos.msgs.geometry import (
    inverse_transform,
    pose_from_transform,
    pose_matrix,
    transform_matrix,
    vector_array,
)
from dimos.msgs.image import image_from_array, image_to_bgr, image_to_rgb
from dimos.msgs.pointcloud import pointcloud_xyz, voxel_downsample_cloud
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC, lattice_quantum
from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.localize import LocalizeTrace, embed_index
from dimos.perception.localize.rig import TF_TOLERANCE, Rig, window_tf
from dimos.perception.localize.types import Localization, LocalizePolicy
from dimos.utils.data import get_data

XARM_DATASET = (
    "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
    "xarm6_worldbelief_20260729_203624_161992.db"
)
REFUSAL_MARGIN = 0.15
POINT_SIZE = 0.005  # m, depth camera point spacing
LIDAR_POINT_SIZE = 0.03  # m, a lidar cloud that carries no lattice pitch
MAP_CELL = 0.01  # m, backdrop voxel and carve cell
BASE_FRAME = "base_link"  # a mobile robot's body frame
ROBOT_HALF_SIZES = (0.35, 0.155, 0.2)  # m, the Go2 body box the go2 blueprints draw
ROBOT_COLOR = (0, 255, 127)
MAP_SATURATION = 0.5  # of the height colours kept, so the textured answers stand out
MASK_ALPHA = 0.45  # mask tint over the camera image
LABEL_COLORS = (  # RGB per query, in query order
    (31, 119, 180),
    (255, 127, 14),
    (44, 160, 44),
    (214, 39, 40),
    (148, 103, 189),
    (140, 86, 75),
    (227, 119, 194),
    (188, 189, 34),
    (23, 190, 207),
    (127, 127, 127),
)


def render(
    out: str,
    rig: Rig,
    traces: list[tuple[str, LocalizeTrace]],
    backdrop_ts: float,
    t0: float,
    t1: float,
) -> None:
    """Write the .rrd - rerun stays an inline import.

    Entity contract: ``map/<camera>`` is each depth camera's RGBD cloud at
    ``backdrop_ts`` and ``map/lidar`` the lidar's scans over the window, both
    carved free of every answer's cells and desaturated. Each
    ``detections/<query>/answer/<k>`` is one verified instance, its sightings
    split per camera under ``<k>/<camera>`` and textured with each sighting's
    own image, wrapped in a labeled wireframe box; answers are static. Every
    camera has its live feed and frustum under ``cameras/<camera>``; each
    detection frame shows in that feed once with every query's masks, and as a
    frozen frustum per query under ``detections/<query>/frames``. A robot, a
    rig whose tf has a ``base_link``, is a body box under ``robot`` moving
    with it. The 3D view hides those frames, and ``cameras`` unless there is
    a robot, until toggled on.
    """
    from matplotlib import colormaps
    import rerun as rr
    import rerun.blueprint as rrb

    from dimos.memory.vis.color import Color
    from dimos.visualization.rerun.init import rerun_init
    from dimos.visualization.rerun.message_helpers import camera_pinhole, image_archetype

    names = {frame: rr.escape_entity_path_part(frame) for frame in rig.cameras}
    robot = base_pose(rig, backdrop_ts) is not None
    hidden = [] if robot else ["cameras"]
    hidden += [f"detections/{query.replace(' ', '_')}/frames" for query, _ in traces]
    rerun_init("memory-localize")
    rr.save(out)
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    origin="/",
                    name="Scene",
                    overrides={path: rrb.EntityBehavior(visible=False) for path in hidden},
                ),
                rrb.Vertical(
                    *(
                        rrb.Spatial2DView(origin=f"cameras/{name}", name=frame)
                        for frame, name in names.items()
                    )
                ),
                column_shares=[2, 1],
            )
        )
    )

    def pose_archetype(pose: PoseStamped) -> Any:
        matrix = pose_matrix(pose.pose)
        return rr.Transform3D(translation=matrix[:3, 3], mat3x3=matrix[:3, :3])

    def at(ts: float) -> None:
        rr.set_time("ts", timestamp=ts)

    def cell_keys(points: np.ndarray) -> np.ndarray:
        cells = np.floor(points / MAP_CELL).astype(np.int64) + (1 << 20)
        return (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]

    # map cells claimed by a detected object, plus one cell of halo
    answer_points = np.vstack(
        [
            pointcloud_xyz(det.pointcloud)
            for _, trace in traces
            for members in trace.answers
            for det in members
        ]
    )
    steps = np.array([-1, 0, 1])
    halo = (steps[:, None, None] << 42) + (steps[None, :, None] << 21) + steps[None, None, :]
    claimed = np.unique((np.unique(cell_keys(answer_points))[:, None] + halo.ravel()).ravel())

    def carve(points: np.ndarray) -> np.ndarray:
        return points[~np.isin(cell_keys(points), claimed)]

    def image_colors(det: Any) -> np.ndarray:
        """Sample the sighting's own image at each cloud point's reprojection."""
        points = pointcloud_xyz(det.pointcloud)
        matrix = transform_matrix(det.transform.transform)
        cam = points @ matrix[:3, :3].T + matrix[:3, 3]
        pixels = Detection3DPC.project_pixels(cam, rig.cameras[det.image.header.frame_id].info)
        cols = np.round(pixels[:, 0]).astype(int)
        rows = np.round(pixels[:, 1]).astype(int)
        rgb = image_to_rgb(det.image)
        height, width = rgb.shape[:2]
        return rgb[np.clip(rows, 0, height - 1), np.clip(cols, 0, width - 1)]

    # backdrop: each depth camera's RGBD cloud at the answer frame and the
    # lidar's scans over the window, height-colored over the range they share
    through_lidar = {frame for frame, camera in rig.cameras.items() if camera.depth is None}
    views: dict[str, tuple[np.ndarray, float]] = {}
    for frame, name in names.items():
        backdrop = rig.backdrop(backdrop_ts, frame) if frame not in through_lidar else None
        if backdrop is not None:
            carved = carve(pointcloud_xyz(voxel_downsample_cloud(backdrop, MAP_CELL)))
            if len(carved):
                views[name] = (carved, POINT_SIZE)
    lidar_size = LIDAR_POINT_SIZE
    lidar_map = rig.lidar_map() if through_lidar else None
    if lidar_map is not None:
        points = pointcloud_xyz(lidar_map)
        lidar_size = lattice_quantum(points) or LIDAR_POINT_SIZE
        carved = carve(points)
        if len(carved):
            views["lidar"] = (carved, lidar_size)
    size = {frame: lidar_size if frame in through_lidar else POINT_SIZE for frame in rig.cameras}
    if views:
        heights = np.concatenate([carved[:, 2] for carved, _ in views.values()])
        low, high = float(heights.min()), float(heights.max())
        turbo = (colormaps["turbo"](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
        for name, (carved, spacing) in views.items():
            t = np.clip((carved[:, 2] - low) / (high - low + 1e-8), 0.0, 1.0)
            colors = turbo[(t * 255).astype(np.uint8)].astype(np.float32)
            gray = colors @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
            colors = gray[:, None] + MAP_SATURATION * (colors - gray[:, None])
            rr.log(
                f"map/{name}",
                rr.Points3D(positions=carved, colors=colors.astype(np.uint8), radii=spacing / 2),
                static=True,
            )

    # each camera's live feed + frustum tracking its capture pose along the
    # timeline; a robot's body box follows its base frame
    for frame, name in names.items():
        rr.log(
            f"cameras/{name}",
            camera_pinhole(rig.cameras[frame].info, optical_frame=""),
            static=True,
        )
    if robot:
        body = rr.Boxes3D(half_sizes=[ROBOT_HALF_SIZES], colors=[ROBOT_COLOR])
        rr.log("robot", body, static=True)
    feed = heapq.merge(
        *(
            camera.color.after(t0).before(t1).transform(throttle(0.1 if (t1 - t0) <= 160 else 0.4))
            for camera in rig.cameras.values()
        ),
        key=lambda obs: obs.ts,
    )
    for obs in feed:
        frame = obs.data.header.frame_id
        pose = rig.camera_pose(obs.ts, frame)
        if pose is None:
            continue
        at(obs.ts)
        rr.log(f"cameras/{names[frame]}/image", image_archetype(obs.data))
        rr.log(f"cameras/{names[frame]}", pose_archetype(pose))
        base = base_pose(rig, obs.ts) if robot else None
        if base is not None:
            rr.log("robot", pose_archetype(base))

    n_instances = sum(len(trace.answers) for _, trace in traces)
    box_colors = [
        list(Color.from_cmap("turbo", k / max(n_instances - 1, 1)).rgb_u8())
        for k in range(n_instances)
    ]
    box_index = 0
    label_colors = {
        query: LABEL_COLORS[k % len(LABEL_COLORS)] for k, (query, _) in enumerate(traces)
    }

    # each detection frame into the live feed once: every label's masks, and each
    # box under the label that scored it highest
    shots: dict[tuple[str, float], tuple[Image, list[Any]]] = {}
    for _, trace in traces:
        for obs in trace.detection_frames:
            image = obs.data.image
            shots.setdefault((image.header.frame_id, obs.ts), (image, []))[1].extend(
                obs.data.detections
            )
    for (camera, ts), (image, dets) in sorted(shots.items(), key=lambda shot: shot[0][1]):
        if rig.camera_pose(ts, camera) is None:
            continue
        at(ts)
        marked = overlay(image, best_per_box(dets), label_colors)
        rr.log(f"cameras/{names[camera]}/image", image_archetype(marked))

    for query, trace in traces:
        root = f"detections/{query.replace(' ', '_')}"

        # a frozen frustum at each detection frame's capture pose, with this label's masks
        for i, obs in enumerate(trace.detection_frames):
            camera = obs.data.image.header.frame_id
            pose = rig.camera_pose(obs.ts, camera)
            if pose is None:
                continue
            at(obs.ts)
            frame = f"{root}/frames/{i}"
            rr.log(frame, pose_archetype(pose))
            rr.log(frame, camera_pinhole(rig.cameras[camera].info, optical_frame=""))
            marked = overlay(obs.data.image, obs.data.detections, label_colors)
            rr.log(f"{frame}/image", image_archetype(marked))

        # the answers: per verified instance, every sighting's cloud colored
        # by its own image and grouped by camera, wrapped in a labeled
        # wireframe box, all static
        for i, members in enumerate(trace.answers):
            for camera, sightings in by_camera(rig, members).items():
                rr.log(
                    f"{root}/answer/{i}/{names[camera]}",
                    rr.Points3D(
                        positions=np.vstack([pointcloud_xyz(det.pointcloud) for det in sightings]),
                        colors=np.vstack([image_colors(det) for det in sightings]),
                        radii=size[camera] / 2,
                    ),
                    static=True,
                )
            positions = np.vstack([pointcloud_xyz(det.pointcloud) for det in members])
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


def overlay(image: Image, detections: list[Any], colors: dict[str, tuple[int, int, int]]) -> Image:
    """The frame with each detection's mask tinted and outlined in its label's colour,
    its box and its label; a detection without a mask gets the box alone."""
    import cv2

    canvas = image_to_bgr(image).copy()
    tint = canvas.copy()
    for det in detections:
        mask = getattr(det, "mask", None)
        if mask is not None:
            tint[mask > 0] = colors[det.name][::-1]
    canvas = cv2.addWeighted(tint, MASK_ALPHA, canvas, 1.0 - MASK_ALPHA, 0.0)
    for det in detections:
        bgr = colors[det.name][::-1]
        mask = getattr(det, "mask", None)
        if mask is not None:
            contours, _ = cv2.findContours(
                (mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(canvas, contours, -1, bgr, 2)
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), bgr, 1)
        label = f"{det.name} {det.confidence:.2f}"
        (width, height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(canvas, (x1, y1 - height - 6), (x1 + width + 4, y1), (0, 0, 0), -1)
        cv2.rectangle(canvas, (x1, y1 - height - 6), (x1 + width + 4, y1), bgr, 1)
        cv2.putText(
            canvas, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
    return image_from_array(canvas, encoding="bgr8", header=image.header)


def best_per_box(detections: list[Any]) -> list[Any]:
    """One detection per box, under the label that scored it highest; every query
    scores the same detector boxes."""
    best: dict[tuple[int, ...], Any] = {}
    for det in detections:
        key = tuple(round(float(v)) for v in det.bbox)
        if key not in best or det.confidence > best[key].confidence:
            best[key] = det
    return list(best.values())


def base_pose(rig: Rig, ts: float) -> PoseStamped | None:
    """World pose of the robot's base frame at ts; None when tf has none."""
    transform = rig.tf.get(BASE_FRAME, rig.world_frame, ts, TF_TOLERANCE)
    return pose_from_transform(inverse_transform(transform)) if transform is not None else None


def by_camera(rig: Rig, sightings: list[Detection3DPC]) -> dict[str, list[Detection3DPC]]:
    """An instance's sightings per camera, in the rig's camera order."""
    grouped: dict[str, list[Detection3DPC]] = {frame: [] for frame in rig.cameras}
    for det in sightings:
        grouped[det.image.header.frame_id].append(det)
    return {frame: dets for frame, dets in grouped.items() if dets}


def camera_line(rig: Rig, sightings: list[Detection3DPC]) -> str:
    """Where each camera puts one instance: its sightings, views and mean
    centroid, then the largest distance between two cameras' means."""
    means: list[np.ndarray] = []
    parts: list[str] = []
    for frame, dets in by_camera(rig, sightings).items():
        centroids = [pointcloud_xyz(det.pointcloud).mean(axis=0) for det in dets]
        cameras = np.array(
            [vector_array(inverse_transform(det.transform).transform.translation) for det in dets]
        )
        views = len(np.unique(np.round(cameras, 2), axis=0))
        mean = np.mean(centroids, axis=0)
        means.append(mean)
        parts.append(
            f"{frame} sightings={len(dets)} views={views} "
            f"at ({mean[0]:.3f}, {mean[1]:.3f}, {mean[2]:.3f})"
        )
    spread = max((float(np.linalg.norm(a - b)) for a, b in combinations(means, 2)), default=0.0)
    return f"cameras: {'; '.join(parts)}; spread={100 * spread:.1f}cm"


def report(query: str, hits: list[Localization], trace: LocalizeTrace, rig: Rig, lo: float) -> bool:
    """Print one query's instances; True when any is verified."""
    if not hits:
        print(f"no verified detection of {query!r}")
        return False
    for hit, sightings in zip(hits, trace.answers, strict=True):
        x, y, z = hit.position_world_xyz
        print(
            f"hit {query!r} [{hit.instance_id}]: position=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"frame={hit.frame_id} ts_offset={hit.last_seen_timestamp - lo:.1f}s "
            f"points={hit.point_cloud.width * hit.point_cloud.height} views={hit.n_views} "
            f"score={hit.semantic_score:.2f} margin={hit.ambiguity_margin:.2f}"
        )
        if len(rig.cameras) > 1:
            print(f"  {camera_line(rig, sightings)}")
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
    parser.add_argument(
        "--from", dest="start", type=float, default=0.0, help="start offset into the recording (s)"
    )
    parser.add_argument("--duration", type=float, default=None, help="how much to parse (s)")
    parser.add_argument(
        "--multi",
        action="store_true",
        help="pass all queries to localize() as one list sharing one detection pass",
    )
    parser.add_argument("--policy", help='JSON of LocalizePolicy overrides, e.g. {"min_views": 3}')
    parser.add_argument("--embedder", default="siglip", help="frame retrieval model")
    parser.add_argument("--detector", default="owlv2", help="open-vocabulary detector")
    parser.add_argument("--segmenter", default="edgetam", help="box to mask model")
    args = parser.parse_args()

    queries = list(args.positionals)
    out = queries.pop() if queries[-1].endswith(".rrd") else None
    if not queries:
        parser.error("no queries given")

    store = SqliteStore(path=args.dataset or get_data(XARM_DATASET))
    rig = Rig.from_store(store)
    lo, hi = rig.time_range()
    after = lo + args.start
    before = lo + args.start + args.duration if args.duration is not None else hi
    rig.tf = window_tf(store, after, before)
    if rig.lidar is not None:
        rig.lidar = rig.lidar.after(after).before(before)
    detector = DanDetector(args.embedder, args.detector, args.segmenter)
    policy = detector.policy(json.loads(args.policy) if args.policy else None)
    default = LocalizePolicy()
    changed = {k: v for k, v in asdict(policy).items() if v != getattr(default, k)}
    print(f"stages: {' + '.join(detector.names)}; policy: {changed or 'default'}")

    traces: list[tuple[str, LocalizeTrace]] = []
    backdrop_ts: float | None = None
    with detector as dan:
        index = embed_index(rig, dan.embedder, after, before)
        for batch in [queries] if args.multi else [[query] for query in queries]:
            batch_traces = [LocalizeTrace() for _ in batch]
            results = dan.localize(batch, index=index, rig=rig, policy=policy, traces=batch_traces)
            for query, trace, hits in zip(batch, batch_traces, results, strict=True):
                if report(query, hits, trace, rig, lo) and backdrop_ts is None:
                    backdrop_ts = hits[0].last_seen_timestamp
                traces.append((query, trace))

    if backdrop_ts is None:
        return 1

    if out is not None:
        render(out, rig, traces, backdrop_ts, after, before)
        print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
