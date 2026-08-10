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

"""Query memory for an object, localize it in 3D via depth, render in rerun.

Run: uv run python -m dimos.perception.memory.tool_localize [query] [out.rrd]
         [--from <s>] [--duration <s>]
"""

import argparse
from pathlib import Path

import numpy as np

from dimos.memory2.embed import EmbedImages
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.tf import StreamTF
from dimos.memory2.transform import throttle
from dimos.memory2.utils.progress import progress
from dimos.models.embedding.clip import CLIPModel
from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.project import ProjectDepthTo3D, sees
from dimos.utils.data import get_data
from dimos.visualization.rerun.init import rerun_init

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("query", nargs="?", default="plant")
parser.add_argument("out", nargs="?", default="localize.rrd")
parser.add_argument("--dataset", type=Path, help="memory2 recording database")
parser.add_argument(
    "--from", dest="start", type=float, default=0.0, help="start offset into the recording (s)"
)
parser.add_argument("--duration", type=float, default=None, help="how much to parse (s)")
args = parser.parse_args()
query, out = args.query, args.out

dataset = args.dataset or get_data(
    "xarm6_worldbelief_realsense_d435i_stationery_calibrated/xarm6_worldbelief_20260729_203624_161992.db"
)
store = SqliteStore(path=dataset)
tf = StreamTF.from_store(store)
camera_info = store.streams.camera_info.first().data
OPTICAL_FRAME = "camera_color_optical_frame"

lo, hi = store.streams.color_image.get_time_range()
t0 = lo + args.start
t1 = min(hi, t0 + args.duration) if args.duration is not None else hi
images = store.streams.color_image.after(t0).before(t1)
span = t1 - t0
print(f"window: {args.start:.0f}s → {args.start + span:.0f}s of the recording ({span:.0f}s)")


def camera_pose(ts):
    """World pose of the camera optical frame at ts — it rides the wrist."""
    return (-tf.get(OPTICAL_FRAME, "world", ts, 0.5)).to_pose()


def camera_speed(ts, dt=0.06):
    """Linear speed of the camera (m/s) around ts, from tf."""
    a, b = camera_pose(ts - dt), camera_pose(ts + dt)
    return float((b.position - a.position).magnitude() / (2 * dt))


def still(ts, envelope=0.15):
    """Camera is still over the whole capture envelope, not just at ts."""
    return all(camera_speed(ts + o) <= SPEED_MAX for o in (-envelope, 0.0, envelope))


# Embed only the requested window and keep the source recording read-only.
clip = CLIPModel()
with progress(int(span) + 1, "embed") as bar:
    embedded = (
        images.transform(throttle(1.0))
        .map(lambda obs: obs.derive(data=obs.data, pose=camera_pose(obs.ts)))
        .transform(EmbedImages(clip))
        .tap(bar)
        .materialize()
    )

# motion gate: only capture while the wrist is (near-)still — the arm parks
# often, and any real motion measurably blurs the frames
SPEED_MAX = 0.02  # m/s
speeds = [camera_speed(obs.ts) for obs in embedded.after(t0).before(t1)]
print(f"motion gate: > {SPEED_MAX} m/s rejects {np.mean([s > SPEED_MAX for s in speeds]):.0%}")

# embeddings zone us into the frames worth looking at.
# rank ALL frames then window: the backend applies .search() before the time
# filters, so a windowed top-k would keep only global hits inside the window
matches = (
    embedded.search(clip.embed_text(query), k=embedded.count())
    .after(t0)
    .before(t1)
    .filter(lambda obs: still(obs.ts))
    .limit(12)
    .materialize()
)
print(matches.summary())
clip.stop()  # free GPU memory for the VLM + segmenter

moondream = MoondreamVlModel()
moondream.start()
segmenter = EdgeTAMImageSegmenter()


def detect(frames):
    """VLM detections refined into masks, frames without detections dropped."""
    return (
        frames.map_data(lambda obs: moondream.query_detections(obs.data, query))
        .map_data(lambda obs: obs.data.filter(lambda det: det.bbox_2d_volume() > 3000))
        .filter(lambda obs: len(obs.data) > 0)
        .map_data(lambda obs: segmenter.segment(obs.data))
    )


with progress(matches.count(), "detect") as bar:
    detections = detect(matches.tap(bar)).materialize()
print(f"{detections.count()} frames with 2d detections")


def depth_at(obs):
    """Temporal join: aligned depth frame for a color observation.

    One frame of tolerance — a farther depth frame may come from mid-motion.
    """
    nearest = store.streams.depth_image.at(obs.ts, 0.05).first()
    return nearest.data if nearest is not None else None


# lift into 3D straight through the depth image — no map needed
# (tight tf tolerance: the arm moves, a stale transform smears the projection)
project = ProjectDepthTo3D(
    depth_at,
    camera_info,
    tf=tf,
    optical_frame=OPTICAL_FRAME,
    time_tolerance=0.5,
    filters=[],  # no pointcloud filtering for now
)
with progress(detections.count(), "project 3d") as bar:
    detections3d = (
        detections.tap(bar).transform(project).filter(lambda obs: len(obs.data) > 0).materialize()
    )
print(f"{detections3d.count()} frames with 3d detections")

# every frame that was looking at the strongest detection
best = max((det for obs in detections3d for det in obs.data), key=lambda det: len(det.pointcloud))
with progress(int(span / 0.25) + 1, "observing") as bar:
    observing = (
        images.transform(throttle(0.25))
        .tap(bar)
        .filter(
            sees(best.pose, camera_info, tf=tf, optical_frame=OPTICAL_FRAME, time_tolerance=0.5)
        )
        .filter(lambda obs: still(obs.ts))
        .materialize()
    )
print(f"{observing.count()} frames observing the detection at {best.pose.position}")

# re-detect from observing frames and project those into 3D too
verify_frames = observing.limit(32).materialize()
with progress(verify_frames.count(), "cross-view") as bar:
    verified3d = (
        detect(verify_frames.tap(bar))
        .transform(project)
        .filter(lambda obs: len(obs.data) > 0)
        .materialize()
    )
print(f"cross-view: {verified3d.count()} observing frames re-detect in 3d")

# Rerun output uses the source timestamps directly.


def render() -> None:
    """Write the .rrd — rerun stays an inline import."""
    import rerun as rr
    import rerun.blueprint as rrb

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

    GREEN, RED = [46, 204, 113], [231, 76, 60]
    POINT_SIZE = 0.005

    def at(ts: float) -> None:
        rr.set_time("ts", timestamp=ts)

    # scene backdrop from one depth frame
    backdrop_obs = detections3d.first()
    backdrop = PointCloud2.from_rgbd(
        backdrop_obs.data.image, depth_at(backdrop_obs), camera_info, depth_scale=0.001
    ).transform(-tf.get(OPTICAL_FRAME, "world", backdrop_obs.ts, 0.5))
    rr.log("map", backdrop.voxel_downsample(0.01).to_rerun(voxel_size=POINT_SIZE), static=True)

    # live camera feed + camera frustum tracking the wrist along the timeline
    rr.log("camera", camera_info.to_rerun(), static=True)
    for obs in images.transform(throttle(0.1)):
        at(obs.ts)
        rr.log("camera/image", obs.data.to_rerun())
        rr.log("camera", camera_pose(obs.ts).to_rerun())

    # marked frames: into the live feed, plus a frozen frustum pinned at the
    # capture pose (not the moving camera entity, which would drag them along)
    for i, obs in enumerate(detections):
        at(obs.ts)
        annotated = obs.data.annotated_image()
        rr.log("camera/image", annotated.to_rerun())
        frame = f"detections/frames/{i}"
        rr.log(frame, camera_pose(obs.ts).to_rerun())
        rr.log(frame, camera_info.to_rerun())
        rr.log(f"{frame}/image", annotated.to_rerun())

    # 3d detections: green = embedding matches, red = cross-view re-detections
    for tag, stream, rgb in [("matched", detections3d, GREEN), ("verified", verified3d, RED)]:
        for i, obs in enumerate(stream):
            at(obs.ts)
            for j, det in enumerate(obs.data):
                rr.log(
                    f"detections/{tag}/{i}_{j}_{det.name.replace(' ', '_')}",
                    det.pointcloud.to_rerun(voxel_size=POINT_SIZE, colors=rgb),
                )


render()
print(f"saved {out}")
