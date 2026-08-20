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

"""Enumerate deduplicated object instances in a recording window.

Run: uv run python -m dimos.perception.memory.tool_inventory [out.rrd]
         [--from <s>] [--duration <s>] [--labels <group> ...] [--no-vocabulary]
         [--include-ungrounded] [--log-progress]

``--labels`` replaces the candidate names with the caller's own. Each token
is one group: the first phrase is the canonical label, and ``|`` separates
synonyms (``"pen|ballpoint pen|ink pen"``). With none given the run uses
``DEFAULT_VOCABULARY``, and ``--no-vocabulary`` names nothing. Naming
abstains either way: a name is reported only when it beats its runner-up by
``InventoryPolicy.name_refusal_margin``, else the instance is ``unknown-N``.

Stdout contract: a summary line ``instances: N`` followed by one line per
instance: ``<i>  id=<run_local_id>  name=<str>  xyz=(x,y,z)  ts_offset=<s>
members=<k>  extent=(x,y,z)  sigma=(x,y,z)  coverage=<f>``. Exit code 0
whenever the call completes; an empty scene is ``instances: 0``, not a
failure.

``extent`` is the instance's bounding size in meters and ``sigma`` the
spread of its member centroids, so a caller can check an object against a
gripper without a second query. Both are what the views actually saw:
``coverage`` is the fraction of viewing azimuth covered, and an instance
seen from one side reports the extent of that side.

The instance list reports the scene as of the window's end: position and
timestamp come from each instance's latest member observation. An object
moved between rest positions inside the window registers once per rest
position - linking rest positions of one object across time is
re-identification, which this tool does not do.

The .rrd holds the same instances the stdout lines report, no second
perception pass: the member clouds on the timeline, one labeled box per
instance in 3D, and each member's pixel box on the camera view at the
keyframe it came from.
"""

import argparse
from pathlib import Path
import sys
from typing import cast

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.transform import throttle
from dimos.perception.memory.inventory import DEFAULT_VOCABULARY, NamingVocabulary
from dimos.perception.memory.rig import Rig
from dimos.perception.memory.types import Instance, SupportObservation
from dimos.utils.data import get_data


def labels_to_vocabulary(tokens: list[str]) -> NamingVocabulary:
    """Parse ``--labels`` tokens into synonym groups.

    Each token is one group. Phrases inside a token are separated by ``|``.
    The first non-empty phrase is the canonical label.
    """
    groups: list[tuple[str, ...]] = []
    for token in tokens:
        surfaces = tuple(part.strip() for part in token.split("|") if part.strip())
        if not surfaces:
            raise ValueError(f"empty --labels group: {token!r}")
        groups.append(surfaces)
    return tuple(groups)


def instance_label(instance: Instance) -> str:
    """``obj-NN name score`` - the score only when that name won the instance."""
    if instance.labels and instance.labels[0][0] == instance.primary_label:
        return f"{instance.instance_id} {instance.primary_label} {instance.labels[0][1]:.2f}"
    return f"{instance.instance_id} {instance.primary_label}"


def render(out: str, rig: Rig, instances: list[Instance], t0: float, t1: float) -> None:
    """Write the .rrd - rerun stays an inline import.

    Entity contract: ``map`` backdrop, ``camera/image`` the live feed carrying
    the per-keyframe pixel boxes, ``instances/<id>_<name>`` the member clouds
    with a static labeled box. One color per instance across both views.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    from dimos.memory.vis.color import Color
    from dimos.visualization.rerun.init import rerun_init

    rerun_init("memory-inventory")
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

    point_size = 0.005 if rig.depth is not None else 0.015

    def at(ts: float) -> None:
        rr.set_time("ts", timestamp=ts)

    grounded = [instance for instance in instances if instance.support is not None]
    colors = [
        list(Color.from_cmap("turbo", i / max(len(grounded) - 1, 1)).rgb_u8())
        for i in range(len(grounded))
    ]
    labels = [instance_label(instance) for instance in grounded]
    paths = [
        f"instances/{instance.instance_id}_{cast('str', instance.primary_label).replace(' ', '_')}"
        for instance in grounded
    ]

    frames: dict[float, list[tuple[int, SupportObservation]]] = {}
    for i, instance in enumerate(grounded):
        for member in instance.members:
            frames.setdefault(member.ts, []).append((i, member))

    # scene backdrop: for depth rigs the last instance keyframe's RGBD cloud,
    # for pointcloud rigs the window's scans merged into one map
    if rig.depth is not None:
        backdrop_ts = max(frames, default=None)
        if backdrop_ts is not None:
            backdrop = rig.backdrop(backdrop_ts)
            if backdrop is not None:
                rr.log(
                    "map",
                    backdrop.voxel_downsample(0.01).to_rerun(voxel_size=point_size),
                    static=True,
                )
    else:
        import numpy as np

        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        scans = [
            obs.data.as_numpy()[0]
            for obs in rig.cloud.after(t0).before(t1).transform(throttle(2.0))
        ]
        if scans:
            merged = PointCloud2.from_numpy(np.vstack(scans), frame_id=rig.world_frame)
            rr.log("map", merged.voxel_downsample(0.05).to_rerun(voxel_size=0.01), static=True)

    # live camera feed + frustum; the empty box clears the overlay off non-keyframes
    rr.log("camera", rig.camera_info.to_rerun(), static=True)
    feed_throttle = 0.1 if (t1 - t0) <= 160 else 0.4
    feed = rig.color.after(t0).before(t1).transform(throttle(feed_throttle))
    for obs in feed:
        pose = rig.camera_pose(obs.ts)
        if pose is None:
            continue
        at(obs.ts)
        rr.log("camera/image", obs.data.to_rerun())
        rr.log("camera", pose.to_rerun())
        rr.log("camera/image/instances", rr.Boxes2D(array=[], array_format=rr.Box2DFormat.XYXY))

    # keyframes, logged after the feed so their boxes win the shared timestamps
    for ts, entries in sorted(frames.items()):
        at(ts)
        keyframe_pose = rig.camera_pose(ts)
        assert keyframe_pose is not None
        rr.log("camera/image", rig.color.at(ts, 0.05).first().data.to_rerun())
        rr.log("camera", keyframe_pose.to_rerun())
        rr.log(
            "camera/image/instances",
            rr.Boxes2D(
                array=[
                    cast("tuple[float, float, float, float]", member.bbox) for _, member in entries
                ],
                array_format=rr.Box2DFormat.XYXY,
                labels=[labels[i] for i, _ in entries],
                colors=[colors[i] for i, _ in entries],
            ),
        )
        for i, member in entries:
            rr.log(paths[i], member.cloud.to_rerun(voxel_size=point_size, colors=colors[i]))

    # the reported instance: one labeled box, static so it holds over the whole timeline
    for i, instance in enumerate(grounded):
        support = instance.support
        assert support is not None
        cx, cy, cz = support.center_xyz
        ex, ey, ez = support.extent_xyz_m
        rr.log(
            f"{paths[i]}/box",
            rr.Boxes3D(
                centers=[(cx, cy, cz)],
                half_sizes=[(ex / 2, ey / 2, ez / 2)],
                colors=[colors[i]],
                labels=[labels[i]],
                fill_mode=rr.components.FillMode.MajorWireframe,
            ),
            static=True,
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "out", nargs="?", default=None, help="rerun recording to write; omitted writes none"
    )
    parser.add_argument("--dataset", type=Path, help="memory recording database")
    parser.add_argument(
        "--from", dest="start", type=float, default=0.0, help="start offset into the recording (s)"
    )
    parser.add_argument("--duration", type=float, default=None, help="how much to parse (s)")
    vocab = parser.add_mutually_exclusive_group()
    vocab.add_argument(
        "--labels",
        nargs="+",
        metavar="GROUP",
        help=(
            "candidate name groups; each token is one group, "
            "'|' separates synonyms (first phrase is canonical); "
            "default is DEFAULT_VOCABULARY"
        ),
    )
    vocab.add_argument(
        "--no-vocabulary",
        action="store_true",
        help="name nothing: every instance stays unknown-N",
    )
    parser.add_argument(
        "--include-ungrounded",
        action="store_true",
        help="also list RGB-only instances that never produced valid depth",
    )
    parser.add_argument(
        "--log-progress",
        action="store_true",
        help="per-keyframe discovery progress lines (off by default)",
    )
    args = parser.parse_args()

    out = args.out
    if args.labels and args.labels[-1].endswith(".rrd"):
        out = args.labels.pop()

    dataset = args.dataset or get_data(
        "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
        "xarm6_worldbelief_20260729_203624_161992.db"
    )
    store = SqliteStore(path=dataset)
    rig = Rig.from_store(store)
    lo, hi = rig.color.get_time_range()
    after = lo + args.start
    before = lo + args.start + args.duration if args.duration is not None else None

    if args.no_vocabulary:
        naming_vocabulary: NamingVocabulary = ()
    elif args.labels:
        naming_vocabulary = labels_to_vocabulary(args.labels)
    else:
        naming_vocabulary = DEFAULT_VOCABULARY

    from dimos.perception.memory.dandetect import DanDetector

    with DanDetector() as dan:
        instances = dan.inventory(
            store,
            naming_vocabulary=naming_vocabulary,
            after=after,
            before=before,
            include_ungrounded=args.include_ungrounded,
            log_progress=args.log_progress,
            rig=rig,
        )

    print(f"instances: {len(instances)}")
    for i, instance in enumerate(instances):
        if instance.latest_position_xyz is not None:
            x, y, z = instance.latest_position_xyz
            xyz = f"({x:.3f},{y:.3f},{z:.3f})"
        else:
            xyz = "None"
        if instance.support is not None:
            ex, ey, ez = instance.support.extent_xyz_m
            sx, sy, sz = instance.support.sigma_xyz_m
            geometry = (
                f"  extent=({ex:.3f},{ey:.3f},{ez:.3f})"
                f"  sigma=({sx:.3f},{sy:.3f},{sz:.3f})"
                f"  coverage={instance.support.coverage:.2f}"
            )
        else:
            geometry = ""
        print(
            f"{i}  id={instance.instance_id}  name={instance.primary_label}  "
            f"xyz={xyz}  ts_offset={instance.latest_seen_ts - lo:.1f}  "
            f"members={len(instance.members)}{geometry}"
        )

    if out is not None:
        render(out, rig, instances, after, before if before is not None else hi)
        print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
