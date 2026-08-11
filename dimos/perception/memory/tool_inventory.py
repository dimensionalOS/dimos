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

Run: uv run python -m dimos.perception.memory.tool_inventory
         [--from <s>] [--duration <s>] [--include-ungrounded] [--log-progress]

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
"""

import argparse
from pathlib import Path
import sys

from dimos.memory2.store.sqlite import SqliteStore
from dimos.perception.memory.inventory import inventory
from dimos.utils.data import get_data


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--dataset", type=Path, help="memory2 recording database")
    parser.add_argument(
        "--from", dest="start", type=float, default=0.0, help="start offset into the recording (s)"
    )
    parser.add_argument("--duration", type=float, default=None, help="how much to parse (s)")
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

    dataset = args.dataset or get_data(
        "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
        "xarm6_worldbelief_20260729_203624_161992.db"
    )
    store = SqliteStore(path=dataset)
    lo, _ = store.streams.color_image.get_time_range()
    after = lo + args.start
    before = lo + args.start + args.duration if args.duration is not None else None

    instances = inventory(
        store,
        after=after,
        before=before,
        include_ungrounded=args.include_ungrounded,
        log_progress=args.log_progress,
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
