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

"""Render a memory2 recording (.db) into rerun.

Loads the TF tree if the recording has one, then logs every PointCloud2 and
Odometry stream. Clouds are drawn in their stored coordinates (Point-LIO's cloud
is already registered into its world frame); each odom stream gets a moving pose
frame plus its full path as a trail. ``--seconds`` bounds the window from the
start of the recording.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

import rerun as rr
import typer

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import resolve_named_path
from dimos.visualization.rerun.init import rerun_init

TIMELINE = "time"


def _open_viewer(rrd: str) -> None:
    exe = shutil.which("rerun")
    if exe:
        subprocess.Popen([exe, rrd])
        print(f"opening {rrd} in rerun")
    else:
        print(f"rerun viewer not found on PATH; open manually:\n    rerun {rrd}")


def _classify(store: SqliteStore) -> tuple[list[str], list[str], list[str], float | None]:
    """Sort streams into (clouds, odoms, tfs) by their first observation's type,
    and return the earliest timestamp across them (the timeline anchor)."""
    clouds: list[str] = []
    odoms: list[str] = []
    tfs: list[str] = []
    t0: float | None = None
    for name in store.list_streams():
        try:
            first = store.streams[name].first()
        except LookupError:
            continue
        data = first.data
        if isinstance(data, PointCloud2):
            clouds.append(name)
        elif isinstance(data, Odometry):
            odoms.append(name)
        elif isinstance(data, TFMessage):
            tfs.append(name)
        else:
            continue
        t0 = first.ts if t0 is None else min(t0, first.ts)
    return clouds, odoms, tfs, t0


def main(
    dataset: str = typer.Argument(..., help="Recording .db: bare name (cwd or data/) or path"),
    out: Path | None = typer.Option(
        None, "--out", help="Output .rrd path (default: next to the .db)"
    ),
    seconds: float | None = typer.Option(
        None, "--seconds", help="Only render the first N seconds of the recording"
    ),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
) -> None:
    db_path = resolve_named_path(dataset, ".db")
    store = SqliteStore(path=str(db_path), must_exist=True)
    with store:
        clouds, odoms, tfs, t0 = _classify(store)
        if t0 is None:
            print("no PointCloud2 / Odometry / TF streams found in this recording")
            raise typer.Exit(1)
        print(f"clouds={clouds}")
        print(f"odoms={odoms}")
        print(f"tf={tfs or '(none)'}")

        rerun_init("db_to_rrd")
        rrd_path = str(out) if out is not None else str(db_path.with_suffix(".rrd"))
        rr.save(rrd_path)
        register_colormap_annotation("turbo")

        def in_window(ts: float) -> bool:
            return seconds is None or ts - t0 <= seconds

        # TF first so the tf graph exists before framed clouds reference it.
        for name in tfs:
            for obs in store.streams[name]:
                if not in_window(obs.ts):
                    break
                if obs.data is None:
                    continue
                rr.set_time(TIMELINE, duration=obs.ts - t0)
                for path, archetype in obs.data.to_rerun():
                    rr.log(path, archetype)

        # Clouds are logged in their stored coordinates (Point-LIO's cloud is
        # already registered into its world frame). Each stream is its own entity
        # so you can toggle them independently in the viewer.
        for name in clouds:
            entity = f"world/clouds/{name}"
            for obs in store.streams[name]:
                if not in_window(obs.ts):
                    break
                cloud = obs.data
                if cloud is None:
                    continue
                rr.set_time(TIMELINE, duration=obs.ts - t0)
                rr.log(entity, cloud.to_rerun(mode="points"))

        for name in odoms:
            trail: list[list[float]] = []
            for obs in store.streams[name]:
                if not in_window(obs.ts):
                    break
                odom = obs.data
                if odom is None:
                    continue
                rr.set_time(TIMELINE, duration=obs.ts - t0)
                # Moving pose frame (has its own transform)...
                rr.log(f"world/poses/{name}", odom.to_rerun())
                trail.append([odom.x, odom.y, odom.z])
            # ...and the full path as a static line in world coords (kept off the
            # posed entity so it isn't transformed by the pose).
            if trail:
                rr.log(f"world/trails/{name}", rr.LineStrips3D([trail]), static=True)

        rr.rerun_shutdown()
        print(f"wrote {rrd_path}")
        if not no_gui:
            _open_viewer(rrd_path)


if __name__ == "__main__":
    typer.run(main)
