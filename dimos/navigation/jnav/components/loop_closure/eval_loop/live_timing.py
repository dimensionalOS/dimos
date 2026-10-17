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

"""Per-frame cost of Ivan's LIVE module path (PGOMap at PGOVoxelMapper defaults) over a recording.

Mirrors PGOVoxelMapper._on_lidar: add(cloud, pose), then global_map() on a rebuild or every
emit_every=5 frames (what the unitree-go2-pgo blueprint runs).
usage: live_timing.py <name>   (reads $EVAL_LOOP_OUT/<name>.src.db) -> <name>.live_timing.json
"""

import json
import resource
import sys
import time

from paths import OUT as HERE, import_ivan

from dimos.memory.store.sqlite import SqliteStore

_module, _, _pgo_map = import_ivan()
PGOVoxelMapperConfig, PGOMap = _module.PGOVoxelMapperConfig, _pgo_map.PGOMap


def main():
    name = sys.argv[1]
    cfg = PGOVoxelMapperConfig(emit_every=5)  # what unitree-go2-pgo runs
    gmap = PGOMap(
        voxel_size=cfg.voxel_size,
        frame_id=cfg.frame_id,
        rebuild_cooldown_s=cfg.rebuild_cooldown_s,
        pgo=cfg.pgo,
    )
    pgo_ms = [0.0]
    process = gmap._pgo.process

    def timed_process(*args):
        started = time.perf_counter()
        process(*args)
        pgo_ms[0] = (time.perf_counter() - started) * 1e3

    gmap._pgo.process = timed_process
    rows = []
    with SqliteStore(path=str(HERE / f"{name}.src.db")) as store:
        for i, obs in enumerate(store.streams.lidar.order_by("ts")):
            pgo_ms[0] = 0.0
            started = time.perf_counter()
            rebuilt = gmap.add(obs.data, obs.pose_stamped)
            added = time.perf_counter()
            emit = rebuilt or (i + 1) % cfg.emit_every == 0
            cloud = gmap.global_map() if emit else None
            done = time.perf_counter()
            rows.append(
                [
                    obs.ts,
                    (added - started) * 1e3,
                    (done - added) * 1e3,
                    gmap.n_keyframes,
                    gmap.n_loops,
                    int(rebuilt),
                    len(cloud) if cloud is not None else -1,
                    pgo_ms[0],
                    resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
                    / 2**20,  # macOS: bytes -> MB
                ]
            )
            if i % 500 == 0:
                (HERE / f"{name}.live_timing.json").write_text(json.dumps(rows))
            if (done - started) > 30:
                print(name, i, f"STOP: one frame took {done - started:.0f}s", flush=True)
                break
            if i % 1000 == 0:
                print(
                    name,
                    i,
                    f"add {rows[-1][1]:.0f}ms map {rows[-1][2]:.0f}ms kf {gmap.n_keyframes} "
                    f"loops {gmap.n_loops} voxels {rows[-1][6]}",
                    flush=True,
                )
    gmap.dispose()
    (HERE / f"{name}.live_timing.json").write_text(json.dumps(rows))


if __name__ == "__main__":
    main()
