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

"""Export one real Go2 memory window through DimOS, independently of the harness."""

import hashlib
import json
import os
from pathlib import Path
import sys

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.vis.plot.elements import Series
from dimos.memory.vis.plot.plot import Plot


def main() -> None:
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    store = SqliteStore(path=os.environ["DIMCODE_TEST_GO2_DB"], must_exist=True)
    try:
        origin = store.streams.lidar.first().ts
        counts = []
        times = []
        report = {"source": os.environ["DIMCODE_TEST_GO2_DB"], "origin": origin, "interval": [1, 5]}
        for name, kind in [("lidar", "points"), ("color_image", "image")]:
            # Materialize once. Rendering reads these exports, never the database.
            selected = store.stream(name).time_range(origin + 1, origin + 5).materialize()
            frames = []
            for i, obs in enumerate(selected):
                path = out / f"{name}-{i:03d}.{'json' if kind == 'points' else 'png'}"
                if kind == "points":
                    xyz, rgb = obs.data.as_numpy()
                    data = {"points": xyz.tolist(), "frame": obs.data.frame_id, "timestamp": obs.ts}
                    if rgb is not None:
                        data["colors"] = (rgb * 255).round().clip(0, 255).astype(int).tolist()
                    path.write_text(json.dumps(data, separators=(",", ":")))
                    counts.append(len(xyz))
                    times.append(obs.ts)
                else:
                    obs.data.save(str(path))
                frames.append(
                    {
                        "path": path.name,
                        "timestamp": obs.ts,
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    }
                )
            assert frames, f"Missing {name} observations"
            index = {
                "type": kind,
                "timeOrigin": origin,
                "source": f"Go2 memory.db · {name} · time_range(origin+1, origin+5).materialize()",
                "frames": frames,
            }
            (out / f"{kind}.json").write_text(json.dumps(index, indent=2))
            report[kind] = {
                "count": len(frames),
                "first": frames[0]["timestamp"],
                "last": frames[-1]["timestamp"],
            }
        Plot().add(
            Series(ts=times, values=counts, label="source point count", color="#43bda7")
        ).to_svg(str(out / "point-count.svg"))
        (out / "query.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
    finally:
        store.stop()


if __name__ == "__main__":
    main()
