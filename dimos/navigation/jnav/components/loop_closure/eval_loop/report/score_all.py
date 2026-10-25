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

"""Every number the report's summary and performance sections show, for each recording in the manifest and each
method: ground-truth error (score_gt.py), AprilTag spread (tag_agreement.py), loops, scans dropped, replay speed,
peak memory and CPU. Writes <out>.json. Run from the mls_reform worktree with its venv; tag_agreement.py needs the
scorer stubs (GSC_SCORE_STUBS, default ~/.cache/gsc_score_stubs) on PYTHONPATH, so it runs as a subprocess.
usage: score_all.py <manifest.json> <out.json>"""

import json
import os
from pathlib import Path
import re
import subprocess
import sys

here = Path(__file__).parent
scripts = Path(os.environ.get("GSC_SCRIPTS", here.parent / "gsc_vs_ivan")).expanduser()
stubs = os.environ.get("GSC_SCORE_STUBS", str(Path.home() / ".cache/gsc_score_stubs"))
manifest_path = Path(sys.argv[1]).expanduser()
manifest = json.loads(manifest_path.read_text())


# the lidar / odometry streams and tag frame per recording come from recordings.sh, as the runs used them
def recording_env(name):
    script = f'recording={name}; source "{scripts}/recordings.sh" >/dev/null 2>&1; echo "$db"; echo "${{eval_args[1]}}"'
    db, odom = subprocess.run(["bash", "-c", script], capture_output=True, text=True).stdout.split(
        "\n"
    )[:2]
    return db, odom


def last_json(text):
    lines = [l for l in text.strip().split("\n") if l.startswith("{") or " {" in l]
    if not lines:
        return None
    line = lines[-1]
    return json.JSONDecoder().raw_decode(line[line.index("{") :])[0]


def gsc_perf(run):
    log = (run / "time.log").read_text()
    out = {}
    m = re.search(
        r"(\d+) scans, (\d+) keyframes, (\d+) closures, ([\d.]+)s data in ([\d.]+)s wall", log
    )
    if m:
        out.update(
            scans=int(m[1]),
            keyframes=int(m[2]),
            data_s=float(m[4]),
            wall_s=float(m[5]),
            realtime=round(float(m[4]) / float(m[5]), 2),
        )
    m = re.search(r"([\d.]+) real\s+([\d.]+) user\s+([\d.]+) sys", log)
    if m:
        out["cpu_cores"] = round((float(m[2]) + float(m[3])) / float(m[1]), 2)
    m = re.search(r"(\d+)\s+maximum resident set size", log)
    if m:
        out["peak_mb"] = round(int(m[1]) / 1e6)
    a = json.loads((run / "a.json").read_text())
    out["loops"] = sum(1 for e in a["committed_closure_edges"] if e["gnc_weight"] >= 0.5)
    out["dropped"] = 0
    return out


def ivan_perf(run):
    s = json.loads((run / "ivan.json").read_text())
    return dict(
        scans=s["frames"] + s["dropped"],
        keyframes=s["keyframes"],
        loops=s["loops"],
        dropped=s["dropped"],
        dropped_pct=round(100 * s["dropped"] / max(1, s["frames"] + s["dropped"]), 1),
        busy_fraction=round(s["busy_s"] / s["wall_s"], 2),
        data_s=s["wall_s"],
    )


results = {}
for entry in manifest["recordings"]:
    name = entry["name"]
    db, odom = recording_env(name)
    row = {}
    for method in ("gsc", "ivan"):
        run = Path(entry[method]).expanduser()
        if not (run / "a.kf").exists():
            continue
        r = gsc_perf(run) if method == "gsc" else ivan_perf(run)
        gt = subprocess.run(
            [sys.executable, str(here / "score_gt.py"), str(manifest_path), name, str(run)],
            capture_output=True,
            text=True,
        )
        r["gt"] = last_json(gt.stdout) if entry.get("score_gt", True) else None
        env = dict(os.environ, PYTHONPATH=stubs)
        # the odometry's body frame and camera frame the tag sightings chain through (manifest "tags", else the Go2 / Mid-360 defaults)
        tag_frames = entry.get("tags", {})
        odom_frame = tag_frames.get(
            "odom_frame", "l1_link" if odom == "go2_odometry" else "mid360_link"
        )
        camera = (
            ["--camera-frame", tag_frames["camera_frame"]] if "camera_frame" in tag_frames else []
        )
        tags = subprocess.run(
            [
                sys.executable,
                str(scripts / "tag_agreement.py"),
                str(run),
                db,
                odom,
                odom_frame,
                *camera,
            ],
            capture_output=True,
            text=True,
            env=env,
        )
        r["tags"] = last_json(tags.stdout)
        m = re.search(r"tags revisited: (\d+)", tags.stdout)
        if m and r["tags"]:
            r["tags"]["revisited"] = int(m[1])
        row[method] = r
        print(name, method, json.dumps(r)[:300], flush=True)
    results[name] = row
Path(sys.argv[2]).expanduser().write_text(json.dumps(results, indent=1))
