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

"""Replay benchmark.json recordings through pgo-eval in deterministic mode at the module defaults (the R1 Jetson's
deterministic_config), so any machine reproduces the same graph: a slower machine only waits longer.

Each recording's db is used from its `db` path when that exists, else pulled from the dimos cloud by its `cloud`
upload id into $EVAL_LOOP_DATA (`dimos login` once first). pgo-eval is built when $PGO_EVAL is missing.

usage: deterministic_battery.py [--cloud-only] [--tier main|do_no_harm|extra] [name ...]   (no names and no tier: the 9-map battery)
    -> $EVAL_LOOP_OUT/deterministic/<name>/{a.json, a.kf*, a.log, run.log, summary.json} and summary.json for all
"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

from paths import OUT, PGO_EVAL
from recordings import CACHE

BENCHMARK = Path(__file__).with_name("benchmark.json")
CRATE = Path(__file__).resolve().parents[1] / "gsc_pgo/rust"
BATTERY = [
    "small_l1",
    "small_fastlio",
    "huge_fastlio",
    "huge_l1",
    "gir1_pointlio",
    "gir2_pointlio",
    "gir_park_pointlio",
    "office555",
    "athens_pointlio",
]


def summarize(log: str) -> dict:
    """The run's speed, closures and overshoots from pgo-eval's stderr."""
    summary: dict = {}
    if match := re.search(r"(\d+) scans, (\d+) keyframes, (\d+) closures, ([\d.]+)s data in ([\d.]+)s", log):
        scans, keyframes, closures, data_s, wall_s = match.groups()
        summary |= {"scans": int(scans), "keyframes": int(keyframes), "closures": int(closures)}
        summary |= {"data_s": float(data_s), "wall_s": float(wall_s), "x_real": float(data_s) / float(wall_s)}
    if match := re.search(r"overshoots per LET \(jobs\): (.*)", log):
        summary["overshoots"] = {
            job: [int(over), int(total)] for job, over, total in re.findall(r"(\w+) (\d+) / (\d+)", match.group(1))
        }
    return summary


def build_pgo_eval() -> None:
    """Build pgo-eval where $PGO_EVAL points (the crate's target/ by default), with cargo or the crate's nix flake."""
    if PGO_EVAL.is_file():
        return
    cargo = ["cargo", "build", "--release", "--features", "native", "--bin", "pgo-eval"]
    if not shutil.which("cargo"):
        cargo = ["nix", "develop", "path:.", "--command", *cargo]
    print(f"building pgo-eval: {' '.join(cargo)}", file=sys.stderr)
    subprocess.run(cargo, cwd=CRATE, check=True)
    # the crate sits in the repo's cargo workspace, whose target dir is not the crate's own
    metadata = subprocess.run(
        [*cargo[: cargo.index("cargo")], "cargo", "metadata", "--format-version", "1", "--no-deps"],
        cwd=CRATE,
        check=True,
        capture_output=True,
        text=True,
    )
    built = Path(json.loads(metadata.stdout)["target_directory"]) / "release/pgo-eval"
    if built != PGO_EVAL:
        PGO_EVAL.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(built, PGO_EVAL)


def dataset(recording: dict, cloud_only: bool) -> Path | None:
    """The recording's db: its local `db` path, else its `cloud` upload pulled into the cache once."""
    local = Path(os.path.expanduser(recording.get("db", "")))
    if local.is_file() and not cloud_only:
        return local
    if not recording.get("cloud"):
        return None
    out = CACHE / f"{recording['cloud']}.db"
    if not out.is_file():
        from dimos.cloud.data import CloudData

        print(f"{recording['name']}: downloading {recording['cloud']} -> {out}", file=sys.stderr, flush=True)
        CloudData().pull(recording["cloud"], out)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("names", nargs="*")
    parser.add_argument("--tier")
    parser.add_argument("--cloud-only", action="store_true", help="ignore local db paths, use the cloud uploads")
    args = parser.parse_args()
    recordings = {}
    for recording in json.loads(BENCHMARK.read_text())["recordings"]:
        # a recording in several dbs (hk_village) runs each part as <name>/<part>
        for part in recording.get("parts", [{}]):
            entry = recording | part
            entry["name"] = f"{recording['name']}/{part['name']}" if part else recording["name"]
            recordings[entry["name"]] = entry
    if args.tier:
        names = [name for name, r in recordings.items() if r["tier"] == args.tier]
    else:
        names = [name for name in recordings for wanted in args.names or BATTERY if name.split("/")[0] == wanted]
    build_pgo_eval()
    (OUT / "deterministic").mkdir(parents=True, exist_ok=True)
    summaries = {}
    for name in names:
        recording = recordings[name]
        db = None if recording.get("status") == "todo" else dataset(recording, args.cloud_only)
        if db is None:
            print(f"{name}: skipped (status {recording.get('status')}, no db or cloud upload)", file=sys.stderr)
            continue
        out = OUT / "deterministic" / name
        out.mkdir(parents=True, exist_ok=True)
        # mounts come from the db's tf; deterministic mode at the module defaults
        command = [str(PGO_EVAL), "--db", str(db), "--odom-stream", recording["odometry"]]
        command += ["--lidar-stream", recording["lidar"], "--mode", "deterministic"]
        command += ["--output", "a.json", "--event-dump", "a.kf"]
        print(f"{name}: {' '.join(command)}", file=sys.stderr)
        with open(out / "a.log", "w") as stdout:
            result = subprocess.run(command, cwd=out, stdout=stdout, stderr=subprocess.PIPE, text=True)
        (out / "run.log").write_text(result.stderr)
        summaries[name] = summarize(result.stderr) | {"returncode": result.returncode, "command": command}
        (out / "summary.json").write_text(json.dumps(summaries[name], indent=2) + "\n")
        print(f"{name}: {json.dumps({k: v for k, v in summaries[name].items() if k != 'command'})}", file=sys.stderr)
    (OUT / "deterministic" / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")


if __name__ == "__main__":
    main()
