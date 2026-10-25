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

"""Build the GSC-vs-Ivan PGO report folder: one export per manifest recording, plus index.html (viewer.html) and report.json.
usage: build_report.py <manifest.json> <out dir> [--only NAME ...]   (any cwd; runs the exports niced from the dimos worktree)
Fails if any file is >= 15 MB or the folder is >= 60 MB."""

import argparse
import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
WORKTREE = Path(
    os.environ.get("GSC_WORKTREE", Path.home() / "repos/_mls_reform_wt")
)  # dimos checkout the scorers run from
parser = argparse.ArgumentParser()
parser.add_argument("manifest", type=Path)
parser.add_argument("out", type=Path)
parser.add_argument(
    "--only",
    nargs="*",
    default=None,
    help="re-export only these recordings (others keep their old export)",
)
args = parser.parse_args()
args.out = args.out.resolve()
manifest = json.loads(args.manifest.read_text())
args.out.mkdir(parents=True, exist_ok=True)
env = {
    **os.environ,
    "PYTHONPATH": os.environ.get("GSC_SCORE_STUBS", str(Path.home() / ".cache/gsc_score_stubs")),
}
for entry in manifest["recordings"]:
    if args.only is not None and entry["name"] not in args.only:
        continue
    subprocess.run(
        [
            "nice",
            "-n",
            "10",
            str(WORKTREE / ".venv/bin/python"),
            str(HERE / "export_recording.py"),
            json.dumps(entry),
            str(args.out),
        ],
        cwd=WORKTREE,
        env=env,
        check=True,
    )
shutil.copy(HERE / "viewer.html", args.out / "index.html")
report = dict(
    title=manifest.get("title", "GSC vs Ivan PGO"),
    built=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
    recordings=[
        dict(name=e["name"], label=e.get("label", e["name"])) for e in manifest["recordings"]
    ],
)
(args.out / "report.json").write_text(json.dumps(report, indent=1))
files = [p for p in args.out.rglob("*") if p.is_file()]
total = sum(p.stat().st_size for p in files)
big = [(p, p.stat().st_size) for p in files if p.stat().st_size >= 15e6]
for name in [e["name"] for e in manifest["recordings"]]:
    print(
        f"  {name}: {sum(p.stat().st_size for p in (args.out / name).rglob('*') if p.is_file()) / 1e6:.1f} MB"
    )
print(f"{args.out}: {len(files)} files, {total / 1e6:.1f} MB")
if big or total >= 60e6:
    sys.exit(
        f"over the size limits: {[(str(p), s) for p, s in big]} total {total / 1e6:.1f} MB — lower options (max_points, icp_*, state_budget_bytes)"
    )
