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

"""Run an exported skill's program once on the DimOS stack already running on
this machine's bus. ``export_skill`` copies this file to ``scripts/run.py``;
run it with the DimOS environment's Python. It prints how the program ended
and where its trace and keyframes are."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tempfile

from dimos.autoresearch.program import TRACE_NAME, run_program

LIVE_BACKEND = "dimos.autoresearch.live:backend"
# The lab's default program timeout (AutoresearchModuleConfig.program_timeout_s), seconds.
TIMEOUT_S = 120.0


def main() -> None:
    skill = Path(__file__).resolve().parent.parent
    validation = json.loads((skill / "assets" / "validation.json").read_text())
    parser = argparse.ArgumentParser(description="Run this skill's program on the running stack.")
    parser.add_argument("--task", default=validation["task"], help="the task in words")
    parser.add_argument("--out", type=Path, help="trace and keyframe folder (default: a temp dir)")
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S, help="seconds before a kill")
    args = parser.parse_args()
    out: Path = args.out or Path(tempfile.mkdtemp(prefix=f"{skill.name}-"))
    outcome = run_program(
        skill / "scripts" / "program.py", out, args.task, args.timeout, LIVE_BACKEND
    )
    print(f"outcome: {outcome.status} after {outcome.duration_s:.1f} s")
    if outcome.error:
        print(outcome.error)
    print(f"trace: {out / TRACE_NAME}")
    print(f"keyframes: {out / 'keyframes'}")
    sys.exit(0 if outcome.status == "completed" else 1)


if __name__ == "__main__":
    main()
