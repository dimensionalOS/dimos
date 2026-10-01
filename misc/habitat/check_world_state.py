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

"""Build the world state once per task, at the spawn, from the scene's ground-truth boxes.

    python misc/habitat/check_world_state.py <scenes dir> [--ground-truth-dir DIR]

Exit 1 if any task's builder raises. Runs in seconds; run it before every scene set."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

from dimos.agents.typesafe.demo_objects import load_scene_objects
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.evals.suites.habitat_nav import TASK_BRIEF
from dimos.robot.raw_robot_bridge import dry_run_world_state


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", type=Path)
    ap.add_argument("--ground-truth-dir", type=Path, default=None)
    args = ap.parse_args()
    failed = 0
    for f in sorted(args.scenes.glob("*.json")):
        scene = json.loads(f.read_text())
        gt = (
            args.ground_truth_dir / Path(scene["ground_truth"]).name
            if args.ground_truth_dir
            else DIMOS_PROJECT_ROOT / scene["ground_truth"]
        )
        objects = load_scene_objects(gt)
        for c in scene["cases"]:
            gx, gy = c["end_xy"]
            goal = f"{TASK_BRIEF}\n\ngo to the {c['label']} at ({gx:.2f}, {gy:.2f})"
            try:
                doc = dry_run_world_state(goal, tuple(c["spawn_xyz"]), c["spawn_yaw_deg"], objects)
                print(f"ok    {scene['scene_id']}_{c['label']}  {len(doc)} chars")
            except Exception:
                failed += 1
                print(f"FAIL  {scene['scene_id']}_{c['label']}\n{traceback.format_exc()}")
    print(f"{failed} failures")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
