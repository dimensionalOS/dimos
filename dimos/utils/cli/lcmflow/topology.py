# Copyright 2025-2026 Dimensional Inc.
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

"""Stream-direction sidecar for the lcmflow dashboard.

The constellation dashboard reads which module publishes/subscribes which
topic from the coordinator's structured ``Transport`` log — but that log
does not record stream direction. Rather than change the coordinator just
for a viz, we recover direction here by introspecting the blueprint's
``In[T]`` / ``Out[T]`` stream declarations (this runs in the dimos venv,
the deno server does not).

Output (stdout): a JSON object mapping ``"<ModuleClass>|<stream_name>"``
to ``"in"`` or ``"out"``, e.g.

    {"GO2Connection|odom": "out", "ReplanningAStarPlanner|odom": "in"}

The deno server matches each Transport log entry on
``(module, original_name)`` to fill in arrow direction. On any failure
(no run, unknown blueprint, heavy import error) it prints ``{}`` and
exits 0 — the dashboard simply falls back to undirected edges.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys


def _running_blueprints() -> list[str]:
    """Blueprint name(s) of the most recent run in the registry."""
    runs_dir = Path.home() / ".local" / "state" / "dimos" / "runs"
    try:
        entries = sorted(runs_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    except OSError:
        return []
    for path in entries:
        try:
            rec = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        # `cli_args` holds the positional blueprint name(s); `blueprint` is a
        # single resolved name. Prefer cli_args (supports composed stacks).
        names = rec.get("cli_args") or ([rec["blueprint"]] if rec.get("blueprint") else [])
        names = [n for n in names if isinstance(n, str) and not n.startswith("-")]
        if names:
            return names
    return []


def directions_for(blueprint_names: list[str]) -> dict[str, str]:
    """Map "<ModuleClass>|<stream_name>" -> "in"/"out" for the given blueprints."""
    from dimos.robot.get_all_blueprints import get_by_name

    out: dict[str, str] = {}
    for name in blueprint_names:
        try:
            blueprint = get_by_name(name)
        except Exception:
            continue
        for bp in blueprint.active_blueprints:
            for conn in bp.streams:
                out[f"{bp.module.__name__}|{conn.name}"] = conn.direction
    return out


def main() -> None:
    # Blueprint import logs go to stderr; only JSON reaches stdout.
    names = sys.argv[1:] or _running_blueprints()
    try:
        result = directions_for(names) if names else {}
    except Exception:
        result = {}
    json.dump(result, sys.stdout)


if __name__ == "__main__":
    main()
