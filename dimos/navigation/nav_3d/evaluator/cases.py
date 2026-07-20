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

"""Case manifests for the nav-3d evaluator.

A suite is one YAML file per dataset under cases/. Start and goal are
foot-level world coordinates, the frame the planner consumes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from dimos.utils.data import resolve_named_path

CASES_DIR = Path(__file__).parent / "cases"


@dataclass
class Case:
    id: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    tags: list[str] = field(default_factory=list)
    # If the pair should not have a valid path, this is assigned by humans
    expect_fail: bool = False
    # Some routes are passable with the incremental map but not passable in the final.
    # For example a door is open, robot walks through, then it gets closed.
    expect_final_fail: bool = False


@dataclass
class Suite:
    dataset: str
    cases: list[Case]
    lidar_stream: str = "pointlio_lidar"
    odom_stream: str = "pointlio_odometry"
    # Recording location override, defaulting to data/<dataset>.db.
    # Set this to keep a recording outside data/.
    db: str | None = None
    path: Path | None = None

    def db_path(self) -> Path:
        if self.db is not None:
            return Path(self.db).expanduser()
        return resolve_named_path(self.dataset, ".db")


def load_suite(path: Path) -> Suite:
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict) or "dataset" not in raw or "cases" not in raw:
        raise ValueError(f"{path}: suite needs 'dataset' and 'cases' keys")
    cases = []
    seen: set[str] = set()
    for entry in raw["cases"]:
        if len(entry["start"]) != 3 or len(entry["goal"]) != 3:
            raise ValueError(f"{path}: case {entry['id']}: start/goal must be xyz")
        sx, sy, sz = (float(v) for v in entry["start"])
        gx, gy, gz = (float(v) for v in entry["goal"])
        expect_fail = bool(entry.get("expect_fail", False))
        expect_final_fail = bool(entry.get("expect_final_fail", False))
        if expect_fail and expect_final_fail:
            raise ValueError(
                f"{path}: case {entry['id']}: expect_fail and expect_final_fail are exclusive"
            )
        case = Case(
            id=str(entry["id"]),
            start=(sx, sy, sz),
            goal=(gx, gy, gz),
            tags=[str(t) for t in entry.get("tags", [])],
            expect_fail=expect_fail,
            expect_final_fail=expect_final_fail,
        )
        if case.id in seen:
            raise ValueError(f"{path}: duplicate case id {case.id}")
        seen.add(case.id)
        cases.append(case)
    return Suite(
        dataset=str(raw["dataset"]),
        cases=cases,
        lidar_stream=str(raw.get("lidar_stream", "pointlio_lidar")),
        odom_stream=str(raw.get("odom_stream", "pointlio_odometry")),
        db=str(raw["db"]) if "db" in raw else None,
        path=path,
    )


def load_suites(paths: list[Path] | None = None) -> list[Suite]:
    """Load the given manifests, or every manifest under cases/."""
    if paths is None:
        paths = sorted(CASES_DIR.glob("*.yaml"))
    if not paths:
        raise FileNotFoundError(f"no case manifests found under {CASES_DIR}")
    return [load_suite(p) for p in paths]


def save_suite(suite: Suite, path: Path | None = None) -> Path:
    """Write the suite manifest as YAML. Defaults to cases/<dataset>.yaml."""
    path = path or suite.path or CASES_DIR / f"{suite.dataset}.yaml"
    doc: dict[str, object] = {"dataset": suite.dataset}
    if suite.db is not None:
        doc["db"] = suite.db
    if suite.lidar_stream != "pointlio_lidar":
        doc["lidar_stream"] = suite.lidar_stream
    if suite.odom_stream != "pointlio_odometry":
        doc["odom_stream"] = suite.odom_stream
    entries = []
    for case in suite.cases:
        entry: dict[str, object] = {
            "id": case.id,
            "start": [round(float(v), 3) for v in case.start],
            "goal": [round(float(v), 3) for v in case.goal],
            "tags": case.tags,
        }
        if case.expect_fail:
            entry["expect_fail"] = True
        if case.expect_final_fail:
            entry["expect_final_fail"] = True
        entries.append(entry)
    doc["cases"] = entries
    path.write_text(yaml.safe_dump(doc, sort_keys=False, default_flow_style=None))
    return path
