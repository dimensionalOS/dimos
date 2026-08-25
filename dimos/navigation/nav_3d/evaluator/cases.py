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

"""Case manifests, one YAML per dataset. Endpoints are foot-level world coords."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.navigation.nav_3d.evaluator.recording import Frame, Trajectory

MANIFEST_DIR = Path(__file__).parent / "case_manifests"

# Default mem2 streams a recording is read from.
LIDAR_STREAM = "pointlio_lidar"
ODOM_STREAM = "pointlio_odometry"


def manifest_path(dataset: str) -> Path:
    """Where a dataset's case manifest lives."""
    return MANIFEST_DIR / f"{dataset}.yaml"


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
    lidar_stream: str = LIDAR_STREAM
    odom_stream: str = ODOM_STREAM
    path: Path | None = None

    def db_path(self) -> Path:
        return resolve_named_path(self.dataset, ".db")

    def trajectory(self) -> Trajectory:
        """The walked path this suite's cases are scored against."""
        from dimos.navigation.nav_3d.evaluator.recording import load_trajectory

        return load_trajectory(self.db_path(), self.odom_stream)

    def frame_count(self) -> int:
        """Upper bound on the frames world_frames yields, for a progress bar."""
        from dimos.memory.store.sqlite import SqliteStore
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        store = SqliteStore(path=str(self.db_path()))
        with store:
            return store.stream(self.lidar_stream, PointCloud2).count()

    def world_frames(self, align_tol: float) -> Iterator[Frame]:
        """Lidar frames registered into the world by their odometry pose."""
        from dimos.navigation.nav_3d.evaluator.recording import iter_world_frames

        return iter_world_frames(self.db_path(), self.lidar_stream, self.odom_stream, align_tol)


def load_suite(path: Path) -> Suite:
    # Lazy: pyyaml is not in the base install, and mounting the CLI must not
    # require it until a manifest is actually read.
    import yaml

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
        lidar_stream=str(raw.get("lidar_stream", LIDAR_STREAM)),
        odom_stream=str(raw.get("odom_stream", ODOM_STREAM)),
        path=path,
    )


def load_suites(paths: list[Path] | None = None) -> list[Suite]:
    """Load the given manifests, or every manifest under case_manifests/."""
    if paths is None:
        paths = sorted(MANIFEST_DIR.glob("*.yaml"))
    if not paths:
        raise FileNotFoundError(f"no case manifests found under {MANIFEST_DIR}")
    return [load_suite(p) for p in paths]


def save_suite(suite: Suite, path: Path) -> Path:
    """Write the suite manifest as YAML."""
    import yaml

    if path.is_relative_to(MANIFEST_DIR) and not (DIMOS_PROJECT_ROOT / ".git").exists():
        raise RuntimeError(
            f"case manifests live in the dimos source tree ({MANIFEST_DIR}); "
            "run ingest and curation from a git checkout"
        )

    doc: dict[str, object] = {"dataset": suite.dataset}
    if suite.lidar_stream != LIDAR_STREAM:
        doc["lidar_stream"] = suite.lidar_stream
    if suite.odom_stream != ODOM_STREAM:
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
