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

"""Manifest → EvalCase for benchmark MJCFs under ``data/``."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import json
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, Field

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.utils.data import get_data_dir

GradeKind = Literal["lifted"]

# Same stack as ``mujoco_xarm`` — perception modules off for planner/MCP evals.
XARM_EVAL_BLUEPRINT = ["xarm-perception-sim", "mcp-server", "observe-skill"]
XARM_EVAL_DISABLE = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)


class GradeSpec(BaseModel):
    type: GradeKind
    body: str
    by_m: float = 0.05


class BenchmarkCase(BaseModel):
    id: str
    language: str
    scene: str
    tracked_bodies: tuple[str, ...] = ()
    grade: GradeSpec
    timeout_s: float = 600.0
    tags: tuple[str, ...] = ()


class BenchmarkManifest(BaseModel):
    source: str
    root: str
    cases: list[BenchmarkCase] = Field(default_factory=list)


def load_manifest(root: str) -> BenchmarkManifest:
    path = get_data_dir() / root / "manifest.json"
    return BenchmarkManifest.model_validate(json.loads(path.read_text()))


def scene_path(root: str, relative: str) -> Path:
    return (get_data_dir() / root / relative).resolve()


def xarm_table_env(scene: Path, tracked_bodies: tuple[str, ...] = ()) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=XARM_EVAL_BLUEPRINT,
        disable=XARM_EVAL_DISABLE,
        scene=scene,
        tracked_bodies=tracked_bodies,
    )


def grade_from_spec(spec: GradeSpec) -> Callable[[Outcome], float]:
    if spec.type != "lifted":
        raise ValueError(f"unsupported grade type: {spec.type}")

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, spec.body).translation.z
                end = last_body_transform(store, spec.body).translation.z
            except LookupError:
                return 0.0
        return min(max((end - start) / spec.by_m, 0.0), 1.0)

    return grade


def cases_from_manifest(
    manifest: BenchmarkManifest,
    *,
    environment_factory: Callable[[BenchmarkCase, Path], Any] | None = None,
    extra_tags: Sequence[str] = (),
) -> Suite:
    make_env = environment_factory or (
        lambda case, scene: xarm_table_env(scene, case.tracked_bodies)
    )
    suite: Suite = []
    for case in manifest.cases:
        scene = scene_path(manifest.root, case.scene)
        suite.append(
            EvalCase(
                id=case.id,
                inputs=case.language,
                environment=make_env(case, scene),
                grade=grade_from_spec(case.grade),
                timeout_s=case.timeout_s,
                tags=frozenset({*case.tags, *extra_tags, manifest.source.lower()}),
            )
        )
    return suite


def write_manifest(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")
