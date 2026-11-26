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

"""On-disk representations, including sessions written by the original arm experiment."""

import json
from pathlib import Path
import tempfile
from typing import Any


def write_text(path: Path, text: str) -> None:
    temp = tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False)
    partial = Path(temp.name)
    try:
        with temp:
            temp.write(text)
        partial.replace(path)
    finally:
        partial.unlink(missing_ok=True)


def read_manifest(directory: Path) -> dict[str, Any]:
    data: dict[str, Any] = json.loads((directory / "session.json").read_text())
    if "experiment" not in data:  # Original on-disk arm sessions.
        data.update(
            experiment="mujoco",
            metric="pass rate",
            settings={
                k: data[k]
                for k in ("blueprint", "scene_source", "success_rule", "dimos_commit")
                if k in data
            },
        )
        old_budget = data["budget"]
        data["settings"]["rounds"] = dict(old_budget)
        old_budget["max_candidates"] = old_budget.get("iterations", 5) * old_budget.get(
            "candidates_per_round", 3
        )
        frozen = directory / "frozen"
        if frozen.is_file():
            data["frozen_candidate"] = int(frozen.read_text())
    return data


def read_trials(directory: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = json.loads((directory / "results.json").read_text())
    for record in records:
        if "score" not in record:  # Original binary verdict and flat evidence.
            record["score"] = float(record.pop("success"))
            record["evidence"] = {
                k: record.pop(k)
                for k in list(record)
                if k not in ("score", "split", "index", "note")
            }
    return records


def artifact_path(directory: Path) -> Path:
    name = directory / "artifact-name"
    if name.is_file():
        return directory / "artifact" / name.read_text()
    return directory / "program.py"  # Original candidate layout.
