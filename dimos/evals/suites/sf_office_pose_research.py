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

"""PoseStamped autoresearch cases loaded from the reviewable QA manifest."""

import json
import os
from pathlib import Path
from typing import Any

from dimos.evals.environments.dataset import Dataset
from dimos.evals.suites.sf_office_pose_grading import grader
from dimos.evals.types import EvalCase, Suite

RECORDING_PATH = Path(
    os.environ.get(
        "SF_OFFICE_POSE_RECORDING",
        Path.home() / "Documents/worktrees/agentencode/recording_go2.db",
    )
)
EXPECTED_RECORDING_SHA256 = "75dab75f22fb19b730ca43121d7601a0460ff7cea533de8f3f9a8d7b88f55fe8"
EXPECTED_POSE_COUNT = 8568
_TIMEOUT_S = 180.0
_QA_MANIFEST: dict[str, Any] = json.loads(
    Path(__file__).with_name("sf_office_pose_qa.json").read_text()
)
_INSTRUCTIONS = " ".join(_QA_MANIFEST["instructions"])
_QA_RECORDS: list[dict[str, Any]] = _QA_MANIFEST["cases"]


def _poses() -> Dataset:
    return Dataset(str(RECORDING_PATH), select=(lambda store: store.streams.odom,))


SUITE: Suite = [
    EvalCase(
        id=record["id"],
        inputs=" ".join([_INSTRUCTIONS, *record["question"]]),
        environment=_poses(),
        grade=grader(record["id"]),
        timeout_s=_TIMEOUT_S,
        tags=frozenset(record["tags"]),
    )
    for record in _QA_RECORDS
]
