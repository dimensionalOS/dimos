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

"""Synthetic pointcloud scale/frame QA. Prepare, then run with the existing runner::

    python -m dimos.evals.suites.pointcloud.dataset.pointcloud_scale
    dimos evals run dimos.evals.suites.pointcloud.dataset.pointcloud_scale \
        --agent dimos.evals.agents.question_answer

The 35 questions use 13 deterministic clouds: two layouts at three scales,
three frame/translation variants, and two overview/crop pairs. All are visible
development cases. Numeric answers have 1% feature-relative tolerances; absolute
coordinate questions use 1% of the local extent, not of the coordinate offset.
The overview gap cases deliberately ask about a local region in the full,
uncropped cloud to probe capability limits. Their crop counterparts supply
only that region; both ask for the same local measurement.
Only points/frame/timestamp enter the recording. Labels and fixture bounds are
not agent observations. No encoder, query layer, or harness changes are needed.
"""

from __future__ import annotations

from functools import partial
from pathlib import Path

from dimos.evals.environments.dataset import Dataset
from dimos.evals.suites.pointcloud.lib.scale import (
    DATA_DIR,
    GeometryQuestion,
    cloud_specs,
    parse_number,
    prepare_recordings,
    questions,
    score_number,
)
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.memory.store.base import Store
from dimos.memory.stream import Stream
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def select_cloud(store: Store) -> Stream[PointCloud2]:
    return store.stream("pointcloud", PointCloud2)


def grade_question(question: GeometryQuestion, outcome: Outcome) -> float:
    return score_number(
        question.expected,
        parse_number(outcome.trajectory.final_answer),
        tolerance=question.tolerance,
    )


def build_suite(directory: Path = DATA_DIR) -> list[EvalCase]:
    return [
        EvalCase(
            id=q.id,
            inputs=q.inputs,
            environment=Dataset(str(q.cloud.path(directory)), select=(select_cloud,)),
            grade=partial(grade_question, q),
            tags=frozenset(
                {"pointcloud", "synthetic", "development", q.cloud.category, q.quantity}
            ),
        )
        for q in questions()
    ]


SUITE: Suite = build_suite()


if __name__ == "__main__":
    prepare_recordings()
    print(f"Prepared {len(cloud_specs())} clouds for {len(SUITE)} questions in {DATA_DIR}")
