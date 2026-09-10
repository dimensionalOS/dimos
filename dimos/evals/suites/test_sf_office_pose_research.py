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

import json
from pathlib import Path

from dimos.evals.suites.sf_office_pose_research import SUITE


def test_pose_research_suite_matches_reference_manifest() -> None:
    manifest = json.loads(Path(__file__).with_name("sf_office_pose_qa.json").read_text())
    records = manifest["cases"]

    assert len(SUITE) == 8
    assert {case.id for case in SUITE} == {record["id"] for record in records}
    assert len({case.id for case in SUITE}) == len(SUITE)
    assert all(set(record) == {"id", "question", "answer", "tags"} for record in records)
    assert all(
        case.inputs == " ".join([*manifest["instructions"], *record["question"]])
        for case, record in zip(SUITE, records, strict=True)
    )


def test_review_manifest_matches_runtime_answers_and_tags() -> None:
    runtime = json.loads(Path(__file__).with_name("sf_office_pose_qa.json").read_text())["cases"]
    review = json.loads(Path(__file__).with_name("sf_office_pose_qa_review.json").read_text())

    assert [record["id"] for record in review] == [record["id"] for record in runtime]
    assert all(set(record) == {"id", "question", "answer", "tags"} for record in review)
    assert all(record["question"] for record in review)
    assert all(
        review_record["answer"] == runtime_record["answer"]
        and review_record["tags"] == runtime_record["tags"]
        for review_record, runtime_record in zip(review, runtime, strict=True)
    )


def test_pose_research_cases_enforce_encoded_odom_evidence() -> None:
    for case in SUITE:
        assert "odom PoseStamped" in case.inputs
        assert "obs.data.agent_encode()" in case.inputs
        assert "sole evidence" in case.inputs
        assert "preprocess_encoded_poses" in case.inputs
        assert case.timeout_s == 180.0
        assert "autoresearch" in case.tags
