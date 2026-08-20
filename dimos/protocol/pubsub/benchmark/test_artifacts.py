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

from dimos.protocol.pubsub.benchmark.artifacts import (
    ArtifactWriter,
    load_samples,
    load_summaries,
    load_trial_summaries,
)
from dimos.protocol.pubsub.benchmark.matrix import build_matrix
from dimos.protocol.pubsub.benchmark.model import MessageSample, TrialRecord
from dimos.protocol.pubsub.benchmark.report import generate_report


def test_artifact_writer_preserves_samples_and_machine_readable_summaries(tmp_path) -> None:
    spec = build_matrix("smoke")[0]
    sample = MessageSample(spec.trial_id, "payload", 0, 4, 1024, 10, 20, 30, 40)
    record = TrialRecord(spec=spec, samples=[sample])
    writer = ArtifactWriter(tmp_path, {"schema_version": 1, "suite": "smoke"})

    summary = writer.write_trial(record)
    writer.finalize([summary], seed=7)

    assert load_samples(tmp_path / "samples" / f"{spec.trial_id}.jsonl.gz") == [sample]
    assert load_summaries(tmp_path) == [summary]
    assert load_trial_summaries(tmp_path) == [summary]
    assert json.loads((tmp_path / "aggregates.json").read_text())[0]["repetitions"] == 1
    assert json.loads((tmp_path / "comparisons.json").read_text()) == []
    assert json.loads((tmp_path / "manifest.json").read_text())["suite"] == "smoke"
    assert (tmp_path / "summary.csv").read_text().startswith("cohort,")

    report = generate_report(tmp_path)

    assert report.name == "report.html"
    assert "complete application stacks" in report.read_text()


def test_artifact_writer_resumes_matching_manifest_and_cleans_failed_trial(tmp_path) -> None:
    spec = build_matrix("smoke")[0]
    manifest = {
        "schema_version": 2,
        "suite": "smoke",
        "seed": 7,
        "trials": [spec.to_dict()],
    }
    writer = ArtifactWriter(tmp_path, manifest)
    writer.write_trial(TrialRecord(spec=spec, error="transient"))
    topology = writer.topology_dir / spec.trial_id
    topology.mkdir()
    (topology / "state.json").write_text("{}")
    (writer.logs_dir / f"{spec.trial_id}-publisher.log").write_text("failed")

    resumed = ArtifactWriter(tmp_path, manifest)
    resumed.prepare_retry(spec.trial_id)

    assert load_trial_summaries(tmp_path)[0]["error"] == "transient"
    assert not (writer.samples_dir / f"{spec.trial_id}.jsonl.gz").exists()
    assert not topology.exists()
    assert not list(writer.logs_dir.glob(f"{spec.trial_id}-*.log"))
