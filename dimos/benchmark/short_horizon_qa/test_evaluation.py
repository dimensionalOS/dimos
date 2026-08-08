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

from pathlib import Path
from types import SimpleNamespace

from dimos.benchmark.evaluation.protocol import AgentOutcome, EvaluationContext
import dimos.benchmark.short_horizon_qa.evaluation as evaluation
from dimos.benchmark.short_horizon_qa.models import (
    ExactIntegerValidatorRef,
    FrozenIntegerQaCase,
    FrozenIntegerQaConfig,
    FrozenRecordingSource,
    IntegerQuestionTask,
)


class FakeSession:
    def __init__(self, captured: dict) -> None:
        self.captured = captured

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def run(self, *, evaluation_protocol: str, task_input: str) -> AgentOutcome:
        self.captured.update(protocol=evaluation_protocol, task=task_input)
        return AgentOutcome("Checked\nANSWER: 2", 3, 1.0)


class FakeAgent:
    def __init__(self, captured: dict) -> None:
        self.captured = captured

    def open_session(self, environment):
        self.captured["environment"] = environment
        return FakeSession(self.captured)


def _case(tmp_path: Path) -> Path:
    private = tmp_path / "private"
    private.mkdir()
    (private / "oracle.json").write_text(
        '{"schema_version":"1.0","expected_count":2,'
        '"counting_policy":"count rooms","rooms":[],'
        '"reviewed_by":["reviewer"]}'
    )
    case = FrozenIntegerQaCase(
        case_id="demo",
        source=FrozenRecordingSource(recording="recording", progress=1.0),
        task=IntegerQuestionTask(prompt="How many rooms?"),
        validator=ExactIntegerValidatorRef(
            revision="v1",
            private_path="private/oracle.json",
        ),
    )
    path = tmp_path / "case.json"
    path.write_text(case.model_dump_json())
    return path


def test_frozen_evaluation_owns_protocol_decoder_and_openevals(
    monkeypatch,
    tmp_path: Path,
) -> None:
    case_path = _case(tmp_path)
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    captured = {}
    monkeypatch.setattr(evaluation, "_materialize_frozen_memory", lambda *_args: bundle)
    monkeypatch.setattr(
        evaluation,
        "load_bundle",
        lambda *_args, **_kwargs: (
            object(),
            SimpleNamespace(cutoff_timestamp=10.0),
            tmp_path / "source.db",
            tmp_path / "derived.db",
        ),
    )

    def exact_match(*, outputs, reference_outputs):
        captured.update(outputs=outputs, reference_outputs=reference_outputs)
        return {"key": "exact_match", "score": True, "comment": None}

    monkeypatch.setattr(evaluation, "exact_match", exact_match)
    context = EvaluationContext(
        run_id="run",
        spec_dir=tmp_path,
        workspace=tmp_path / "workspace",
        agent=FakeAgent(captured),
        progress=None,
    )

    report = evaluation.frozen_integer_qa.run(
        FrozenIntegerQaConfig(case=case_path.name),
        context,
    )

    assert captured["task"] == "How many rooms?"
    assert "ANSWER: <integer>" in captured["protocol"]
    assert captured["outputs"] == {"status": "parsed", "integer_answer": 2}
    assert captured["reference_outputs"] == {
        "status": "parsed",
        "integer_answer": 2,
    }
    assert report.native_result.value == {
        "key": "exact_match",
        "score": True,
        "comment": None,
    }
    assert [item.key for item in report.summary] == [
        "case",
        "recording",
        "answer",
        "exact_match",
        "tool_calls",
        "duration",
    ]


def test_materialize_accepts_bundle_published_by_concurrent_runner(
    monkeypatch,
    tmp_path: Path,
) -> None:
    recording = tmp_path / "recording.db"
    recording.touch()
    case = FrozenIntegerQaCase(
        case_id="concurrent",
        source=FrozenRecordingSource(recording=str(recording), progress=1.0),
        task=IntegerQuestionTask(prompt="How many rooms?"),
        validator=ExactIntegerValidatorRef(
            revision="v1",
            private_path="private/oracle.json",
        ),
    )
    monkeypatch.setattr(evaluation, "CACHE_DIR", tmp_path / "cache")
    monkeypatch.setattr(evaluation, "resolve_dataset", lambda _recording: recording)

    def publish_first(_recording, _cutoffs, output: Path, **_kwargs) -> None:
        output.mkdir()
        (output / "manifest.v1.json").write_text("{}")
        raise FileExistsError(output)

    monkeypatch.setattr(evaluation, "prepare_bundle", publish_first)
    context = EvaluationContext(
        run_id="run",
        spec_dir=tmp_path,
        workspace=tmp_path,
        agent=FakeAgent({}),
        progress=None,
    )

    bundle = evaluation._materialize_frozen_memory(case, context)

    assert bundle.joinpath("manifest.v1.json").is_file()
