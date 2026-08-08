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

import pytest

from dimos.benchmark.agent_eval.models import (
    EvalCase,
    EvalRunConfig,
    ExactIntegerValidatorRef,
    FrozenRecordingSource,
    IntegerQuestionTask,
)
from dimos.benchmark.agent_eval.pi_process import PiRunResult
import dimos.benchmark.agent_eval.single_case as single_case
from dimos.benchmark.agent_eval.single_case import execute_single_case


def _case(tmp_path: Path) -> Path:
    private = tmp_path / "private"
    private.mkdir()
    (private / "oracle.json").write_text(
        '{"schema_version":"1.0","expected_count":2,'
        '"counting_policy":"count rooms","rooms":[],'
        '"reviewed_by":["reviewer"]}'
    )
    case = EvalCase(
        case_id="demo",
        source=FrozenRecordingSource(recording="recording", progress=1.0),
        task=IntegerQuestionTask(prompt="How many rooms?"),
        validator=ExactIntegerValidatorRef(revision="v1", private_path="private/oracle.json"),
    )
    path = tmp_path / "case.json"
    path.write_text(case.model_dump_json())
    return path


def test_direct_run_publishes_only_compact_result_and_native_transcript(
    monkeypatch, tmp_path: Path
) -> None:
    case_path = _case(tmp_path)
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    monkeypatch.setenv("OPENAI_API_KEY", "secret")
    monkeypatch.setattr(single_case, "_materialize_frozen_memory", lambda *_args: bundle)
    monkeypatch.setattr(
        single_case,
        "load_bundle",
        lambda *_args, **_kwargs: (
            object(),
            SimpleNamespace(cutoff_timestamp=10.0),
            tmp_path / "source.db",
            tmp_path / "derived.db",
        ),
    )
    monkeypatch.setattr(single_case, "_pi_paths", lambda: (case_path, case_path))

    class Server:
        mcp_url = "http://127.0.0.1:1234/mcp"
        session = SimpleNamespace(execution_count=3)

        def __init__(self, _config):
            pass

        def start(self):
            pass

        def stop(self):
            pass

    class Runner:
        def __init__(self, **_kwargs):
            pass

        def run(self, *, run_dir, **_kwargs):
            transcript = run_dir / "native.jsonl"
            transcript.write_text('{"type":"session"}\n')
            return PiRunResult("Checked\nANSWER: 2", 3, 1.0, transcript, "")

    monkeypatch.setattr(single_case, "CodePolicyMcpServer", Server)
    monkeypatch.setattr(single_case, "PiCliRunner", Runner)
    output = tmp_path / "output"
    result = execute_single_case(case_path, config=EvalRunConfig(), output=output)
    assert result.passed is True
    assert {path.name for path in output.iterdir()} == {
        "result.json",
        "pi-transcript.jsonl",
    }


def test_nonempty_output_is_rejected_before_execution(tmp_path: Path) -> None:
    output = tmp_path / "output"
    output.mkdir()
    (output / "keep").write_text("user data")
    with pytest.raises(FileExistsError, match="absent or an empty"):
        execute_single_case(_case(tmp_path), config=EvalRunConfig(), output=output)
    assert (output / "keep").read_text() == "user data"


def test_materialize_accepts_bundle_published_by_concurrent_runner(
    monkeypatch, tmp_path: Path
) -> None:
    recording = tmp_path / "recording.db"
    recording.touch()
    case = EvalCase(
        case_id="concurrent",
        source=FrozenRecordingSource(recording=str(recording), progress=1.0),
        task=IntegerQuestionTask(prompt="How many rooms?"),
        validator=ExactIntegerValidatorRef(revision="v1", private_path="private/oracle.json"),
    )
    monkeypatch.setattr(single_case, "CACHE_DIR", tmp_path / "cache")
    monkeypatch.setattr(single_case, "resolve_dataset", lambda _recording: recording)

    def publish_first(_recording, _cutoffs, output: Path, **_kwargs) -> None:
        output.mkdir()
        (output / "manifest.v1.json").write_text("{}")
        raise FileExistsError(output)

    monkeypatch.setattr(single_case, "prepare_bundle", publish_first)

    bundle = single_case._materialize_frozen_memory(case, None)

    assert bundle.joinpath("manifest.v1.json").is_file()
