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

from dimos.agents.code_policy_core import EmptyEnvironment
from dimos.benchmark.agent_eval.models import (
    EvalCase,
    EvalRunConfig,
    ExactIntegerValidatorRef,
    ExternalEvaluatorRef,
    FrozenRecordingSource,
    IntegerQuestionTask,
    NoEnvironmentSource,
    VerbatimPromptTask,
)
from dimos.benchmark.agent_eval.pi_process import PiRunError, PiRunResult
import dimos.benchmark.agent_eval.single_case as single_case
from dimos.benchmark.agent_eval.single_case import VERBATIM_SYSTEM_PROMPT, execute_single_case

VERBATIM_PROMPT = "Which room is the robot in? Reply with the room name and nothing else."


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


def _environment_free_case(tmp_path: Path) -> Path:
    """A case wired from the kinds that need neither a recording nor a local oracle."""
    case = EvalCase(
        case_id="verbatim",
        source=NoEnvironmentSource(),
        task=VerbatimPromptTask(prompt=VERBATIM_PROMPT),
        validator=ExternalEvaluatorRef(benchmark="space", revision="abc123"),
    )
    path = tmp_path / "case.json"
    path.write_text(case.model_dump_json())
    return path


def _never_materialize(*_args: object) -> Path:
    raise AssertionError("a case without a recording must not materialize one")


def _fake_server(environments: list[object]) -> type:
    """A server stand-in that records the environment it was configured with."""

    class Server:
        mcp_url = "http://127.0.0.1:1234/mcp"
        session = SimpleNamespace(execution_count=1)

        def __init__(self, config):
            environments.append(config.environment)

        def start(self):
            pass

        def stop(self):
            pass

    return Server


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


def test_case_without_an_environment_runs_the_prompt_verbatim_and_defers_scoring(
    monkeypatch, tmp_path: Path
) -> None:
    case_path = _environment_free_case(tmp_path)
    monkeypatch.setenv("OPENAI_API_KEY", "secret")
    monkeypatch.setattr(single_case, "_materialize_frozen_memory", _never_materialize)
    monkeypatch.setattr(single_case, "_pi_paths", lambda: (case_path, case_path))
    environments: list[object] = []
    turns: dict[str, str] = {}

    class Runner:
        def __init__(self, **_kwargs):
            pass

        def run(self, *, prompt, system_prompt, run_dir, **_kwargs):
            turns["prompt"] = prompt
            turns["system_prompt"] = system_prompt
            transcript = run_dir / "native.jsonl"
            transcript.write_text('{"type":"session"}\n')
            return PiRunResult("Kitchen", 1, 1.0, transcript, "")

    monkeypatch.setattr(single_case, "CodePolicyMcpServer", _fake_server(environments))
    monkeypatch.setattr(single_case, "PiCliRunner", Runner)
    output = tmp_path / "output"
    result = execute_single_case(case_path, config=EvalRunConfig(), output=output)

    assert turns["prompt"] == VERBATIM_PROMPT
    assert turns["system_prompt"] == VERBATIM_SYSTEM_PROMPT
    assert isinstance(environments[0], EmptyEnvironment)
    assert result.final_response == "Kitchen"
    assert result.prediction_status == "not_evaluated"
    assert result.integer_answer is None
    assert result.passed is None
    assert result.recording is None
    assert result.progress is None
    assert {path.name for path in output.iterdir()} == {
        "result.json",
        "pi-transcript.jsonl",
    }


def test_case_without_an_environment_reports_the_original_failure(
    monkeypatch, tmp_path: Path
) -> None:
    case_path = _environment_free_case(tmp_path)
    monkeypatch.setenv("OPENAI_API_KEY", "secret")
    monkeypatch.setattr(single_case, "_materialize_frozen_memory", _never_materialize)
    monkeypatch.setattr(single_case, "_pi_paths", lambda: (case_path, case_path))

    class Runner:
        def __init__(self, **_kwargs):
            pass

        def run(self, **_kwargs):
            raise PiRunError("pi exited with status 1", stderr="node: out of memory")

    monkeypatch.setattr(single_case, "CodePolicyMcpServer", _fake_server([]))
    monkeypatch.setattr(single_case, "PiCliRunner", Runner)
    output = tmp_path / "output"
    result = execute_single_case(case_path, config=EvalRunConfig(), output=output)

    assert result.attempt_status == "failed"
    assert result.infra_error == "PiRunError: pi exited with status 1"
    assert result.recording is None
    assert result.progress is None
    assert (output / "stderr.log").read_text() == "node: out of memory"
