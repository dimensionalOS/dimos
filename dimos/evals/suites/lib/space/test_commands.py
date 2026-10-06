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

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space import commands
from dimos.evals.suites.lib.space.commands import run_job
from dimos.evals.suites.lib.space.constants import SMOKE_INDEX
from dimos.evals.suites.lib.space.data import Example, SpacePaths, load_examples


@pytest.mark.parametrize(
    ("model", "offline", "timeout", "message"),
    [
        (None, False, 10, "provider model is required"),
        ("", False, 10, "provider model is required"),
        ("a-model", True, 10, "cannot select"),
        (None, True, 0, "finite and positive"),
        (None, True, float("inf"), "finite and positive"),
    ],
)
def test_invalid_job_fails_before_acquisition_or_execution(
    tmp_path: Path, model: str | None, offline: bool, timeout: float, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        run_job(
            SpacePaths(tmp_path / "absent-cache"),
            tmp_path / "jobs",
            model=model,
            offline_smoke=offline,
            timeout_s=timeout,
        )
    assert not (tmp_path / "jobs").exists()


def test_truncated_run_retains_explicit_incomplete_job(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    example = Example(SMOKE_INDEX, {"question": "Synthetic", "metadata": {"env_dir": "test"}})
    mocker.patch.object(commands, "load_examples", return_value=(example,))

    def interrupted_write(args: list[str], log_path: Path, timeout_s: float) -> int:
        run = log_path.parent / "runs" / "run-interrupted"
        run.mkdir(parents=True)
        (run / "manifest.json").write_text("{")
        return 0

    mocker.patch.object(commands, "run_process", side_effect=interrupted_write)
    job = run_job(SpacePaths(tmp_path / "cache"), tmp_path, model=None, offline_smoke=True)
    assert not job["complete"]
    assert job["scored_denominator"] == 0
    assert job["counts"] == {"unreported": 1}
    assert "JSONDecodeError" in job["error"]
    assert json.loads((Path(job["job_dir"]) / "job.json").read_text()) == job
    assert (Path(job["job_dir"]) / "runs/run-interrupted/manifest.json").read_text() == "{"


@pytest.mark.self_hosted
def test_native_offline_smoke_preserves_prompt_and_official_score(
    native_space_paths: SpacePaths, tmp_path: Path
) -> None:
    job = run_job(native_space_paths, tmp_path, model=None, offline_smoke=True, timeout_s=120)
    assert job["complete"], job
    report = json.loads(Path(job["report_path"]).read_text())
    assert report["mode"] == "offline-smoke"
    assert report["evidence_label"] == "offline software contract"
    assert report["scored_denominator"] == 1
    assert not report["fixed_subset_complete"]
    (example,) = load_examples(native_space_paths, (SMOKE_INDEX,))
    run = Path(report["run_dir"])
    manifest = json.loads((run / "manifest.json").read_text())
    assert manifest["source"]["cases"] == [example.identity()]
    assert manifest["agent"]["kwargs"]["model"] == "FakeListChatModel"
    result = json.loads((run / "results.jsonl").read_text())
    assert result["final_answer"] == '{"answer":1}'
    assert result["tool_calls"] == 0
    assert not result["error"]
    (request,) = (run / example.case_id / "raw").glob("*-request.json")
    messages = json.loads(request.read_text())["messages"]
    assert len(messages) == 1
    assert messages[0]["type"] == "human"
    assert messages[0]["content"] == [{"type": "text", "text": example.question}]
    official = json.loads(Path(report["official_results_path"]).read_text())
    assert report["official_percent"] == official["mean_metrics"]["accuracy"]
    assert result["score"] * 100 == official["all_metrics"][0]["accuracy"]


@pytest.mark.parametrize("phase", ["runner", "scoring"])
def test_interrupted_job_retains_cancellation_status(
    tmp_path: Path, mocker: MockerFixture, phase: str
) -> None:
    example = Example(SMOKE_INDEX, {"question": "Synthetic", "metadata": {"env_dir": "test"}})
    mocker.patch.object(commands, "load_examples", return_value=(example,))

    def interrupted(args: list[str], log_path: Path, timeout_s: float) -> int:
        run = log_path.parent / "runs" / "run-test"
        run.mkdir(parents=True)
        (run / "manifest.json").write_text("{}")
        if phase == "runner":
            raise KeyboardInterrupt
        return 0

    mocker.patch.object(commands, "run_process", side_effect=interrupted)
    mocker.patch.object(commands, "score_run", side_effect=KeyboardInterrupt)
    with pytest.raises(KeyboardInterrupt):
        run_job(SpacePaths(tmp_path / "cache"), tmp_path, model=None, offline_smoke=True)
    (saved,) = tmp_path.glob("space-*/job.json")
    job = json.loads(saved.read_text())
    assert not job["complete"]
    assert job["cancelled"]
    assert job["scored_denominator"] == 0
    assert job["counts"] == {"unreported": 1}
    assert job["error"] == f"{phase.capitalize()} interrupted"
