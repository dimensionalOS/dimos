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

from __future__ import annotations

import hashlib
from importlib.metadata import PackageNotFoundError, PathDistribution
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space import upstream
from dimos.evals.suites.lib.space.constants import SOURCE_SHA256, SPACE_PACKAGE
from dimos.evals.suites.lib.space.upstream import OfficialSpace, RawReplyAgent, question_digest


@pytest.fixture
def package(tmp_path: Path, mocker: MockerFixture, monkeypatch: pytest.MonkeyPatch) -> Path:
    # Restore any real SPACE modules after isolating this synthetic distribution.
    for name in tuple(sys.modules):
        if name == "space" or name.startswith("space."):
            monkeypatch.delitem(sys.modules, name)
    root = tmp_path / "site-packages"
    package_version = "9.9.9"
    metadata = root / f"{SPACE_PACKAGE.replace('-', '_')}-{package_version}.dist-info"
    metadata.mkdir(parents=True)
    (metadata / "METADATA").write_text(
        f"Metadata-Version: 2.1\nName: {SPACE_PACKAGE}\nVersion: {package_version}\n"
    )
    hashes: dict[str, str] = {}
    for relative in SOURCE_SHA256:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("# Synthetic source-integrity fixture.\n")
        hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    mocker.patch.object(upstream, "distribution", return_value=PathDistribution(metadata))
    mocker.patch.object(upstream, "SOURCE_SHA256", hashes)
    return root


def test_verify_package_accepts_the_installed_distribution(package: Path) -> None:
    installed = upstream.verify_package()
    assert installed.version == "9.9.9"
    assert Path(str(installed.locate_file(""))) == package


def test_verify_package_reports_a_missing_dependency(mocker: MockerFixture) -> None:
    mocker.patch.object(upstream, "distribution", side_effect=PackageNotFoundError(SPACE_PACKAGE))
    with pytest.raises(RuntimeError, match=SPACE_PACKAGE) as error:
        upstream.verify_package()
    assert "install" in str(error.value).lower()


@pytest.mark.parametrize("missing", [False, True])
def test_verify_package_rejects_changed_or_missing_scorer(package: Path, missing: bool) -> None:
    evaluator = package / "space/evaluate_qas.py"
    if missing:
        evaluator.unlink()
    else:
        evaluator.write_text("# unexpected change\n")
    with pytest.raises(FileNotFoundError if missing else ValueError):
        upstream.verify_package()


@pytest.mark.parametrize("name", ["space", "space.evaluate_qas"])
def test_prepare_rejects_an_existing_foreign_space_package(
    package: Path, tmp_path: Path, mocker: MockerFixture, name: str
) -> None:
    foreign = ModuleType(name)
    foreign.__file__ = str(tmp_path / "elsewhere" / "__init__.py")
    mocker.patch.dict(sys.modules, {name: foreign})
    with pytest.raises(RuntimeError, match="outside the installed package"):
        OfficialSpace().prepare()


def test_score_uses_the_official_metric_and_exact_question(
    package: Path, mocker: MockerFixture
) -> None:
    evaluator = SimpleNamespace()
    registry = SimpleNamespace()
    evaluate = mocker.Mock(return_value=({"accuracy": 37.5}, 2, {}))
    evaluator.evaluate_on_qa = evaluate
    registry.AGENTS_REGISTRY = {}
    registry.register_agent = mocker.Mock()
    modules = {"space.evaluate_qas": evaluator, "space.registry": registry}
    mocker.patch(
        "dimos.evals.suites.lib.space.upstream.importlib.import_module",
        side_effect=modules.__getitem__,
    )
    qa: dict[str, Any] = {
        "question": "Unchanged\nquestion",
        "answer": 2,
        "metadata": {"untouched": True},
    }
    raw = '  {"answer":2}\n'
    scorer = OfficialSpace()
    scorer.prepare()
    result = scorer.score(qa, raw)
    assert result.accuracy == 37.5
    assert result.prediction == 2
    evaluate.assert_called_once_with(
        agent_name="RawReplyAgent",
        agent_cfg={"responses": {question_digest(qa["question"]): raw}},
        qa=qa,
    )
    registry.register_agent.assert_called_once_with(RawReplyAgent)


def test_replay_accepts_official_config_and_never_selects_by_label(mocker: MockerFixture) -> None:
    parser = SimpleNamespace()
    parse = mocker.Mock(return_value=3)
    parser.QA_Agent = type("QA_Agent", (), {"parse_answer_from_response": parse})
    mocker.patch(
        "dimos.evals.suites.lib.space.upstream.importlib.import_module", return_value=parser
    )
    raw = ' {"answer":"3"} '
    agent = RawReplyAgent({question_digest("question"): raw}, use_vllm=False)
    assert agent.get_prediction("question", 1) == 3
    assert agent.get_prediction("question", 4) == 3
    assert parse.call_args_list == [mocker.call(None, raw), mocker.call(None, raw)]


def test_replay_rejects_enabling_a_model_backend() -> None:
    with pytest.raises(ValueError, match="cannot enable a model backend"):
        RawReplyAgent({}, use_vllm=True)


def test_unprepared_scoring_fails_before_using_a_response() -> None:
    with pytest.raises(RuntimeError, match="Prepare"):
        OfficialSpace().score({"question": "q", "answer": 1}, '{"answer":1}')
