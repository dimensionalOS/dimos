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

"""Execute the pinned SPACE implementation against replies already produced by DimOS."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
from pathlib import Path
import subprocess
import sys
from types import ModuleType
from typing import Any

from dimos.evals.suites.lib.space.constants import SOURCE_SHA256, SPACE_REVISION


def verify_source(source: Path) -> None:
    """Reject a different revision or changes to the external executable source."""
    source = source.resolve(strict=True)
    root = subprocess.run(
        ["git", "-C", str(source), "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    ).stdout.strip()
    if Path(root).resolve() != source:
        raise ValueError(f"SPACE source must be the checkout root: {source}")
    revision = subprocess.run(
        ["git", "-C", str(source), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    ).stdout.strip()
    if revision != SPACE_REVISION:
        raise ValueError(f"SPACE revision {revision} does not match {SPACE_REVISION}")
    changes = subprocess.run(
        ["git", "-C", str(source), "status", "--porcelain", "--untracked-files=all", "--", "space"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    ).stdout.strip()
    if changes:
        raise ValueError(f"SPACE executable source has local changes: {changes}")
    for relative, expected in SOURCE_SHA256.items():
        actual = hashlib.sha256((source / relative).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"SPACE source hash mismatch: {relative}")


def question_digest(question: str) -> str:
    """The replay lookup key; it never incorporates the ground-truth answer."""
    return hashlib.sha256(question.encode("utf-8")).hexdigest()


class RawReplyAgent:
    """The upstream agent interface, backed only by saved DimOS responses."""

    def __init__(self, responses: dict[str, str], use_vllm: bool = False) -> None:
        if use_vllm:
            raise ValueError("SPACE response replay cannot enable a model backend")
        self._responses = dict(responses)

    def reset(self) -> None:
        return None

    def get_prediction(self, question_content: Any, answer: Any) -> Any:
        del answer  # Upstream passes the label; it must never select a response.
        if not isinstance(question_content, str):
            raise TypeError("SPACE text QA replay requires an unchanged string question")
        raw = self._responses[question_digest(question_content)]
        # SPACE is an optional external checkout, loaded explicitly by prepare().
        parser = importlib.import_module("space.agents.qa_agent")
        return parser.QA_Agent.parse_answer_from_response(None, raw)

    def get_eval_cost(self) -> dict[str, float]:
        # Replay incurs no provider calls; original usage remains in DimOS artifacts.
        return {}


@dataclass(frozen=True, kw_only=True)
class CaseScore:
    accuracy: float  # The official percentage, before DimOS's 0..1 normalization.
    prediction: Any


def _check_origins(source: Path) -> None:
    expected = source / "space"
    for name, module in tuple(sys.modules.items()):
        if name != "space" and not name.startswith("space."):
            continue
        if module is None:
            raise RuntimeError(f"SPACE import is incomplete: {name}")
        filename = vars(module).get("__file__")
        paths = [filename] if filename else list(vars(module).get("__path__", ()))
        if not paths or any(not Path(path).resolve().is_relative_to(expected) for path in paths):
            raise RuntimeError(f"SPACE module {name} was loaded outside the pinned checkout")


class OfficialSpace:
    """An explicitly prepared scorer; construction performs no imports or downloads."""

    def __init__(self, source: Path) -> None:
        self._source = source
        self._evaluator: ModuleType | None = None

    def prepare(self) -> None:
        """Validate and import SPACE before the eval starts making provider calls.

        Preparation belongs to the sequential run owner. The pinned package stays
        loaded for subsequent grades; another SPACE checkout in this interpreter
        is rejected instead of replacing its module or registry state.
        """
        source = self._source.resolve(strict=True)
        verify_source(source)
        _check_origins(source)
        if self._evaluator is not None:
            return
        path = str(source)
        sys.path.insert(0, path)
        try:
            evaluator = importlib.import_module("space.evaluate_qas")
            registry = importlib.import_module("space.registry")
        finally:
            sys.path.remove(path)
        _check_origins(source)
        existing = registry.AGENTS_REGISTRY.get(RawReplyAgent.__name__)
        if existing is not None and existing is not RawReplyAgent:
            raise RuntimeError("SPACE RawReplyAgent registry name is already occupied")
        if existing is None:
            registry.register_agent(RawReplyAgent)
        self._evaluator = evaluator

    def score(self, qa: dict[str, Any], raw_response: str) -> CaseScore:
        """Call the official per-case evaluator and parser without a model request."""
        if self._evaluator is None:
            raise RuntimeError("Prepare the official SPACE scorer before grading")
        question = qa["question"]
        if not isinstance(question, str) or not isinstance(raw_response, str):
            raise TypeError("SPACE text QA needs a string question and raw response")
        metrics, prediction, _ = self._evaluator.evaluate_on_qa(
            agent_name=RawReplyAgent.__name__,
            agent_cfg={"responses": {question_digest(question): raw_response}},
            qa=qa,
        )
        return CaseScore(accuracy=float(metrics["accuracy"]), prediction=prediction)
