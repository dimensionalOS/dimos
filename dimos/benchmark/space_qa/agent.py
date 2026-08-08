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

"""SPACE's own QA agent, answering through the DimOS agent-evaluation path.

SPACE stays in charge: it drives the loop, formats the question, parses the
reply and keeps the answer key. ``get_prediction`` keeps the upstream method
line for line with one call replaced — what would reach a chat completion runs
a static evaluation case instead — so every judgement about the answer is
still made by inherited code.

Two things about this module are load-bearing:

* Importing it registers ``DimosQAAgent`` and the ``dimos_qa`` config in
  SPACE's global registries. ``register_config`` asserts that a name is
  registered once, so it must be imported under its own dotted name and never
  run as a script, which would register it a second time as ``__main__``.
* Importing it needs a SPACE checkout on ``sys.path``. ``run_space_task`` puts
  the pinned one there first; nothing else in this package imports it.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import time
from typing import Any

from space.agents.qa_agent import QA_Agent  # type: ignore[import-not-found]
from space.registry import register_agent, register_config  # type: ignore[import-not-found]

from dimos.benchmark.agent_eval.models import CompactEvalResult, EvalCase, EvalRunConfig
from dimos.benchmark.agent_eval.single_case import execute_single_case
from dimos.benchmark.space_qa.manifest import (
    RECORD_NAME,
    SelectedRow,
    case_dir,
    read_manifest,
    run_dir_from_env,
)

CONFIG_NAME = "dimos_qa"
SAVE_DIR_PATTERN = re.compile(r"qa_(\d+)")


@register_agent
class DimosQAAgent(QA_Agent):  # type: ignore[misc]
    """A ``QA_Agent`` whose model call is one DimOS evaluation case."""

    def __init__(
        self,
        model_name: str,
        host_port: str | None = None,
        save_dir: str | None = None,
        image_detail: str = "low",
        max_new_tokens: int = 2048,
        completion_cost_per_mil: float = 0.0,
        prompt_cost_per_mil: float = 0.0,
        subsampling_factor: int = 1,
        max_images_per_query: int = -1,
        **kwargs: Any,
    ) -> None:
        # Deliberately not calling super().__init__(): it asserts an API key and
        # builds a live chat client, and this agent never calls a model itself.
        self.model_name = model_name
        self.host_port = host_port
        self.save_dir = save_dir
        self.image_detail = image_detail
        self.max_new_tokens = max_new_tokens
        self.completion_cost_per_mil = completion_cost_per_mil
        self.prompt_cost_per_mil = prompt_cost_per_mil
        self.subsampling_factor = subsampling_factor
        self.max_images_per_query = max_images_per_query
        # Untyped like their upstream counterparts: `reset` fills them in, and
        # the inherited methods that read them are the ones that know the shape.
        self.writer: Any = None
        self.dialog: Any = None
        self.completion_tokens: Any = None
        self.prompt_tokens: Any = None
        self.client: Any = None

    def get_prediction(self, question_content: list[Any] | str, answer: Any) -> Any:
        """The upstream method with its model call replaced.

        Gone with it is the image-batching branch upstream takes for list
        question content: this agent answers text tasks and refuses anything
        else. Everything that reads the reply is still inherited.

        Nothing raises out of here. A worker that dies takes its pool slot and
        the rest of the round with it, so an unrunnable question returns None
        — which SPACE scores as a miss — and says why in its own record.
        """
        started = time.monotonic()
        record: dict[str, Any] = {
            "case_id": None,
            "ordinal": None,
            "question_sha256": None,
            "pred": None,
            "gt": answer,
            "space_parse_status": "invalid",
            "prediction_status": None,
            "infra_error": None,
            "tool_call_count": 0,
            "duration_seconds": 0.0,
            "final_text": "",
        }
        prediction: Any = None
        try:
            question = _text_question(question_content)
            record["question_sha256"] = hashlib.sha256(question.encode("utf-8")).hexdigest()
            question_content = self.preprocess_question(question_content)
            self.dialog.add_user_message(content=question_content)

            result, case, row = self._answer_through_dimos(question)  # the one replaced call
            record.update(
                case_id=case.case_id,
                ordinal=row.ordinal,
                prediction_status=result.prediction_status,
                infra_error=result.infra_error,
                tool_call_count=result.tool_call_count,
                final_text=result.final_response,
            )

            response_txt = self.postprocess_response(result.final_response)
            self.dialog.add_assistant_message(content=response_txt)
            prediction = self.parse_answer_from_response(response_txt)
            self.dialog.log_writer.write(
                f"\n\nGround-truth answer: {answer}, prediction: {prediction}"
            )
            self.completion_tokens += 0
            self.prompt_tokens += 0
            self.dialog.log_token_usage(self.prompt_tokens, self.completion_tokens, 0.0)
            self.dialog.log_response_time(time.monotonic() - started)
            self.dialog.delete_last_message()
            self.dialog.delete_last_message()

            record["pred"] = prediction
            record["space_parse_status"] = "invalid" if prediction is None else "parsed"
        except Exception as exc:
            prediction = None
            record["pred"] = None
            record["space_parse_status"] = "invalid"
            record["infra_error"] = f"{type(exc).__name__}: {exc}"
        finally:
            record["duration_seconds"] = round(time.monotonic() - started, 3)
            _write_record(self.save_dir, record)
        return prediction

    def _answer_through_dimos(
        self, question: str
    ) -> tuple[CompactEvalResult, EvalCase, SelectedRow]:
        """Run the case this question was drawn as, and hand back the raw reply."""
        if not self.save_dir:
            raise RuntimeError("the dimos_qa config must keep its save_dir key")
        run_dir = run_dir_from_env()
        manifest = read_manifest(run_dir)
        subset_index = _subset_index(self.save_dir)
        row = manifest.row_for(subset_index)
        case_path = case_dir(run_dir, subset_index) / "case.json"
        case = EvalCase.model_validate_json(case_path.read_bytes())
        if case.task.prompt != question:
            raise ValueError(
                f"{case.case_id} was built for a different question than SPACE passed to "
                f"subset index {subset_index}; the run directory and the subset disagree"
            )
        result = execute_single_case(
            case_path, config=EvalRunConfig(), output=Path(self.save_dir) / "dimos"
        )
        return result, case, row


@register_config(CONFIG_NAME)
@dataclass
class DimosQAConfig:
    """Defaults ``space.evaluate_qas.main`` reads to build the agent for every question."""

    # Read by evaluate_qas itself; both keys are mandatory.
    agent_name: str = "DimosQAAgent"
    use_vllm: bool = False
    # Read by the agent. model_name must not start with `claude` or `mistralai`:
    # QA_Agent branches on that prefix when it picks a dialog and preprocessor.
    model_name: str = "dimos-pi-gpt-5.6-luna"
    max_context_tokens: int = 128_000
    completion_cost_per_mil: float = 0.0
    prompt_cost_per_mil: float = 0.0
    supports_system_prompt: bool = True
    max_new_tokens: int = 4096
    host_port: str | None = None
    # The key has to exist, or evaluate_qas never assigns a per-question directory.
    save_dir: str | None = None
    image_detail: str = "low"
    subsampling_factor: int = 1


def _text_question(question_content: list[Any] | str) -> str:
    if not isinstance(question_content, str):
        raise TypeError(
            "this agent answers text questions only; "
            f"got {type(question_content).__name__} question content"
        )
    return question_content


def _subset_index(save_dir: str) -> int:
    """SPACE names a question's directory after its position in the subset it was given."""
    matched = SAVE_DIR_PATTERN.fullmatch(Path(save_dir).name)
    if matched is None:
        raise ValueError(f"cannot read a subset index from save_dir {save_dir!r}")
    return int(matched.group(1))


def _write_record(save_dir: str | None, record: dict[str, Any]) -> None:
    """One record per question, beside SPACE's own transcript for the same question."""
    if not save_dir:
        return
    destination = Path(save_dir)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / RECORD_NAME).write_text(
        json.dumps(record, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
