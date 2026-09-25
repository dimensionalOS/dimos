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
"""Generic TypeSafe agent: an input stream becomes a JSON state, one System One request
answers typed questions about it, and the answers go out by type.

Subclasses declare the input streams and implement `trigger`, `state` and
`questions`; `on_answers` is the hook for side effects. Inference runs on every
trigger update, or on the latest snapshot at `max_hz` when set.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
from typing import Any

from reactivex import Observable, operators as ops
from reactivex.disposable import Disposable
import requests

from dimos.agents.typesafe.constants import (
    BASE_URL_ENV,
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    REQUEST_TIMEOUT_S,
)
from dimos.agents.typesafe.types import Answers, ChoiceAnswer, NoulAnswer, Question, ScoreAnswer
from dimos.constants import LOG_DIR
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def typesafe_api_key() -> str | None:
    """Blueprint requirement: the key comes from `TYPESAFE_API_KEY` / global config."""
    from dimos.core.global_config import global_config

    if global_config.typesafe_api_key:
        return None
    return "TYPESAFE_API_KEY is not set. Create a key at https://console.typesafe.ai/settings/keys"


class TypeSafeAgentConfig(ModuleConfig):
    model: str = DEFAULT_MODEL
    max_hz: float | None = None  # None: infer on every trigger update
    timeout_s: float = REQUEST_TIMEOUT_S
    trace: bool = False  # raw request/response pairs under the run's log dir


class TypeSafeAgent(Module):
    config: TypeSafeAgentConfig

    choices: Out[dict[str, ChoiceAnswer]]
    scores: Out[dict[str, ScoreAnswer]]
    nouls: Out[dict[str, NoulAnswer]]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session = requests.Session()
        self._busy = threading.Lock()
        self._seq = 0

    # ---- subclass surface ----------------------------------------------------
    def trigger(self) -> Observable[object]:
        """The input stream whose updates drive inference."""
        raise NotImplementedError

    def state(self, trigger: object) -> object | None:
        """The JSON state for this tick given the triggering message, or None to skip it."""
        raise NotImplementedError

    def questions(self, state: object) -> dict[str, Question]:
        raise NotImplementedError

    def on_answers(self, state: object, answers: Answers) -> None:
        """Side effects on a fresh answer set; answers are already published by type."""

    # ---- lifecycle -------------------------------------------------------------
    @rpc
    def start(self) -> None:
        super().start()
        self._session.headers["Authorization"] = f"Bearer {self.config.g.typesafe_api_key or ''}"
        source = self.trigger()
        if self.config.max_hz:
            source = source.pipe(ops.sample(1.0 / self.config.max_hz))
        self.register_disposable(Disposable(source.subscribe(self._on_trigger).dispose))

    @rpc
    def stop(self) -> None:
        self._session.close()
        super().stop()

    def _on_trigger(self, msg: object) -> None:
        # One request in flight; a trigger that lands during it is dropped, the next one wins.
        if not self._busy.acquire(blocking=False):
            return
        try:
            self._infer(msg)
        except Exception:
            logger.exception("TypeSafeAgent inference failed")
        finally:
            self._busy.release()

    def _infer(self, trigger: object) -> None:
        state = self.state(trigger)
        if state is None:
            return
        qs = self.questions(state)
        answers = self._post(state, qs)
        if self.config.trace:
            self._trace(state, qs, answers)
        self.choices.publish({k: a for k, a in answers.items() if a["type"] == "choice"})
        self.scores.publish({k: a for k, a in answers.items() if a["type"] == "score"})
        self.nouls.publish({k: a for k, a in answers.items() if a["type"] == "noul"})
        self.on_answers(state, answers)

    def _post(self, state: object, qs: dict[str, Question]) -> Answers:
        """`POST /v1/systemone`. A failure skips this tick; the next trigger asks again."""
        r = self._session.post(
            os.environ.get(BASE_URL_ENV, DEFAULT_BASE_URL) + "/v1/systemone",
            json={"state": state, "model": self.config.model, "questions": qs},
            timeout=self.config.timeout_s,
        )
        r.raise_for_status()
        answers: Answers = r.json()["answers"]
        return answers

    def _trace(self, state: object, qs: dict[str, Question], answers: Answers) -> None:
        d = Path(os.environ.get("DIMOS_RUN_LOG_DIR", LOG_DIR)) / "typesafe"
        d.mkdir(parents=True, exist_ok=True)
        self._seq += 1
        (d / f"{self._seq}-request.json").write_text(
            json.dumps({"body": {"state": state, "questions": qs}})
        )
        (d / f"{self._seq}-response.json").write_text(json.dumps({"body": {"answers": answers}}))
