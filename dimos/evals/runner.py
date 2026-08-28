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

"""EvalRunner — the one engine behind CLI, MCP skill, and pytest.

Implements the :class:`~dimos.evals.types.EvalRig` protocol structurally.
Cases own their evaluation flow (``case.evaluate(rig)``); the runner owns
resources (model client, MCP adapter, sim process, live store) plus run
lifecycle: preflight, timing, error isolation, artifacts.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import subprocess
import time
from typing import TYPE_CHECKING, Any

from dimos.constants import STATE_DIR
from dimos.core.resource import CompositeResource
from dimos.evals.types import EvalCase, EvalResult, InteractiveEval, ResponseT, Suite
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from langchain_core.language_models.chat_models import BaseChatModel

    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

logger = setup_logger()

EVAL_SYSTEM_PROMPT = (
    "You are evaluating a robot's perception and memory. Answer the question "
    "using only the provided observations. Reply with the answer value only — "
    "a bare number or a short phrase. No explanation, no units unless asked."
)

BLIND_BLOCK: dict[str, str] = {
    "type": "text",
    "text": "[observations withheld — answer anyway]",
}


class EvalRunnerConfig(BaseConfig):
    model: str = "gpt-5.6-luna"  # mirrors McpClientConfig.model
    # House convention (StoreConfig): pass an instance to inject, e.g. a fake
    # chat model in tests. None -> built from `model` like McpClient does.
    chat_model: Any | None = None
    mcp_url: str | None = None  # None -> localhost:{global_config.mcp_port}/mcp
    live_db: str = "recording.db"  # store the Recorder writes (interactive)
    blind: bool = False  # ablation: context withheld (SPACE guessing check)
    threshold: float = 1.0  # passed = score >= threshold
    strict: bool = False  # preflight failure aborts the whole run
    context_budget: int = 8  # max observations encoded per context Select
    attach: bool = False  # True: drive an already-running dimos
    launch_timeout_s: float = 1200.0  # blueprint + MCP readiness (e2e parity)
    out_dir: Path = STATE_DIR / "evals"


@dataclass(frozen=True, kw_only=True)
class RunSummary:
    n: int
    mean_score: float
    pass_rate: float
    errors: int
    duration_s: float


def summarize(results: Sequence[EvalResult]) -> RunSummary:
    scored = [r for r in results if not r.error]
    return RunSummary(
        n=len(results),
        mean_score=sum(r.score for r in scored) / len(scored) if scored else 0.0,
        pass_rate=sum(r.passed for r in scored) / len(scored) if scored else 0.0,
        errors=sum(1 for r in results if r.error),
        duration_s=sum(r.duration_s for r in results),
    )


class EvalRunner(Configurable, CompositeResource):
    config: EvalRunnerConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)
        self._model: BaseChatModel | None = None
        self._proc: DimosCliCall | None = None
        self._sim: DimSimClient | None = None
        self._run_dir: Path | None = None

    # -- run lifecycle -----------------------------------------------------------

    def run(
        self,
        cases: Suite,
        *,
        tags: frozenset[str] = frozenset(),
        limit: int = 0,
    ) -> list[EvalResult]:
        selected = [c for c in cases if not tags or tags & c.tags]
        if limit:
            selected = selected[:limit]
        self._run_dir = self._new_run_dir()

        results: list[EvalResult] = []
        runnable: list[EvalCase] = []
        for case in selected:
            try:
                case.preflight(self)
                runnable.append(case)
            except Exception as e:
                if self.config.strict:
                    raise
                logger.warning("preflight failed", case=case.id, error=str(e))
                results.append(EvalResult(case_id=case.id, error=f"preflight: {e}"))

        for case in runnable:
            result = self._guarded(case)
            logger.info(
                "eval case done",
                case=case.id,
                score=round(result.score, 3),
                error=result.error or None,
            )
            results.append(result)

        self._write_artifacts(results)
        self.stop()
        return results

    def _guarded(self, case: EvalCase) -> EvalResult:
        t0 = time.monotonic()
        try:
            result = case.evaluate(self)
            transcript = self.run_dir / f"{case.id}.jsonl"
            return replace(
                result,
                duration_s=time.monotonic() - t0,
                passed=result.score >= self.config.threshold and not result.error,
                transcript=str(transcript) if transcript.exists() else result.transcript,
            )
        except Exception as e:
            return EvalResult(case_id=case.id, error=repr(e), duration_s=time.monotonic() - t0)
        finally:
            self.teardown_env()

    @property
    def run_dir(self) -> Path:
        assert self._run_dir is not None, "run_dir is available only during run()"
        return self._run_dir

    def _new_run_dir(self) -> Path:
        run_dir = self.config.out_dir / time.strftime("run-%Y%m%d-%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        return run_dir

    def _write_artifacts(self, results: list[EvalResult]) -> None:
        lines = [json.dumps(asdict(r)) for r in results]
        (self.run_dir / "results.jsonl").write_text("\n".join(lines) + "\n")
        summary: dict[str, Any] = asdict(summarize(results))
        summary |= {"model": self.config.model, "blind": self.config.blind, "git": _git_sha()}
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    def stop(self) -> None:
        self.teardown_env()
        super().stop()

    # -- EvalRig: shared ------------------------------------------------------------

    @property
    def blind(self) -> bool:
        return self.config.blind

    @property
    def mcp_url(self) -> str:
        if self.config.mcp_url is not None:
            return self.config.mcp_url
        from dimos.core.global_config import global_config

        return f"http://localhost:{global_config.mcp_port}/mcp"

    def open_dataset(self, name: str) -> Store:
        from dimos.memory.cli.dataset import open_dataset

        return open_dataset(name)

    def live_store(self) -> Store:
        from dimos.memory.store.sqlite import SqliteStore

        return SqliteStore(path=self.config.live_db, must_exist=True)

    def encode(self, stream: Stream[Any, Any]) -> list[dict[str, Any]]:
        """mem2 Stream -> model-legible content blocks (the surface under test).

        Metadata iterates lazily; blobs load only for the <= context_budget
        observations actually encoded. ``agent_encode()`` is used where a type
        provides it; ``str(data)`` otherwise (an encoding gap the eval will
        surface, by design).
        """
        observations = list(stream)
        if not observations:
            return [{"type": "text", "text": f"stream {stream.name!r}: no observations"}]

        budget = self.config.context_budget
        if len(observations) > budget:
            step = (len(observations) - 1) / (budget - 1)
            observations = [observations[round(i * step)] for i in range(budget)]

        t0 = observations[0].ts
        blocks: list[dict[str, Any]] = [
            {
                "type": "text",
                "text": f"observations from stream {stream.name!r} "
                f"(t is seconds from the first shown):",
            }
        ]
        for obs in observations:
            data = obs.data
            encoded = data.agent_encode() if hasattr(data, "agent_encode") else None
            stamp = f"[t={obs.ts - t0:.1f}s]"
            if isinstance(encoded, list):  # e.g. Image -> image_url blocks
                blocks.append({"type": "text", "text": stamp})
                blocks.extend(encoded)
            elif encoded is not None:
                blocks.append(
                    {"type": "text", "text": f"{stamp} {json.dumps(encoded, default=str)}"}
                )
            else:
                blocks.append({"type": "text", "text": f"{stamp} {data}"})
        return blocks

    def ask(self, context: Sequence[dict[str, Any]], question: str) -> str:
        from langchain_core.messages import HumanMessage, SystemMessage

        blocks = list(context) if context else [BLIND_BLOCK]
        message = HumanMessage(content=[*blocks, {"type": "text", "text": question}])
        response = self.model.invoke([SystemMessage(EVAL_SYSTEM_PROMPT), message])
        return str(response.text)

    def ask_structured(
        self,
        context: Sequence[dict[str, Any]],
        question: str,
        schema: type[ResponseT],
    ) -> ResponseT:
        """Ask for a provider-native response validated against a Pydantic schema."""
        from langchain_core.messages import HumanMessage, SystemMessage

        blocks = list(context) if context else [BLIND_BLOCK]
        message = HumanMessage(content=[*blocks, {"type": "text", "text": question}])
        try:
            structured_model = self.model.with_structured_output(schema)
        except NotImplementedError as exc:
            raise RuntimeError(
                f"model {self.config.model!r} does not support structured output"
            ) from exc
        response = structured_model.invoke([SystemMessage(EVAL_SYSTEM_PROMPT), message])
        if not isinstance(response, schema):
            raise TypeError(
                f"model {self.config.model!r} returned {type(response).__name__}, "
                f"expected {schema.__name__}"
            )
        return response

    @property
    def model(self) -> BaseChatModel:
        if self.config.chat_model is not None:
            return self.config.chat_model  # type: ignore[no-any-return]
        if self._model is None:
            # Same construction as the production agent (Responses-API branch
            # for gpt-5.x) so evals measure the deployed model config.
            from dimos.agents.mcp.mcp_client import _init_model

            self._model = _init_model(self.config.model)
        return self._model

    def call_skill(self, name: str, args: Mapping[str, object]) -> str:
        from dimos.agents.mcp.mcp_adapter import McpAdapter

        return McpAdapter(self.mcp_url).call_tool_text(name, dict(args))

    def mcp_ready(self) -> bool:
        from dimos.agents.mcp.mcp_adapter import McpAdapter

        return McpAdapter(self.mcp_url).wait_for_ready(timeout=2.0)

    def agent_loop(self, case: EvalCase) -> str:
        """Fresh create_agent per case over the MCP toolset — the McpClient loop
        minus its queue/thread shell. Transcript -> <run_dir>/<case_id>.jsonl."""
        from langchain.agents import create_agent
        from langchain_core.messages import HumanMessage, SystemMessage
        from langchain_core.tools import StructuredTool

        from dimos.agents.mcp.mcp_adapter import McpAdapter

        adapter = McpAdapter(self.mcp_url)
        tools = [
            StructuredTool(
                name=t["name"],
                description=t.get("description", ""),
                args_schema=t.get("inputSchema", {}),
                func=lambda _name=t["name"], **kwargs: adapter.call_tool_text(_name, kwargs),
            )
            for t in adapter.list_tools()
        ]
        graph: Any = create_agent(self.model, tools)
        messages: list[Any] = [SystemMessage(EVAL_SYSTEM_PROMPT), HumanMessage(case.inputs)]
        transcript = self.run_dir / f"{case.id}.jsonl"
        final_text = ""
        with transcript.open("w") as fh:
            for update in graph.stream({"messages": messages}, stream_mode="updates"):
                for payload in update.values():
                    for msg in payload.get("messages", []):
                        fh.write(
                            json.dumps({"type": type(msg).__name__, "content": str(msg.content)})
                            + "\n"
                        )
                        final_text = str(msg.content)
        return final_text

    # -- EvalRig: interactive ----------------------------------------------------------

    def setup_env(self, case: InteractiveEval) -> None:
        from dimos.evals.types import _no_setup

        if case.simulator and not self.config.attach:
            from dimos.e2e_tests.dimos_cli_call import DimosCliCall

            proc = DimosCliCall()
            proc.simulator = case.simulator
            proc.global_args = ["--dimsim-scene", case.scene]
            proc.demo_args = ["run", *case.blueprint.split()]
            proc.start()
            self._proc = proc
        if not self._wait_mcp(self.config.launch_timeout_s):
            raise RuntimeError(f"MCP at {self.mcp_url} not ready — is dimos up?")
        if case.setup is not _no_setup:
            from dimos.e2e_tests.dim_sim_client import DimSimClient

            sim = DimSimClient()
            sim.start()
            self._sim = sim
            case.setup(sim)

    def teardown_env(self) -> None:
        """Per-case cleanup — the runner owns env lifecycle, cases just declare it."""
        if self._sim is not None:
            self._sim.stop()
            self._sim = None
        if self._proc is not None:
            self._proc.stop()
            self._proc = None

    def check_env(self, case: InteractiveEval) -> None:
        if self.config.attach or not case.simulator:
            if not self.mcp_ready():
                raise RuntimeError(
                    f"{case.id}: attach mode needs a running dimos at {self.mcp_url}"
                )
            return
        import shutil

        if case.simulator == "dimsim" and shutil.which("deno") is None:
            raise RuntimeError(f"{case.id}: dimsim requires deno on PATH")

    def _wait_mcp(self, timeout: float) -> bool:
        from dimos.agents.mcp.mcp_adapter import McpAdapter

        return McpAdapter(self.mcp_url).wait_for_ready(timeout=timeout, interval=2.0)

    def instruct(self, text: str) -> None:
        from dimos.core.transport_factory import make_transport

        transport = make_transport("/human_input")
        transport.start()
        try:
            transport.publish(text)
        finally:
            transport.stop()

    def sample(
        self, score: Callable[[Store], float], interval_s: float, timeout_s: float
    ) -> list[tuple[float, float]]:
        """Score the live Recorder store on an interval — the mem2 analogue of
        lcm_spy.wait_until_odom_position, but it returns a graded series."""
        deadline = time.monotonic() + timeout_s
        t0 = time.monotonic()
        series: list[tuple[float, float]] = []
        store = self._wait_live_store(deadline)
        try:
            while time.monotonic() < deadline:
                try:
                    value = score(store)
                except LookupError:
                    value = None  # stream not written yet — keep waiting
                if value is not None:
                    series.append((time.monotonic() - t0, value))
                    if value >= 0.999:  # ponytail: early exit on success; drop if
                        break  # aggregates ever need the full window
                time.sleep(interval_s)
        finally:
            store.stop()
        return series

    def _wait_live_store(self, deadline: float) -> Store:
        path = Path(self.config.live_db)
        while not path.exists() and time.monotonic() < deadline:
            time.sleep(1.0)
        return self.live_store()


def _git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        ).stdout.strip()
    except OSError:
        return ""
