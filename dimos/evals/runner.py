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
from typing import TYPE_CHECKING, Any, cast

from dimos.constants import STATE_DIR
from dimos.core.resource import CompositeResource
from dimos.evals.types import (
    EvalCase,
    EvalResult,
    InteractiveAction,
    InteractiveEval,
    InteractiveOutputScore,
    InteractiveTrialResult,
    Suite,
)
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from langchain_core.language_models.chat_models import BaseChatModel

    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream
    from dimos.porcelain.dimos import Dimos
    from dimos.sim2.episodes import PublicEpisodeContext
    from dimos.sim2.evaluation import (
        EpisodeActivationResult,
        EpisodeBoundary,
        EpisodeBoundaryListener,
        EpisodeEvaluationResult,
        EpisodeProvider,
        PreparedEpisode,
    )

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
        self._app: Dimos | None = None
        self._episode_provider: EpisodeProvider | None = None
        self._episode: PreparedEpisode | None = None
        self._episode_context: PublicEpisodeContext | None = None
        self._episode_sample_index: int | None = None
        self._episode_activation: EpisodeActivationResult | None = None
        self._episode_boundary_sequence = 0
        self._process_trial_initial_sample_index: int | None = None
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
        lines = [json.dumps(result.to_json_dict()) for result in results]
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
                for _node, payload in update.items():
                    for msg in payload.get("messages", []):
                        fh.write(
                            json.dumps({"type": type(msg).__name__, "content": str(msg.content)})
                            + "\n"
                        )
                        final_text = str(msg.content)
        return final_text

    # -- EvalRig: interactive ----------------------------------------------------------

    def run_interactive_trials(self, case: InteractiveEval) -> EvalResult:
        """Run explicit samples and aggregate their physical outcomes."""

        if case.episode is None:
            raise ValueError("repeated interactive trials require an episode provider")
        trials: list[InteractiveTrialResult] = []
        if case.trial_isolation.value == "episode-boundary":
            self.setup_env(case)
            for trial_index in range(case.trials):
                trials.append(self._run_interactive_trial(case, trial_index))
        else:
            self._process_trial_initial_sample_index = None
            try:
                for trial_index in range(case.trials):
                    try:
                        self._prepare_isolated_trial(case, trial_index)
                        trials.append(self._run_interactive_trial(case, trial_index))
                    finally:
                        self._teardown_trial(case, trial_index)
            finally:
                self._process_trial_initial_sample_index = None

        scores = [trial.score for trial in trials]
        errors = [trial.error for trial in trials if trial.error]
        metric_names = sorted({name for trial in trials for name in trial.metrics})
        metrics = {
            name: sum(trial.metrics[name] for trial in trials if name in trial.metrics)
            / sum(name in trial.metrics for trial in trials)
            for name in metric_names
        }
        oracle = json.dumps(
            [
                {
                    "trial_index": trial.trial_index,
                    "sample_index": trial.sample_index,
                    "sample_digest": trial.sample_digest,
                    "oracle": trial.oracle,
                    "error": trial.error,
                }
                for trial in trials
            ],
            sort_keys=True,
        )
        first = trials[0]
        return EvalResult(
            case_id=case.id,
            provider=case.provider_name,
            episode_id=first.episode_id,
            episode_case_id=case.episode.case_id,
            outputs=trials[-1].outputs,
            score=case.trial_aggregate(scores),
            error="; ".join(errors),
            trials=tuple(trials),
            oracle=oracle,
            metrics=metrics,
        )

    def _run_interactive_trial(
        self,
        case: InteractiveEval,
        trial_index: int,
    ) -> InteractiveTrialResult:
        if self._episode is None:
            raise RuntimeError("interactive trial requires a prepared episode")
        sample_index = (
            self._episode.initial_sample_index
            if case.trial_isolation.value == "process"
            else self._episode.initial_sample_index + trial_index
        )
        activation: EpisodeActivationResult | None = None
        outputs = ""
        try:
            if (
                self._episode_activation is not None
                and self._episode_activation.sample_index == sample_index
            ):
                activation = self._episode_activation
            else:
                activation = self._activate_episode_sample(case, sample_index)

            if case.action is not None:
                outputs = self.run_action(case.action)
            elif case.skill:
                outputs = self.call_skill(case.skill, case.skill_args)
            else:
                instruction = case.instruction_override or self.episode_instruction()
                self.instruct(instruction)

            oracle = ""
            metrics: Mapping[str, float] = {}
            if case.output_score is not None:
                value, oracle = self.score_output(case.output_score, outputs)
                series = [(0.0, value)]
            elif case.score is not None:
                series = self.sample(case.score, case.interval_s, case.timeout_s)
            else:
                series, evaluation = self.sample_episode(case.interval_s, case.timeout_s)
                oracle = evaluation.summary
                metrics = evaluation.metrics
            if not series:
                raise RuntimeError(f"{case.id} trial {trial_index}: no samples collected")
            return InteractiveTrialResult(
                trial_index=trial_index,
                sample_index=sample_index,
                episode_id=self._episode.episode_id,
                sample_digest=activation.sample_digest or "",
                outputs=outputs,
                score=case.aggregate([value for _elapsed, value in series]),
                oracle=oracle,
                metrics=metrics,
                provenance=activation.provenance,
                series=tuple(series),
            )
        except Exception as error:
            return InteractiveTrialResult(
                trial_index=trial_index,
                sample_index=sample_index,
                episode_id="" if self._episode is None else self._episode.episode_id,
                sample_digest=(
                    ""
                    if activation is None or activation.sample_digest is None
                    else activation.sample_digest
                ),
                outputs=outputs,
                score=0.0,
                error=repr(error),
                provenance={} if activation is None else activation.provenance,
            )

    def _activate_episode_sample(
        self,
        case: InteractiveEval,
        sample_index: int,
    ) -> EpisodeActivationResult:
        from dimos.e2e_tests.episode import activate_episode

        if self._episode_provider is None or self._episode is None:
            raise RuntimeError("episode sample activation requires a running provider")
        activation = activate_episode(self._episode_provider, self._episode, sample_index)
        if not activation.initial_conditions_passed:
            raise RuntimeError(
                f"episode initial conditions failed: {list(activation.failed_conditions)}"
            )
        self._episode_context = activation.context
        self._episode_sample_index = activation.sample_index
        self._episode_activation = activation
        self._publish_episode_boundary(activation.boundary)
        return activation

    def _publish_episode_boundary(self, boundary: EpisodeBoundary) -> None:
        expected = self._episode_boundary_sequence + 1
        if boundary.sequence != expected:
            raise ValueError(
                f"episode boundary sequence is {boundary.sequence}; expected {expected}"
            )
        self._episode_boundary_sequence = boundary.sequence
        if self._app is None:
            return
        listeners = [rpc for rpc in self._app.list_rpcs() if rpc.name == "on_episode_boundary"]
        for listener in listeners:
            if listener.module_name is None:
                continue
            module = self._app.get_module(listener.module_name)
            cast("EpisodeBoundaryListener", module).on_episode_boundary(boundary)

    def _prepare_isolated_trial(self, case: InteractiveEval, trial_index: int) -> None:
        if trial_index == 0:
            self._setup_episode_env(case)
            assert self._episode is not None
            self._process_trial_initial_sample_index = self._episode.initial_sample_index
            return
        if self._process_trial_initial_sample_index is None:
            raise RuntimeError("process-isolated trials have no initial sample index")
        self._setup_episode_env(
            case,
            sample_index=self._process_trial_initial_sample_index + trial_index,
        )

    def _teardown_trial(self, case: InteractiveEval, trial_index: int) -> None:
        del case, trial_index
        self.teardown_env()

    def setup_env(self, case: InteractiveEval) -> None:
        if case.episode is not None:
            self._setup_episode_env(case)
            if case.action is None and not self._wait_mcp(self.config.launch_timeout_s):
                raise RuntimeError(f"MCP at {self.mcp_url} not ready — is dimos up?")
            return

        assert case.environment is not None
        environment = case.environment
        if environment.simulator and not self.config.attach:
            from dimos.e2e_tests.dimos_cli_call import DimosCliCall

            proc = DimosCliCall()
            proc.simulator = environment.simulator
            proc.global_args = ["--dimsim-scene", environment.scene] if environment.scene else []
            proc.demo_args = ["run", *case.blueprint.split()]
            proc.start()
            self._proc = proc
        if not self._wait_mcp(self.config.launch_timeout_s):
            raise RuntimeError(f"MCP at {self.mcp_url} not ready — is dimos up?")
        from dimos.evals.types import _no_setup

        if environment.setup is not _no_setup:
            from dimos.e2e_tests.dim_sim_client import DimSimClient

            sim = DimSimClient()
            sim.start()
            self._sim = sim
            environment.setup(sim)

    def teardown_env(self) -> None:
        """Per-case cleanup — the runner owns env lifecycle, cases just declare it."""
        self._episode = None
        self._episode_context = None
        self._episode_sample_index = None
        self._episode_activation = None
        self._episode_boundary_sequence = 0
        resources = (
            ("episode provider", "_episode_provider"),
            ("DimOS connection", "_app"),
            ("DimSim client", "_sim"),
            ("DimOS process", "_proc"),
        )
        for label, attribute in resources:
            resource = getattr(self, attribute)
            setattr(self, attribute, None)
            if resource is None:
                continue
            try:
                resource.stop()
            except Exception as error:
                logger.warning("eval resource cleanup failed", resource=label, error=str(error))

    def check_env(self, case: InteractiveEval) -> None:
        if case.episode is not None:
            if self.config.attach:
                raise RuntimeError(
                    f"{case.id}: exact provider episodes own launch and reset; attach is unsupported"
                )
            from dimos.sim2.evaluation import load_episode_provider

            load_episode_provider(case.provider_name).stop()
            return
        assert case.environment is not None
        if self.config.attach or not case.environment.simulator:
            if not self.mcp_ready():
                raise RuntimeError(
                    f"{case.id}: attach mode needs a running dimos at {self.mcp_url}"
                )
            return
        import shutil

        if case.environment.simulator == "dimsim" and shutil.which("deno") is None:
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

    def episode_instruction(self) -> str:
        if self._episode_context is None:
            raise RuntimeError("exact episode instruction requires an active episode")
        return self._episode_context.instruction

    def run_action(self, action: InteractiveAction) -> str:
        if self._app is None or self._episode_context is None:
            raise RuntimeError("public action requires an active exact episode")
        output = action(self._app, self._episode_context)
        if not isinstance(output, str):
            raise TypeError("interactive public action must return str")
        return output

    def score_output(self, score: InteractiveOutputScore, output: str) -> tuple[float, str]:
        if self._episode is None:
            raise RuntimeError("output scoring requires an active exact episode")
        value = float(score(output, self._episode))
        summary = json.dumps(
            {
                "kind": "prepared-episode-match",
                "expected_roles": {
                    role_id: role.name for role_id, role in self._episode.context.roles.items()
                },
                "expected_reset_positions": {
                    role_id: position
                    for role_id, position in self._episode.private_role_reset_positions.items()
                },
            },
            sort_keys=True,
        )
        return value, summary

    def episode_identity(self) -> tuple[str, str, str]:
        if self._episode is None:
            raise RuntimeError("no exact episode is active")
        return (
            self._episode.provider_name,
            self._episode.episode_id,
            self._episode.case_id,
        )

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

    def sample_episode(
        self, interval_s: float, timeout_s: float
    ) -> tuple[list[tuple[float, float]], EpisodeEvaluationResult]:
        """Poll evaluator-only episode truth without publishing it to mem2."""
        from dimos.e2e_tests.episode import evaluate_episode

        if self._episode_provider is None or self._episode is None:
            raise RuntimeError("private episode scoring requires an active exact episode")
        deadline = time.monotonic() + timeout_s
        t0 = time.monotonic()
        series: list[tuple[float, float]] = []
        result = evaluate_episode(self._episode_provider, self._episode)
        while True:
            series.append((time.monotonic() - t0, 1.0 if result.passed else 0.0))
            if result.passed or time.monotonic() >= deadline:
                return series, result
            time.sleep(interval_s)
            result = evaluate_episode(self._episode_provider, self._episode)

    def _setup_episode_env(
        self,
        case: InteractiveEval,
        *,
        sample_index: int | None = None,
    ) -> None:
        from dimos.e2e_tests.dimos_cli_call import DimosCliCall
        from dimos.e2e_tests.episode import prepare_episode, start_episode
        from dimos.sim2.evaluation import EvaluationCase, load_episode_provider

        assert case.episode is not None
        provider = load_episode_provider(case.provider_name)
        self._episode_provider = provider
        episode_output = self.run_dir / case.id / "episode"
        if sample_index is not None:
            episode_output /= f"sample-{sample_index}"
        episode = prepare_episode(
            provider,
            EvaluationCase(
                episode_request=case.episode,
                blueprint_name=case.blueprint,
                required_modules=case.required_modules,
                required_roles=case.required_roles,
            ),
            episode_output,
            sample_index=sample_index,
        )
        self._episode = episode

        if case.trial_isolation.value == "episode-boundary":
            self._preflight_episode_boundary(case, episode)

        proc = DimosCliCall()
        proc.simulator = episode.simulator
        proc.global_args = list(episode.global_args)
        proc.extra_env = dict(episode.extra_env)
        proc.demo_args = [episode.blueprint_name]
        self._proc = proc
        proc.start()

        app = self._connect_dimos(episode.required_modules, self.config.launch_timeout_s)
        self._app = app
        activation = start_episode(provider, episode)
        if not activation.initial_conditions_passed:
            raise RuntimeError(
                f"episode initial conditions failed: {list(activation.failed_conditions)}"
            )
        self._episode_context = activation.context
        self._episode_sample_index = activation.sample_index
        self._episode_activation = activation
        self._publish_episode_boundary(activation.boundary)

    def _preflight_episode_boundary(
        self,
        case: InteractiveEval,
        episode: PreparedEpisode,
    ) -> None:
        """Reject cross-topology warm trials before launching DimOS."""

        from dimos.e2e_tests.episode import validate_episode_activation

        if self._episode_provider is None:
            raise RuntimeError("episode-boundary preflight requires a prepared provider")
        for trial_index in range(case.trials):
            validate_episode_activation(
                self._episode_provider,
                episode,
                episode.initial_sample_index + trial_index,
            )

    def _connect_dimos(self, required_modules: tuple[str, ...], timeout_s: float) -> Dimos:
        from dimos.porcelain.dimos import Dimos

        deadline = time.monotonic() + timeout_s
        last_error: BaseException | None = None
        while time.monotonic() < deadline:
            if self._proc is not None:
                process = self._proc.process
                if process is not None and process.poll() is not None:
                    raise RuntimeError(
                        f"DimOS exited during startup with code {process.returncode}"
                    )
            app: Dimos | None = None
            try:
                app = Dimos.connect(timeout=2.0)
                for module_name in required_modules:
                    app.get_module(module_name)
                return app
            except Exception as error:
                last_error = error
                if app is not None:
                    app.stop()
                time.sleep(0.5)
        raise TimeoutError(f"DimOS modules were not ready after {timeout_s:.0f}s: {last_error}")

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
