# Copyright 2025-2026 Dimensional Inc.
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

"""Phase 3 checks for repeated provider-neutral InteractiveEval trials."""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

import pytest

import dimos.e2e_tests.dimos_cli_call as cli_call_module
from dimos.e2e_tests.episode import start_episode
from dimos.e2e_tests.test_episode_provider_lifecycle import (
    _digest,
    _evaluation_case,
    _prepared_episode,
    _Provider,
)
from dimos.evals.runner import EvalRunner
from dimos.evals.types import InteractiveEval
import dimos.sim2.evaluation as evaluation_module
from dimos.sim2.evaluation import EpisodeBoundary, TrialIsolationMode


class _App:
    def __init__(self) -> None:
        self.stopped = False

    def list_rpcs(self) -> list[Any]:
        return []

    def stop(self) -> None:
        self.stopped = True


class _Process:
    instances: list[_Process] = []

    def __init__(self) -> None:
        self.simulator: str | None = None
        self.global_args: list[str] = []
        self.extra_env: dict[str, str] = {}
        self.demo_args: list[str] = []
        self.started = False
        self.stopped = False
        self.process = None
        self.instances.append(self)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True


class _TrialRunner(EvalRunner):
    def __init__(self, provider: _Provider) -> None:
        super().__init__()
        self.provider = provider
        self.setup_calls: list[int | None] = []
        self.teardown_calls: list[int] = []
        self.boundaries: list[EpisodeBoundary] = []
        self.context_names: list[str] = []
        self.current_trial = -1

    def setup_env(self, case: InteractiveEval) -> None:
        self._setup_episode_env(case)

    def _setup_episode_env(
        self,
        case: InteractiveEval,
        *,
        sample_index: int | None = None,
    ) -> None:
        self.setup_calls.append(sample_index)
        episode = self.provider.prepare(
            _evaluation_case(),
            Path("unused"),
            sample_index=sample_index,
        )
        self._episode_provider = self.provider
        self._episode = episode
        activation = start_episode(self.provider, episode)
        self._episode_context = activation.context
        self._episode_sample_index = activation.sample_index
        self._episode_activation = activation
        self._episode_boundary_sequence = 0
        self._publish_episode_boundary(activation.boundary)

    def _publish_episode_boundary(self, boundary: EpisodeBoundary) -> None:
        expected = self._episode_boundary_sequence + 1
        if boundary.sequence != expected:
            raise ValueError(
                f"episode boundary sequence is {boundary.sequence}; expected {expected}"
            )
        self._episode_boundary_sequence = boundary.sequence
        self.boundaries.append(boundary)

    def run_action(self, action: Any) -> str:
        if self._episode_context is None:
            raise RuntimeError("test runner has no public episode context")
        self.context_names.append(self._episode_context.role("object").name)
        return action(cast("Any", None), self._episode_context)

    def sample_episode(self, _interval: float, _timeout: float) -> tuple[Any, Any]:
        if self._episode is None:
            raise RuntimeError("test runner has no prepared episode")
        evaluation = self.provider.evaluate(self._episode)
        return [(0.0, 1.0 if evaluation.passed else 0.0)], evaluation

    def _teardown_trial(self, case: InteractiveEval, trial_index: int) -> None:
        self.teardown_calls.append(trial_index)
        super()._teardown_trial(case, trial_index)


def _case(
    *,
    trials: int = 3,
    isolation: TrialIsolationMode = TrialIsolationMode.EPISODE_BOUNDARY,
    action: Any | None = None,
) -> InteractiveEval:
    return InteractiveEval(
        id="repeated-distribution",
        blueprint="robot-sim",
        episode=_evaluation_case().episode_request,
        action=((lambda _app, context: context.role("object").name) if action is None else action),
        required_roles=("object",),
        interval_s=0.0,
        timeout_s=0.0,
        trials=trials,
        trial_aggregate=lambda values: sum(values) / len(values),
        trial_isolation=isolation,
    )


def _production_runner(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    provider: _Provider,
) -> tuple[EvalRunner, list[_Process]]:
    _Process.instances = []
    monkeypatch.setattr(evaluation_module, "load_episode_provider", lambda _name: provider)
    monkeypatch.setattr(cli_call_module, "DimosCliCall", _Process)
    runner = EvalRunner(out_dir=tmp_path)
    runner._run_dir = tmp_path
    app = _App()
    runner._connect_dimos = lambda _modules, _timeout: cast("Any", app)
    return runner, _Process.instances


def test_runner_prepares_and_starts_once_then_activates_each_sample(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    provider = _Provider()
    runner, processes = _production_runner(monkeypatch, tmp_path, provider)

    result = runner.run_interactive_trials(_case())

    assert provider.prepare_calls == [None]
    assert provider.start_calls == 1
    assert provider.activate_calls == [1, 2]
    assert len(processes) == 1
    assert processes[0].started is True
    assert tuple(trial.sample_index for trial in result.trials) == (0, 1, 2)


def test_each_trial_receives_only_its_fresh_public_context() -> None:
    runner = _TrialRunner(_Provider())

    result = runner.run_interactive_trials(_case())

    assert runner.context_names == ["object 0", "object 1", "object 2"]
    assert tuple(trial.outputs for trial in result.trials) == tuple(runner.context_names)


def test_private_oracle_scores_each_trial_and_trial_aggregate_scores_case() -> None:
    runner = _TrialRunner(_Provider())

    result = runner.run_interactive_trials(_case())

    assert tuple(trial.score for trial in result.trials) == (1.0, 0.0, 1.0)
    assert tuple(trial.oracle for trial in result.trials) == (
        "private outcome for sample 0",
        "private outcome for sample 1",
        "private outcome for sample 2",
    )
    assert result.score == pytest.approx(2.0 / 3.0)
    assert result.metrics == {"sample_index": 1.0}


def test_trial_errors_are_retained_without_losing_other_trial_results() -> None:
    def fail_middle(_app: Any, context: Any) -> str:
        if context.role("object").entity_id == "object-1":
            raise RuntimeError("sample-specific action failure")
        return context.role("object").name

    runner = _TrialRunner(_Provider())

    result = runner.run_interactive_trials(_case(action=fail_middle))

    assert len(result.trials) == 3
    assert result.trials[0].error == ""
    assert "sample-specific action failure" in result.trials[1].error
    assert result.trials[2].error == ""
    assert "sample-specific action failure" in result.error


def test_each_trial_retains_compact_exact_replay_identity() -> None:
    runner = _TrialRunner(_Provider())

    result = runner.run_interactive_trials(_case())

    for index, trial in enumerate(result.trials):
        assert trial.sample_digest == _digest(index)
        assert trial.provenance == {
            "distribution_digest": "a" * 64,
            "sample_index": index,
            "sample_digest": _digest(index),
        }
        assert not ({"path", "cache", "scenario"} & trial.provenance.keys())


def test_process_isolation_is_explicit_and_never_selected_implicitly() -> None:
    warm_runner = _TrialRunner(_Provider())
    process_runner = _TrialRunner(_Provider())

    warm_runner.run_interactive_trials(_case())
    process_runner.run_interactive_trials(_case(isolation=TrialIsolationMode.PROCESS))

    assert warm_runner.setup_calls == [None]
    assert warm_runner.teardown_calls == []
    assert process_runner.setup_calls == [None, 1, 2]
    assert process_runner.teardown_calls == [0, 1, 2]


def test_episode_boundary_preflights_every_sample_before_process_launch() -> None:
    provider = _Provider()
    runner = EvalRunner()
    episode = _prepared_episode()
    runner._episode_provider = provider

    runner._preflight_episode_boundary(_case(), episode)

    assert provider.validate_calls == [0, 1, 2]
    assert runner._proc is None
    assert provider.active_sample_index is None


def test_episode_boundary_rejects_cross_topology_sample_before_process_launch() -> None:
    provider = _Provider()
    provider.rejected_samples.add(2)
    runner = EvalRunner()
    runner._episode_provider = provider

    with pytest.raises(ValueError, match="requires another topology"):
        runner._preflight_episode_boundary(_case(), _prepared_episode())

    assert provider.validate_calls == [0, 1, 2]
    assert runner._proc is None
    assert provider.start_calls == 0


def test_process_isolation_prepares_each_requested_sample_index() -> None:
    provider = _Provider()
    runner = _TrialRunner(provider)

    result = runner.run_interactive_trials(_case(isolation=TrialIsolationMode.PROCESS))

    assert provider.prepare_calls == [None, 1, 2]
    assert provider.start_calls == 3
    assert provider.activate_calls == []
    assert tuple(trial.sample_index for trial in result.trials) == (0, 1, 2)


def test_runner_consumes_provider_start_activation_without_duplicate_activate() -> None:
    provider = _Provider()
    runner = _TrialRunner(provider)

    result = runner.run_interactive_trials(_case(trials=1))

    assert provider.start_calls == 1
    assert provider.activate_calls == []
    assert result.trials[0].sample_index == 0
    assert result.trials[0].sample_digest == _digest(0)


def test_runner_publishes_one_boundary_for_the_startup_activation() -> None:
    runner = _TrialRunner(_Provider())

    runner.run_interactive_trials(_case(trials=1))

    assert len(runner.boundaries) == 1
    assert runner.boundaries[0].previous_sample_index is None
    assert runner.boundaries[0].sample_index == 0
    assert runner.boundaries[0].sequence == 1
