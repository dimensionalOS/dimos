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

"""Phase 3 checks for stateful-module reset at warm episode boundaries."""

from types import SimpleNamespace
from typing import Any, cast

from dimos.e2e_tests.episode import start_episode
from dimos.e2e_tests.test_episode_provider_lifecycle import (
    _evaluation_case,
    _prepared_episode,
    _Provider,
)
from dimos.evals.runner import EvalRunner
from dimos.evals.types import InteractiveEval
from dimos.sim2.evaluation import EpisodeBoundary


def test_boundary_contains_provider_episode_sample_and_monotonic_sequence() -> None:
    runner = EvalRunner()
    first = EpisodeBoundary(
        provider_name="fake",
        episode_id="episode-1",
        previous_sample_index=None,
        sample_index=0,
        sequence=1,
    )
    second = EpisodeBoundary(
        provider_name="fake",
        episode_id="episode-1",
        previous_sample_index=0,
        sample_index=1,
        sequence=2,
    )

    runner._publish_episode_boundary(first)
    runner._publish_episode_boundary(second)

    assert runner._episode_boundary_sequence == 2
    assert second.to_wire_dict() == {
        "provider_name": "fake",
        "episode_id": "episode-1",
        "previous_sample_index": 0,
        "sample_index": 1,
        "sequence": 2,
    }


def test_runner_publishes_boundary_after_activation_before_action() -> None:
    provider = _Provider()
    episode = _prepared_episode()
    startup = start_episode(provider, episode)
    runner = EvalRunner()
    runner._episode_provider = provider
    runner._episode = episode
    runner._episode_context = startup.context
    runner._episode_sample_index = startup.sample_index
    runner._episode_activation = startup
    runner._episode_boundary_sequence = startup.boundary.sequence
    events = provider.events
    events.clear()
    runner._publish_episode_boundary = lambda boundary: events.append(
        f"boundary:{boundary.sample_index}"
    )

    def run_action(action: Any) -> str:
        events.append("action")
        return action(cast("Any", None), runner._episode_context)

    runner.run_action = run_action
    runner.sample_episode = lambda _interval, _timeout: (
        [(0.0, 1.0)],
        provider.evaluate(episode),
    )
    case = InteractiveEval(
        id="boundary-order",
        blueprint="robot-sim",
        episode=_evaluation_case().episode_request,
        action=lambda _app, context: context.role("object").name,
        required_roles=("object",),
        trials=2,
    )

    result = runner._run_interactive_trial(case, 1)

    assert result.error == ""
    assert events == ["activate:1", "boundary:1", "action"]


def test_registered_stateful_modules_receive_each_boundary_once() -> None:
    received: dict[str, list[EpisodeBoundary]] = {"planner": [], "memory": []}

    class Listener:
        def __init__(self, name: str) -> None:
            self.name = name

        def on_episode_boundary(self, boundary: EpisodeBoundary) -> None:
            received[self.name].append(boundary)

    modules = {name: Listener(name) for name in received}
    app = SimpleNamespace(
        list_rpcs=lambda: [
            SimpleNamespace(name="on_episode_boundary", module_name="planner"),
            SimpleNamespace(name="on_episode_boundary", module_name="memory"),
            SimpleNamespace(name="unrelated", module_name="planner"),
        ],
        get_module=modules.__getitem__,
    )
    runner = EvalRunner()
    runner._app = cast("Any", app)
    boundaries = (
        EpisodeBoundary(
            provider_name="fake",
            episode_id="episode-1",
            previous_sample_index=None,
            sample_index=0,
            sequence=1,
        ),
        EpisodeBoundary(
            provider_name="fake",
            episode_id="episode-1",
            previous_sample_index=0,
            sample_index=1,
            sequence=2,
        ),
    )

    for boundary in boundaries:
        runner._publish_episode_boundary(boundary)

    assert received == {
        "planner": list(boundaries),
        "memory": list(boundaries),
    }
