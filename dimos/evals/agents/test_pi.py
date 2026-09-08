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

"""Check Pi's CLI configuration and recording export without launching an agent."""

from pathlib import Path

from dimos.evals.agents.pi import PiAdapter, recording_file
from dimos.evals.cli import load_agent
from dimos.evals.types import RunningEnvironment
from dimos.memory.store.sqlite import SqliteStore


def test_cli_overrides_reach_pi_adapter() -> None:
    agent = load_agent(
        "dimos.evals.agents.pi",
        ["model=eval-model", "max_steps=3", 'modules=["rangefinder-skill"]'],
    )

    assert isinstance(agent, PiAdapter)
    assert agent.config.model == "eval-model"
    assert agent.config.max_steps == 3
    assert agent.config.modules == ("rangefinder-skill",)


def test_zero_budget_returns_without_starting_pi(tmp_path: Path) -> None:
    agent = PiAdapter(max_steps=0, cli=str(tmp_path / "pi-not-installed"))
    environment = RunningEnvironment(mcp_url="", streams=(), artifacts={})

    trajectory = agent.run("Answer the question.", environment, tmp_path, timeout_s=10)

    assert trajectory.extra.ended_by == "max_steps"
    assert [step.source for step in trajectory.steps] == ["user"]
    assert list(tmp_path.iterdir()) == []


def test_recording_export_contains_only_selected_data(dataset: str, tmp_path: Path) -> None:
    """Pi must receive the case's selected observations, not the full recording."""
    exported = tmp_path / "selected.db"
    with SqliteStore(path=dataset, must_exist=True) as source:
        source.stream("excluded", str).append("not selected", ts=1000.0)
        recording_file((source.streams.odom.limit(2),), exported)

    with SqliteStore(path=str(exported), must_exist=True) as copy:
        assert copy.list_streams() == ["odom"]
        assert [(obs.ts, obs.data.position.x) for obs in copy.streams.odom] == [
            (1000.0, 0.0),
            (1001.0, 1.0),
        ]
