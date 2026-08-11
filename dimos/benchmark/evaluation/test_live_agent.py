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

from pathlib import Path

from pytest_mock import MockerFixture

import dimos.benchmark.evaluation.live_agent as live_agent
from dimos.benchmark.evaluation.live_agent import LiveAgentRuntimeFactory
from dimos.benchmark.evaluation.models import RuntimeCondition
from dimos.benchmark.evaluation.pi_process import PiRunResult


def test_live_runtime_waits_for_start_and_records_selected_condition(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    events: list[str] = []

    class FakeServer:
        mcp_url = "http://127.0.0.1:1/mcp"

        def __init__(self, *, live_recording_path: str) -> None:
            assert live_recording_path.endswith("recording.db")

        def start(self) -> None:
            events.append("server.start")

        def stop(self) -> None:
            events.append("server.stop")

    class FakeRunner:
        def __init__(self, **kwargs: object) -> None:
            assert kwargs["model"] == "gpt-5.6-luna"
            assert kwargs["thinking_level"] == "high"

        def run(self, **kwargs: object) -> PiRunResult:
            events.append("pi.run")
            assert kwargs["prompt"] == "follow the route"
            return PiRunResult("done", 3, 2.0, None, "")

    marker = tmp_path / "marker"
    marker.touch()
    mocker.patch.object(live_agent, "CodePolicyMcpServer", FakeServer)
    mocker.patch.object(live_agent, "PiCliRunner", FakeRunner)
    mocker.patch.object(live_agent, "_pi_paths", return_value=(marker, marker))
    runtime = LiveAgentRuntimeFactory(
        api_key="secret",
        workspace=tmp_path,
        condition=RuntimeCondition(model="gpt-5.6-luna", thinking_level="high"),
    )

    execution = runtime.prepare(
        prompt="follow the route",
        system_prompt="use python_exec",
        memory_path=tmp_path / "recording.db",
        episode_timeout_s=300.0,
    )

    assert events == ["server.start"]
    assert runtime.identity.profile == "live-agent-v1"
    assert runtime.identity.thinking_level == "high"
    execution.start()
    outcome = execution.finish()
    assert outcome.tool_call_count == 3
    assert events == ["server.start", "pi.run", "server.stop"]
    assert {artifact.path for artifact in runtime.runtime_artifacts} >= {
        "runtime/live-agent/task-prompt.txt",
        "runtime/live-agent/live-agent.json",
    }
