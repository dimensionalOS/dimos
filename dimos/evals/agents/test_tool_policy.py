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

"""The same provider and tool contract across native Pi and dimcode runtimes."""

import json
from pathlib import Path
from urllib.parse import urlsplit

import pytest

from dimos.evals.agents.conftest import NativeHarness, ScriptedProvider
from dimos.evals.agents.lib.pi_config import RunPaths


def test_allowed_tools_execute_and_excluded_tools_do_not(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    provider.call("write", path=harness.path("forbidden.txt"), content="must not execute")
    provider.call("bash", command=f"printf selected-observation > {harness.path('facts.txt')}")
    provider.call("grep", pattern="selected-observation", path=harness.path("facts.txt"), context=1)
    result = harness.run(harness.agent(provider, ("bash", "grep")))
    assert result.extra.ended_by == "answer", result.extra
    assert result.final_answer == "OK"
    assert len(provider.requests) == 4
    endpoint, output_cap = {
        "openai": ("/responses", "max_output_tokens"),
        "anthropic": ("/v1/messages", "max_tokens"),
    }[provider.name]
    assert {urlsplit(route).path for route in provider.routes} == {endpoint}
    for request in provider.requests:
        assert request["model"] == provider.model
        assert request[output_cap] == 1024
        tools = request["tools"]
        assert isinstance(tools, list)
        names = set()
        for tool in tools:
            assert isinstance(tool, dict)
            names.add(str(tool["name"]))
        assert names == {"bash", "grep"}
    assert (harness.workspace / "facts.txt").read_text() == "selected-observation"
    assert not (harness.workspace / "forbidden.txt").exists()
    observation = result.steps[-2].observation
    assert observation is not None
    output = observation.results[0].content
    assert "selected-observation" in output
    assert json.dumps(output) in json.dumps(provider.requests[-1])
    assert result.final_metrics.total_prompt_tokens == 40
    assert result.final_metrics.total_completion_tokens == 20
    assert result.final_metrics.total_cost_usd is not None


def test_no_tools_blocks_even_a_provider_requested_call(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    provider.call("write", path=harness.path("forbidden.txt"), content="must not execute")
    result = harness.run(harness.agent(provider, ()))
    assert result.extra.ended_by == "answer", result.extra
    assert all(request.get("tools", []) == [] for request in provider.requests)
    assert not (harness.workspace / "forbidden.txt").exists()


@pytest.mark.parametrize("harness", ["dimcode"], indirect=True)
def test_unknown_tool_fails_before_inference(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    result = harness.run(harness.agent(provider, ("unknown_tool",)))
    assert result.extra.ended_by == "error"
    assert "unknown_tool" in result.extra.error
    assert provider.requests == []


def test_missing_runtime_extension_fails_before_inference(
    harness: NativeHarness,
    provider: ScriptedProvider,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    agent = harness.agent(provider, ("bash",))
    build = agent._build_pi_command
    # A previous run's marker must not let a missing extension bypass startup checks.
    ready = RunPaths.for_run(harness.root).config / "extensions/runtime-ready.json"
    ready.parent.mkdir(parents=True)
    ready.write_text('{"tools":["bash"],"unknown":[]}')

    def broken(inputs: str, prompt: str, paths: RunPaths) -> list[str]:
        command = build(inputs, prompt, paths)
        (paths.config / "extensions/runtime.js").unlink()
        return command

    monkeypatch.setattr(agent, "_build_pi_command", broken)
    result = harness.run(agent)
    assert result.extra.ended_by == "error"
    assert provider.requests == []


@pytest.mark.parametrize("harness", ["sandbox"], indirect=True)
def test_sandbox_grep_cannot_read_host_files(
    harness: NativeHarness, provider: ScriptedProvider, tmp_path: Path
) -> None:
    secret = tmp_path / "host-only.txt"
    secret.write_text("host-only-content")
    provider.call("grep", pattern="host-only-content", path=str(secret), context=1)
    result = harness.run(harness.agent(provider, ("grep",)))
    assert result.extra.ended_by == "answer", result.extra
    observation = result.steps[-2].observation
    assert observation is not None
    output = observation.results[0].content
    assert "No such file or directory" in output
    assert "exited with code 2" in output
    assert "host-only-content" not in output
    assert json.dumps(output) in json.dumps(provider.requests[-1])


@pytest.mark.parametrize("harness", ["sandbox"], indirect=True)
@pytest.mark.parametrize("provider", ["openai"], indirect=True)
def test_sandbox_grep_treats_shell_metacharacters_as_data(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    provider.call(
        "grep", pattern="$(touch /workspace/forbidden) ' \" ;", literal=True, path="/input"
    )
    result = harness.run(harness.agent(provider, ("grep",)))
    assert result.extra.ended_by == "answer", result.extra
    assert not (harness.workspace / "forbidden").exists()
