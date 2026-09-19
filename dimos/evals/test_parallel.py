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

from dimos.evals.agents.pi import robot_readme
from dimos.evals.parallel import _job_command
from dimos.evals.suites.examples import SUITE


def test_job_command_on_host_and_in_container(tmp_path: Path, monkeypatch) -> None:
    case = SUITE[0]
    job = tmp_path / f"{case.id}-1"
    cmd, env = _job_command("s.m", "a.m", ["model=x"], case, job, "", "run1")
    assert cmd[:3] == ["dimos", "evals", "run"]
    assert cmd[-4:] == ["--case", case.id, "--set", "model=x"]
    assert env is not None and env["XDG_STATE_HOME"] == str(job / "state")
    monkeypatch.setenv("OPENAI_API_KEY", "k")
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    cmd, env = _job_command("s.m", "a.m", [], case, job, "img", "run1")
    assert env is None and cmd[:2] == ["docker", "run"] and "img" in cmd
    assert cmd.count("-e") == 2 and "OPENAI_API_KEY" in cmd and "ANTHROPIC_API_KEY" not in cmd
    assert f"{job}:/out" in cmd and cmd[cmd.index("--label") + 1] == "dimos-eval=run1"


def test_robot_readme_lists_only_served_topics() -> None:
    text = robot_readme("tcp/127.0.0.1:1", ("world_state", "cmd_vel", "finished"))
    assert "robot/world_state/json" in text and "robot/finished/json" in text
    assert "robot/camera/jpeg" not in text and "{" not in text.split("robot/cmd_vel")[0]
    assert "max 2" in text and "1 m/s" in text
