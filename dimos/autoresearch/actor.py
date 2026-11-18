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

"""The actor run by the lab itself: Pi (``dimos.evals.agents.pi``) driving
the lab's tools over MCP with the autoresearch prompt."""

from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import sys
import threading

from dimos.autoresearch.prompt import EXCLUDED_KEYWORDS, actor_prompt
from dimos.autoresearch.session import Session
from dimos.core.coordination.process_lifecycle import kill_run_processes
from dimos.evals.agents.lib.pi_config import RunPaths
from dimos.evals.agents.pi import PiAdapter
from dimos.evals.constants import PROVIDERS
from dimos.evals.types import RunningEnvironment, Trajectory

# pi.py tags the Pi process and everything it starts with this variable, set to the run directory.
RUN_ID_ENV = "DIMOS_EVAL_RUN_ID"
# How often the stop watcher looks at the stop event, seconds.
STOP_POLL_S = 0.5
# Where the dimos CLI lives: this interpreter's bin directory, which a stack
# launched by Desktop (a service environment) does not have on its PATH.
INTERPRETER_BIN = Path(sys.executable).parent


def _pi_path() -> str:
    """The PATH the Pi process gets: the interpreter's bin directory first, so ``dimos`` resolves."""
    return os.pathsep.join([str(INTERPRETER_BIN), os.environ.get("PATH", "")])


class Actor(PiAdapter):
    """Pi whose process finds the dimos CLI beside this interpreter."""

    def _build_process_env(self, paths: RunPaths) -> dict[str, str]:
        env = super()._build_process_env(paths)
        env["PATH"] = _pi_path()
        return env


def provider_key(provider: str) -> str:
    """The environment variable that holds ``provider``'s API key, for example
    ``ANTHROPIC_API_KEY`` for ``"anthropic"``. Refuses a provider that
    ``dimos.evals.constants.PROVIDERS`` does not list."""
    if provider not in PROVIDERS:
        raise ValueError(f"provider must be one of {sorted(PROVIDERS)}, not {provider!r}")
    return PROVIDERS[provider][0]


def run_actor(
    mcp_url: str,
    session: Session,
    model: str,
    provider: str = "anthropic",
    max_steps: int = 600,
    *,
    timeout_s: float,
    run_dir: Path,
    stop: threading.Event,
) -> Trajectory:
    """Run Pi on the session's task against the lab at ``mcp_url``, with at
    most ``max_steps`` model calls in ``timeout_s`` seconds, and return its
    trajectory, also written to ``run_dir/trajectory.json``. Setting ``stop``
    kills the Pi process, and the run returns what it has. Raises before
    launching anything when the provider's key is not in the environment or
    the ``pi`` command is not on PATH or ``dimos`` is on neither PATH nor
    this interpreter's bin directory."""
    key = provider_key(provider)
    if not os.environ.get(key):
        raise RuntimeError(f"{key} is not set: put it in the .env of the dimos directory")
    agent = Actor(
        provider=provider, model=model, max_steps=max_steps, excluded_keywords=EXCLUDED_KEYWORDS
    )
    if shutil.which(agent.config.cli) is None:
        raise RuntimeError(
            f"{agent.config.cli!r} is not on PATH (npm install -g @earendil-works/pi-coding-agent)"
        )
    if shutil.which("dimos", path=_pi_path()) is None:
        raise RuntimeError(
            f"Pi reaches the lab through the dimos CLI, which is on neither PATH nor {INTERPRETER_BIN}"
        )
    info = session.info()
    env = RunningEnvironment(mcp_url=mcp_url, streams=(), artifacts={"session": session.dir})
    run_dir.mkdir(parents=True, exist_ok=True)
    ended = threading.Event()
    watcher = threading.Thread(
        target=_kill_on_stop, args=(stop, ended, run_dir), name="autoresearch-actor-stop"
    )
    watcher.start()
    try:
        trajectory = agent.run(
            actor_prompt(info.task, info.budget), env, run_dir, timeout_s=timeout_s
        )
    finally:
        ended.set()
        watcher.join()
    (run_dir / "trajectory.json").write_text(json.dumps(asdict(trajectory), indent=2, default=str))
    return trajectory


def _kill_on_stop(stop: threading.Event, ended: threading.Event, run_dir: Path) -> None:
    """Kill Pi and whatever it started while ``stop`` is set, until the run has ended."""
    while not ended.is_set():
        if stop.wait(STOP_POLL_S):
            kill_run_processes(str(run_dir), env_var=RUN_ID_ENV)
