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

"""The autoresearch lab in MuJoCo as an eval environment: one launch is one session."""

from __future__ import annotations

import json
from pathlib import Path
import re
import tempfile
import time
from typing import TYPE_CHECKING, Any

from dimos.autoresearch.session import Budget
from dimos.constants import STATE_DIR
from dimos.evals.environments.mujoco_sim import MujocoEnvironment, MujocoEnvironmentConfig

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store


class AutoresearchEnvironmentConfig(MujocoEnvironmentConfig):
    task: str
    """The task in words, handed to the lab and the episode recorder."""
    budget: Budget
    """How much searching the session may do; the lab reads it from its config."""
    session_root: Path = STATE_DIR / "autoresearch"
    """Every launch makes a fresh session directory under here."""
    blueprint: list[str] = ["autoresearch-metal-sim"]
    ready_streams: tuple[str, ...] = ("coordinator_joint_state",)
    recorded_topics: tuple[str, ...] = ("coordinator_joint_state", "body_poses", "tf")
    agent_artifacts: tuple[str, ...] = ("session",)
    graceful_stop: bool = True


class AutoresearchEnvironment(MujocoEnvironment):
    """Launch the lab blueprint on a fresh session directory and hand that
    directory to the actor. Settling waits for the lab to score the frozen
    candidate on the held-out trials, so the grader finds ``session.json``
    at ``status: "scored"``."""

    config: AutoresearchEnvironmentConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session_dir: Path | None = None

    @property
    def session_dir(self) -> Path:
        if self._session_dir is None:
            raise RuntimeError("no session directory: the lab has not been launched")
        return self._session_dir

    def configure_launch(self, proc: DimosCliCall) -> None:
        super().configure_launch(proc)
        self._session_dir = new_session_dir(self.config.session_root, self.config.task)
        proc.extra_env.update(
            {
                "AUTORESEARCHMODULE__SESSION_DIR": str(self._session_dir),
                "AUTORESEARCHMODULE__TASK": self.config.task,
                "AUTORESEARCHMODULE__BUDGET": self.config.budget.model_dump_json(),
                "RECORDER__RECORDING": str(self._session_dir / "recording"),
                "EPISODES__TASK": self.config.task,
            }
        )

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        super().prepare_recording(recording, path, deadline)
        return {"session": self.session_dir}

    def settle(self, budget_s: float) -> None:
        """Wait up to ``budget_s`` seconds for ``session.json`` to say scored."""
        deadline = time.monotonic() + budget_s
        while True:
            status = session_status(self.session_dir)
            if status == "scored":
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"session {self.session_dir} is {status or 'missing'} after the "
                    f"{budget_s:.0f} s settle budget; the lab never reached scored"
                )
            time.sleep(self.config.settle_poll_s)

    def stop(self) -> None:
        try:
            super().stop()
        finally:
            self._session_dir = None


def new_session_dir(root: Path, task: str) -> Path:
    """Make an empty, uniquely named directory for one session under ``root``."""
    root.mkdir(parents=True, exist_ok=True)
    slug = re.sub(r"[^a-z0-9]+", "-", task.lower()).strip("-") or "session"
    return Path(tempfile.mkdtemp(prefix=f"{slug}-{time.strftime('%Y%m%d-%H%M%S')}-", dir=root))


def session_status(session_dir: Path) -> str:
    """The ``status`` in ``session.json``, or ``""`` while the file is absent or half written."""
    try:
        info = json.loads((session_dir / "session.json").read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return ""
    return str(info.get("status", ""))
