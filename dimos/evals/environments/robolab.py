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

"""NVIDIA RoboLab tasks, run live by RoboLab in Isaac Sim with dimos driving its robot.

Each case starts RoboLab's evaluation of one task with the dimos runner
(dimos/simulation/robolab/runner), then ``dimos run`` on the RoboLab blueprints. RoboLab
judges the episode itself and writes its results row; that file is the grader's
``robolab_results`` artifact and is never given to the agent.

RoboLab runs in its own Python (Isaac Sim 5.0, Python 3.11). ``ROBOLAB_PYTHON`` names that
interpreter (``/workspace/isaaclab/_isaac_sim/python.sh`` in RoboLab's image) and
``ROBOLAB_DIR`` its checkout; zenoh and msgpack must be importable there.
"""

from __future__ import annotations

from collections.abc import Sequence
import contextlib
from dataclasses import replace
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from typing import TYPE_CHECKING

from pydantic import Field

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.evals.environments.sim import Sim, SimConfig
from dimos.evals.types import RunningEnvironment
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store

logger = setup_logger()

RUNNER = DIMOS_PROJECT_ROOT / "dimos" / "simulation" / "robolab" / "runner"


class RoboLabEnvironmentConfig(SimConfig):
    task: str
    robolab_python: str = Field(
        default_factory=lambda: os.environ.get(
            "ROBOLAB_PYTHON", "/workspace/isaaclab/_isaac_sim/python.sh"
        )
    )
    robolab_dir: str = Field(
        default_factory=lambda: os.environ.get("ROBOLAB_DIR", "/workspace/robolab")
    )
    # Extra entries for the runner's PYTHONPATH, e.g. where zenoh and msgpack are installed.
    robolab_pythonpath: str = Field(
        default_factory=lambda: os.environ.get("ROBOLAB_PYTHONPATH", "")
    )
    settle_on_joints: bool = True
    module_env: dict[str, str] = Field(default_factory=dict)
    ready_streams: tuple[str, ...] = ("color_image", "coordinator_joint_state")
    recorded_topics: tuple[str, ...] = (
        "color_image",
        "camera_info",
        "coordinator_joint_state",
        "tf",
    )
    # After the agent: time for RoboLab to settle the scene and write its results.
    results_timeout_s: float = 300.0


class RoboLabEnvironment(Sim):
    """One RoboLab episode of ``task``, judged by RoboLab."""

    config: RoboLabEnvironmentConfig

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._runner: subprocess.Popen[bytes] | None = None
        self._endpoint = ""
        self._out = Path()

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = "robolab"
        proc.global_args = ["--record-topics", ",".join(self.config.recorded_topics)]
        proc.extra_env["ROBOLAB_ENDPOINT"] = self._endpoint
        proc.extra_env.update(self.config.module_env)

    def start(self, modules: Sequence[str], *, requires_mcp: bool = False) -> RunningEnvironment:
        self._out = Path(tempfile.mkdtemp(prefix=f"robolab-{self.config.task}-"))
        self._endpoint = f"tcp/127.0.0.1:{_free_port()}"
        self._start_runner()
        running = super().start(modules, requires_mcp=requires_mcp)
        artifacts = {
            **running.artifacts,
            "robolab_results": self._out / "results" / "episode_results.jsonl",
            "robolab_log": self._out / "runner.log",
        }
        return replace(running, artifacts=artifacts)

    def _start_runner(self) -> None:
        pythonpath = os.pathsep.join(p for p in (str(RUNNER), self.config.robolab_pythonpath) if p)
        env = {
            **os.environ,
            "PYTHONPATH": pythonpath,
            "OMNI_KIT_ACCEPT_EULA": "YES",
            "ACCEPT_EULA": "Y",
        }
        command = [
            *self.config.robolab_python.split(),
            str(RUNNER / "run.py"),
            "--headless",
            "--task",
            self.config.task,
            "--num-runs",
            "1",
            "--listen",
            self._endpoint,
            # An absolute folder name is used as is.
            "--output-folder-name",
            str(self._out / "results"),
        ]
        log = (self._out / "runner.log").open("wb")
        self._resources.callback(log.close)
        self._runner = subprocess.Popen(
            command,
            cwd=self.config.robolab_dir,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._resources.callback(self._stop_runner)
        logger.info("RoboLab runner started", task=self.config.task, endpoint=self._endpoint)

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        while time.monotonic() < deadline:
            if self._runner is not None and self._runner.poll() is not None:
                raise RuntimeError(f"RoboLab exited early; see {self._out / 'runner.log'}")
            try:
                ages = [
                    time.time() - getattr(recording.streams, name).last().data.ts
                    for name in self.config.ready_streams
                ]
            except (LookupError, AttributeError):
                ages = []
            if len(ages) == len(self.config.ready_streams) and all(age < 10.0 for age in ages):
                return {}
            time.sleep(0.2)
        raise TimeoutError(f"RoboLab did not publish fresh {', '.join(self.config.ready_streams)}")

    def settle(self, budget_s: float) -> None:
        """Let the arm come to rest, then let RoboLab finish the episode and judge it."""
        super().settle(budget_s)
        from dimos.simulation.robolab.protocol import open_session, pack_release

        session = open_session(self._endpoint, timeout_s=10.0)
        if session is not None:
            try:
                session.declare_publisher("robolab/cmd").put(pack_release())
                time.sleep(0.5)
            finally:
                session.close()
        runner = self._runner
        if runner is None:
            return
        try:
            runner.wait(timeout=self.config.results_timeout_s)
        except subprocess.TimeoutExpired:
            logger.warning("RoboLab did not finish the episode in time", task=self.config.task)

    def _stop_runner(self) -> None:
        runner, self._runner = self._runner, None
        if runner is None or runner.poll() is not None:
            return
        with contextlib.suppress(ProcessLookupError):
            os.killpg(runner.pid, signal.SIGINT)
        try:
            runner.wait(timeout=30)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(runner.pid, signal.SIGKILL)
            runner.wait()


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])
