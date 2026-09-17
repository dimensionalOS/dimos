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

"""A live simulator driven through ``dimos run``."""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Sequence
import math
from pathlib import Path
import socket
import time
from typing import TYPE_CHECKING, Any

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.constants import RECORDINGS_DIR
from dimos.core.run_registry import list_runs
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.base import Environment
from dimos.evals.environments.lib.launch import default_mcp_url, validate_blueprints
from dimos.evals.types import RunningEnvironment
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.raw_robot_bridge import RAW_ENDPOINT

if TYPE_CHECKING:
    from dimos.evals.agents.base import Agent
    from dimos.memory.store.base import Store
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class SimConfig(BaseConfig):
    blueprint: list[str]
    # Module registry names to disable in the composed blueprint.
    disable: tuple[str, ...] = ()
    # Also expose the robot as plain Zenoh topics (raw-robot-bridge) for agents without dimOS.
    raw_bridge: bool = False
    attach: bool = False
    launch_timeout_s: float = 1200.0
    at_rest_m: float = 0.05
    at_rest_s: float = 2.0
    settle_poll_s: float = 0.5


class Sim(Environment):
    """A live simulator composed from blueprint names shared by every evaluated agent.

    Agent modules extend that composition. ``attach`` uses an existing dimos
    instead, which must have been started with ``--record``.
    """

    has_robot = True

    config: SimConfig

    @property
    def provides_raw_robot(self) -> bool:
        return self.config.raw_bridge

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._recording: Store | None = None
        self._raw_endpoint = RAW_ENDPOINT  # an attached dimos runs its bridge on the default

    @abstractmethod
    def configure_launch(self, proc: DimosCliCall) -> None:
        """Set simulator flags/environment on the owned process before launch."""

    def setup_scene(self) -> None:
        """Apply optional simulator setup after MCP is ready, before recording discovery."""

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        """Wait for simulator observations and return extra artifacts, if needed."""
        return {}

    @abstractmethod
    def latest_pose(self, recording: Store) -> PoseStamped:
        """Return achieved pose for settling; raise LookupError before the first sample."""

    def preflight(self, agent: Agent) -> None:
        if self.config.attach:
            if agent.config.modules:
                raise RuntimeError(
                    "Sim attaches to an existing dimos; "
                    f"{type(agent).__name__} also adds modules {agent.config.modules!r}"
                )
            mcp_url = default_mcp_url()
            if not McpAdapter(mcp_url).wait_for_ready(timeout=2.0):
                raise RuntimeError(f"attach needs a running dimos at {mcp_url}")
            return
        bridge = ["raw-robot-bridge"] if self.config.raw_bridge else []
        validate_blueprints(
            (*self.config.blueprint, *agent.config.modules, *bridge, *self.config.disable)
        )

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        # SQLite memory codecs are only needed for a running simulator.
        from dimos.memory.store.sqlite import SqliteStore

        deadline = time.monotonic() + self.config.launch_timeout_s
        pid = None
        proc = None
        if self.config.attach and self.config.raw_bridge and not _listening(self._raw_endpoint):
            raise RuntimeError(
                f"attach with raw_bridge needs a running raw-robot-bridge at {self._raw_endpoint}"
            )
        if not self.config.attach:
            proc = DimosCliCall()
            self.configure_launch(proc)
            proc.global_args.append("--record")
            disabled = [arg for name in self.config.disable for arg in ("--disable", name)]
            bridge = ["raw-robot-bridge"] if self.config.raw_bridge else []
            if self.config.raw_bridge:
                self._raw_endpoint = f"tcp/127.0.0.1:{_free_port()}"  # one bridge per run
                proc.extra_env["RAWROBOTBRIDGE__ENDPOINT"] = self._raw_endpoint
            proc.demo_args = ["run", *self.config.blueprint, *modules, *bridge, *disabled]
            self._resources.callback(proc.stop)
            proc.start()
            assert proc.process is not None
            pid = proc.process.pid
        mcp_url = default_mcp_url()
        adapter = McpAdapter(mcp_url)
        if proc is not None:
            process = proc.process
            assert process is not None
            while not adapter.wait_for_ready(
                timeout=min(1.0, max(0.0, deadline - time.monotonic()))
            ):
                if process.poll() is not None:
                    raise RuntimeError(
                        f"Simulator eval process exited with code {process.returncode}"
                    )
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"MCP at {mcp_url} not ready before simulator launch deadline"
                    )
        elif not adapter.wait_for_ready(timeout=self.config.launch_timeout_s, interval=2.0):
            raise RuntimeError(f"MCP at {mcp_url} not ready — is dimos up?")
        self.setup_scene()
        path = self._wait_recording(deadline, pid)
        self._recording = SqliteStore(path=str(path), must_exist=True)
        self._resources.callback(self._recording.stop)
        artifacts = {"recording": path}
        artifacts.update(self.prepare_recording(self._recording, path, deadline))
        return RunningEnvironment(
            mcp_url=mcp_url,
            streams=(),
            artifacts=artifacts,
            raw_endpoint=self._raw_endpoint if self.config.raw_bridge else None,
        )

    def _wait_recording(self, deadline: float, pid: int | None) -> Path:
        """Find the recording of the launched process, or the attached dimos."""
        while True:
            runs = [run for run in list_runs(alive_only=True) if pid is None or run.pid == pid]
            if runs:
                path = RECORDINGS_DIR / runs[-1].run_id / "memory.db"
                if path.exists():
                    return path
            if time.monotonic() > deadline:
                where = f"pid {pid}" if pid is not None else "the attached dimos"
                raise TimeoutError(
                    f"no recording from {where} within the launch timeout; "
                    "an attached dimos must be started with --record"
                )
            time.sleep(1.0)

    def settle(self, budget_s: float) -> None:
        """Wait until the robot is at rest after skills that start asynchronous motion."""
        if self._recording is None:
            return
        anchor = None
        anchor_t = 0.0
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            try:
                pose = self.latest_pose(self._recording)
            except LookupError:
                return
            position = pose.position
            if (
                anchor is None
                or math.hypot(position.x - anchor.x, position.y - anchor.y) > self.config.at_rest_m
            ):
                anchor, anchor_t = position, time.monotonic()
            elif time.monotonic() - anchor_t >= self.config.at_rest_s:
                return
            time.sleep(self.config.settle_poll_s)

    def stop(self) -> None:
        try:
            super().stop()
        finally:
            self._recording = None


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _listening(endpoint: str) -> bool:
    host, port = endpoint.rsplit("/", 1)[1].rsplit(":", 1)
    with socket.socket() as s:
        s.settimeout(1.0)
        return s.connect_ex((host, int(port))) == 0
