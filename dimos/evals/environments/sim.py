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

from collections.abc import Callable, Sequence
import math
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.constants import RECORDINGS_DIR
from dimos.core.run_registry import list_runs
from dimos.e2e_tests.dim_sim_client import DimSimClient
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.base import Environment
from dimos.evals.environments.lib.launch import default_mcp_url, validate_blueprints
from dimos.evals.types import RunningEnvironment
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.evals.agents.base import Agent
    from dimos.memory.store.base import Store


class SimConfig(BaseConfig):
    blueprint: list[str]
    simulator: str = "dimsim"
    scene: str = "apartment"
    setup: Callable[[DimSimClient], None] | None = None
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

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._recording: Store | None = None

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
        validate_blueprints((*self.config.blueprint, *agent.config.modules))

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        # SQLite memory codecs are only needed for a running simulator.
        from dimos.memory.store.sqlite import SqliteStore

        deadline = time.monotonic() + self.config.launch_timeout_s
        pid = None
        if not self.config.attach:
            proc = DimosCliCall()
            proc.simulator = self.config.simulator
            proc.global_args = ["--dimsim-scene", self.config.scene, "--record"]
            proc.demo_args = ["run", *self.config.blueprint, *modules]
            self._resources.callback(proc.stop)
            proc.start()
            assert proc.process is not None
            pid = proc.process.pid
        mcp_url = default_mcp_url()
        if not McpAdapter(mcp_url).wait_for_ready(
            timeout=self.config.launch_timeout_s, interval=2.0
        ):
            raise RuntimeError(f"MCP at {mcp_url} not ready — is dimos up?")
        if self.config.setup is not None:
            sim = DimSimClient()
            self._resources.callback(sim.stop)
            sim.start()
            self.config.setup(sim)
        path = self._wait_recording(deadline, pid)
        self._recording = SqliteStore(path=str(path), must_exist=True)
        self._resources.callback(self._recording.stop)
        return RunningEnvironment(mcp_url=mcp_url, streams=(), artifacts={"recording": path})

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
        if self._recording is None or "odom" not in self._recording.streams:
            return
        odom = self._recording.streams.odom
        anchor = None
        anchor_t = 0.0
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            try:
                observation = odom.last()
            except LookupError:
                return
            position = observation.data.position
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
