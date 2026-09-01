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

from collections.abc import Callable
from dataclasses import dataclass, field
import math
from pathlib import Path
import shutil
import time
from typing import TYPE_CHECKING, ClassVar

from dimos.evals.environments.lib.launch import blueprint_modules, default_mcp_url
from dimos.evals.types import Agent, RunningEnvironment

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store


def _no_setup(sim: DimSimClient) -> None:
    return None


@dataclass(kw_only=True)
class Sim:
    """A live simulator. ``start(modules)`` launches ``dimos --simulation
    <simulator> --dimsim-scene <scene> --record run <blueprint> <modules>``,
    waits for MCP, runs ``setup``, and hands out the recording ``--record``
    writes. ``blueprint`` is the world every agent gets — robot, mapping,
    ``McpServer``, motion skills; its skill containers are the tool set.
    ``attach`` drives an already-running dimos instead (it must have been
    started with ``--record``); ``blueprint`` is then a declaration."""

    blueprint: str
    simulator: str = "dimsim"
    scene: str = "apartment"
    setup: Callable[[DimSimClient], None] = _no_setup
    attach: bool = False
    launch_timeout_s: float = 1200.0  # blueprint + MCP readiness (e2e parity)
    at_rest_m: float = 0.05  # settle: at rest = moved less than this for at_rest_s
    at_rest_s: float = 2.0
    settle_poll_s: float = 0.5

    artifacts: ClassVar[tuple[str, ...]] = ("recording",)
    has_robot: ClassVar[bool] = True
    _proc: DimosCliCall | None = field(default=None, init=False, repr=False, compare=False)
    _sim: DimSimClient | None = field(default=None, init=False, repr=False, compare=False)
    _recording: Store | None = field(default=None, init=False, repr=False, compare=False)

    def preflight(self, agent: Agent) -> None:
        from dimos.agents.mcp.mcp_adapter import McpAdapter

        if self.attach:
            if not McpAdapter(default_mcp_url()).wait_for_ready(timeout=2.0):
                raise RuntimeError(f"attach needs a running dimos at {default_mcp_url()}")
            return
        if self.simulator == "dimsim" and shutil.which("deno") is None:
            raise RuntimeError("dimsim requires deno on PATH")
        blueprint_modules(f"{self.blueprint} {agent.modules}")  # raises: unknown name

    def start(self, modules: str) -> RunningEnvironment:
        from dimos.agents.mcp.mcp_adapter import McpAdapter
        from dimos.memory.store.sqlite import SqliteStore

        deadline = time.monotonic() + self.launch_timeout_s
        if not self.attach:
            from dimos.e2e_tests.dimos_cli_call import DimosCliCall

            proc = DimosCliCall()
            proc.simulator = self.simulator
            proc.global_args = ["--dimsim-scene", self.scene, "--record"]
            proc.demo_args = ["run", *self.blueprint.split(), *modules.split()]
            proc.start()
            self._proc = proc
        mcp_url = default_mcp_url()
        if not McpAdapter(mcp_url).wait_for_ready(timeout=self.launch_timeout_s, interval=2.0):
            raise RuntimeError(f"MCP at {mcp_url} not ready — is dimos up?")
        if self.setup is not _no_setup:
            from dimos.e2e_tests.dim_sim_client import DimSimClient

            sim = DimSimClient()
            sim.start()
            self._sim = sim
            self.setup(sim)
        path = self._wait_recording(deadline)
        self._recording = SqliteStore(path=str(path), must_exist=True)
        return RunningEnvironment(mcp_url=mcp_url, streams=(), artifacts={"recording": path})

    def _wait_recording(self, deadline: float) -> Path:
        """``recordings/<run-id>/memory.db`` of the dimos this environment drives."""
        from dimos.constants import RECORDINGS_DIR
        from dimos.core.run_registry import list_runs

        pid = self._proc.process.pid if self._proc is not None and self._proc.process else None
        while True:
            runs = [r for r in list_runs(alive_only=True) if pid is None or r.pid == pid]
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
        """Wait until the robot is at rest — a navigation skill returns once
        the goal is set, while the robot keeps driving."""
        if self._recording is None:
            return
        anchor = None
        anchor_t = 0.0
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            try:
                p = self._recording.streams.odom.last().data.position
            except (AttributeError, LookupError):
                return
            if anchor is None or math.hypot(p.x - anchor.x, p.y - anchor.y) > self.at_rest_m:
                anchor, anchor_t = p, time.monotonic()
            elif time.monotonic() - anchor_t >= self.at_rest_s:
                return
            time.sleep(self.settle_poll_s)

    def stop(self) -> None:
        if self._recording is not None:
            self._recording.stop()
            self._recording = None
        if self._sim is not None:
            self._sim.stop()
            self._sim = None
        if self._proc is not None:
            self._proc.stop()
            self._proc = None
