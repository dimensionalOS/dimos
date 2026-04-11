#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Standalone MCP kill/restart stress test — no pytest.

Simulates what happens when DimOS crashes and restarts:
1. Start blueprint with MCP server
2. Verify MCP is responsive
3. Send burst of calls
4. Kill the process (SIGKILL)
5. Verify MCP is dead
6. Restart
7. Verify recovery
8. Repeat N cycles

Usage:
    python -m dimos.core.tests.e2e_mcp_killtest
    python -m dimos.core.tests.e2e_mcp_killtest --cycles 5
"""

from __future__ import annotations

import argparse
import multiprocessing
import multiprocessing.synchronize
import os
import signal
import sys
import time
from typing import Any

import requests

MCP_PORT = 9990
MCP_URL = f"http://localhost:{MCP_PORT}/mcp"


def mcp_call(method: str, params: dict[str, object] | None = None) -> Any:
    payload: dict[str, object] = {"jsonrpc": "2.0", "id": 1, "method": method}
    if params:
        payload["params"] = params
    resp = requests.post(MCP_URL, json=payload, timeout=10)
    resp.raise_for_status()
    return resp.json()


def wait_for_mcp(timeout: float = 15.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            resp = requests.post(
                MCP_URL,
                json={"jsonrpc": "2.0", "id": 1, "method": "initialize"},
                timeout=2,
            )
            if resp.status_code == 200:
                return True
        except (requests.ConnectionError, requests.ReadTimeout):
            pass
        time.sleep(0.3)
    return False


def wait_for_mcp_down(timeout: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            requests.post(
                MCP_URL,
                json={"jsonrpc": "2.0", "id": 1, "method": "initialize"},
                timeout=1,
            )
        except (requests.ConnectionError, requests.ReadTimeout):
            return True
        time.sleep(0.3)
    return False


def run_blueprint_in_process(ready_event: multiprocessing.synchronize.Event) -> None:
    os.environ["CI"] = "1"
    from dimos.agents.mcp.mcp_server import McpServer
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.global_config import global_config
    from dimos.core.tests.stress_test_module import StressTestModule

    global_config.update(viewer="none", n_workers=1)
    bp = autoconnect(StressTestModule.blueprint(), McpServer.blueprint())
    coord = ModuleCoordinator.build(bp)
    ready_event.set()
    try:
        while True:
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        coord.stop()


def p(msg: str, ok: bool = True) -> None:
    icon = "\u2705" if ok else "\u274c"
    print(f"  {icon} {msg}")


def section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def test_mcp_basic_ops() -> int:
    failures = 0
    [
        ("initialize", lambda: (mcp_call("initialize"), "initialize \u2192 dimensional")),
    ]

    # initialize
    try:
        result = mcp_call("initialize")
        assert result["result"]["serverInfo"]["name"] == "dimensional"
        p("initialize \u2192 dimensional")
    except Exception as e:
        p(f"initialize failed: {e}", ok=False)
        failures += 1

    # tools/list
    try:
        result = mcp_call("tools/list")
        tools = {t["name"] for t in result["result"]["tools"]}
        assert "echo" in tools and "ping" in tools
        p(f"tools/list \u2192 {len(tools)} tools")
    except Exception as e:
        p(f"tools/list failed: {e}", ok=False)
        failures += 1

    # echo
    try:
        result = mcp_call("tools/call", {"name": "echo", "arguments": {"message": "killtest"}})
        assert result["result"]["content"][0]["text"] == "killtest"
        p("echo \u2192 killtest")
    except Exception as e:
        p(f"echo failed: {e}", ok=False)
        failures += 1

    # ping
    try:
        result = mcp_call("tools/call", {"name": "ping", "arguments": {}})
        assert result["result"]["content"][0]["text"] == "pong"
        p("ping \u2192 pong")
    except Exception as e:
        p(f"ping failed: {e}", ok=False)
        failures += 1

    # dimos/status
    try:
        result = mcp_call("dimos/status")
        assert "StressTestModule" in result["result"]["modules"]
        p(f"dimos/status \u2192 pid={result['result']['pid']}")
    except Exception as e:
        p(f"dimos/status failed: {e}", ok=False)
        failures += 1

    # dimos/agent_send
    try:
        result = mcp_call("dimos/agent_send", {"message": "hello from killtest"})
        assert "hello from killtest" in result["result"]["content"][0]["text"]
        p("agent_send \u2192 delivered")
    except Exception as e:
        p(f"agent_send failed: {e}", ok=False)
        failures += 1

    # rapid burst
    try:
        for i in range(10):
            r = mcp_call("tools/call", {"name": "echo", "arguments": {"message": f"burst-{i}"}})
            assert r["result"]["content"][0]["text"] == f"burst-{i}"
        p("rapid burst \u2192 10/10 echo calls OK")
    except Exception as e:
        p(f"rapid burst failed: {e}", ok=False)
        failures += 1

    # error handling
    try:
        result = mcp_call("nonexistent/method")
        assert "error" in result
        p("unknown method \u2192 error (correct)")
    except Exception as e:
        p(f"error handling failed: {e}", ok=False)
        failures += 1

    return failures


def run_kill_restart_cycle(cycle: int) -> int:
    failures = 0
    section(f"CYCLE {cycle}: Starting DimOS")

    ctx = multiprocessing.get_context("spawn")
    ready = ctx.Event()
    proc = ctx.Process(target=run_blueprint_in_process, args=(ready,))
    proc.start()

    if not ready.wait(timeout=30):
        p("Blueprint failed to start within 30s", ok=False)
        proc.kill()
        proc.join(5)
        return 1

    if not wait_for_mcp(timeout=15):
        p("MCP server did not come up", ok=False)
        proc.kill()
        proc.join(5)
        return 1
    p("MCP server is up")

    failures += test_mcp_basic_ops()

    # KILL
    section(f"CYCLE {cycle}: SIGKILL \u2192 simulating crash")
    assert proc.pid is not None
    os.kill(proc.pid, signal.SIGKILL)
    proc.join(10)
    p(f"Process killed (pid={proc.pid}, exitcode={proc.exitcode})")

    if wait_for_mcp_down(timeout=10):
        p("MCP confirmed dead after kill")
    else:
        p("MCP still responding after kill!", ok=False)
        failures += 1

    time.sleep(1)

    # RESTART
    section(f"CYCLE {cycle}: Restarting DimOS")
    ready2 = ctx.Event()
    proc2 = ctx.Process(target=run_blueprint_in_process, args=(ready2,))
    proc2.start()

    if not ready2.wait(timeout=30):
        p("Blueprint failed to restart within 30s", ok=False)
        proc2.kill()
        proc2.join(5)
        return failures + 1

    if not wait_for_mcp(timeout=15):
        p("MCP server did not recover after restart", ok=False)
        proc2.kill()
        proc2.join(5)
        return failures + 1
    p("MCP server recovered!")

    section(f"CYCLE {cycle}: Post-recovery verification")
    failures += test_mcp_basic_ops()

    # Clean shutdown
    section(f"CYCLE {cycle}: Clean shutdown")
    assert proc2.pid is not None
    os.kill(proc2.pid, signal.SIGTERM)
    proc2.join(10)
    if proc2.exitcode is not None:
        p(f"Clean shutdown (exitcode={proc2.exitcode})")
    else:
        p("Process did not exit cleanly, forcing kill", ok=False)
        proc2.kill()
        proc2.join(5)
        failures += 1

    if wait_for_mcp_down(timeout=10):
        p("MCP confirmed dead after clean shutdown")
    else:
        p("MCP still responding after shutdown!", ok=False)
        failures += 1

    time.sleep(1)
    return failures


def main() -> None:
    parser = argparse.ArgumentParser(description="MCP kill/restart stress test")
    parser.add_argument("--cycles", type=int, default=3, help="Number of kill/restart cycles")
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  MCP KILL/RESTART STRESS TEST")
    print(f"  Cycles: {args.cycles}")
    print("=" * 60)

    total_failures = 0
    for cycle in range(1, args.cycles + 1):
        failures = run_kill_restart_cycle(cycle)
        total_failures += failures

    print("\n" + "=" * 60)
    if total_failures == 0:
        print("  \u2705 ALL CYCLES PASSED \u2014 MCP is resilient to kill/restart")
    else:
        print(f"  \u274c {total_failures} FAILURES across {args.cycles} cycles")
    print("=" * 60 + "\n")

    sys.exit(1 if total_failures else 0)


if __name__ == "__main__":
    main()
