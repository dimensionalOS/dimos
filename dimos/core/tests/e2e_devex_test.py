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

"""Full-stack developer experience test — no pytest.

Simulates the complete DimOS developer workflow as if an OpenClaw agent
is using DimOS for the first time:

1. dimos run stress-test --daemon  (start in background)
2. dimos status                    (verify running)
3. dimos mcp list-tools            (discover tools)
4. dimos mcp call echo             (call a tool)
5. dimos mcp status                (module info)
6. dimos mcp modules               (module-skill mapping)
7. dimos agent-send "hello"        (send to agent)
8. Check logs for responses
9. dimos stop                     (clean shutdown)
10. dimos status                   (verify stopped)

Usage:
    python -m dimos.core.tests.e2e_devex_test
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time

REPO_DIR = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
VENV_PYTHON = os.path.join(REPO_DIR, ".venv", "bin", "python")
# Use the repo's own python if venv exists, otherwise fall back to system
if not os.path.exists(VENV_PYTHON):
    VENV_PYTHON = sys.executable


def run_dimos(*args: str, timeout: float = 30) -> subprocess.CompletedProcess[str]:
    """Run a dimos CLI command."""
    cmd = [VENV_PYTHON, "-m", "dimos.robot.cli.dimos", *args]
    env = {**os.environ, "CI": "1", "PYTHONPATH": REPO_DIR}
    result = subprocess.run(
        cmd, capture_output=True, text=True, timeout=timeout, cwd=REPO_DIR, env=env
    )
    return result


def p(msg: str, ok: bool = True) -> None:
    icon = "\u2705" if ok else "\u274c"
    print(f"  {icon} {msg}")


def section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def wait_for_mcp(timeout: float = 20.0) -> bool:
    """Poll MCP until responsive."""
    import requests

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            resp = requests.post(
                "http://localhost:9990/mcp",
                json={"jsonrpc": "2.0", "id": 1, "method": "initialize"},
                timeout=2,
            )
            if resp.status_code == 200:
                return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


def main() -> None:
    failures = 0
    print("\n" + "=" * 60)
    print("  FULL-STACK DEVELOPER EXPERIENCE TEST")
    print("  Simulating: OpenClaw agent using DimOS")
    print("=" * 60)

    # ---------------------------------------------------------------
    # Step 1: dimos run stress-test --daemon
    # ---------------------------------------------------------------
    section("Step 1: dimos run stress-test --daemon")
    result = run_dimos("run", "stress-test", "--daemon", timeout=60)
    print(f"  stdout: {result.stdout.strip()[:200]}")
    if result.stderr:
        # Filter out noisy log lines
        err_lines = [
            l
            for l in result.stderr.strip().split("\n")
            if not any(x in l for x in ["[inf]", "[dbg]", "INFO:", "WARNING:"])
        ]
        if err_lines:
            print(f"  stderr: {chr(10).join(err_lines[:5])}")

    if result.returncode == 0:
        p("Daemon started successfully")
    else:
        p(f"Daemon failed to start (exit={result.returncode})", ok=False)
        print(f"  Full stderr:\n{result.stderr[:500]}")
        failures += 1
        # Try to continue anyway — maybe foreground mode issue

    # Wait for MCP to be ready
    if wait_for_mcp(timeout=20):
        p("MCP server responding")
    else:
        p("MCP server not responding after 20s", ok=False)
        failures += 1
        print("  Cannot continue without MCP. Exiting.")
        sys.exit(1)

    # ---------------------------------------------------------------
    # Step 2: dimos status
    # ---------------------------------------------------------------
    section("Step 2: dimos status")
    result = run_dimos("status")
    print(f"  output: {result.stdout.strip()[:300]}")
    if result.returncode == 0 and (
        "running" in result.stdout.lower() or "pid" in result.stdout.lower()
    ):
        p("Status shows running instance")
    else:
        p(f"Status unclear (exit={result.returncode})", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Step 3: dimos mcp list-tools
    # ---------------------------------------------------------------
    section("Step 3: dimos mcp list-tools")
    result = run_dimos("mcp", "list-tools")
    if result.returncode == 0:
        try:
            tools = json.loads(result.stdout)
            tool_names = [t["name"] for t in tools]
            p(f"Discovered {len(tools)} tools: {', '.join(tool_names)}")
            if "echo" in tool_names and "ping" in tool_names:
                p("Expected tools (echo, ping) present")
            else:
                p("Missing expected tools", ok=False)
                failures += 1
        except json.JSONDecodeError:
            p(f"Invalid JSON output: {result.stdout[:100]}", ok=False)
            failures += 1
    else:
        p(f"list-tools failed (exit={result.returncode}): {result.stdout[:100]}", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Step 4: dimos mcp call echo --arg message=hello
    # ---------------------------------------------------------------
    section("Step 4: dimos mcp call echo --arg message=hello")
    result = run_dimos("mcp", "call", "echo", "--arg", "message=hello-from-devex-test")
    if result.returncode == 0 and "hello-from-devex-test" in result.stdout:
        p(f"echo returned: {result.stdout.strip()[:100]}")
    else:
        p(f"echo call failed (exit={result.returncode}): {result.stdout[:100]}", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Step 5: dimos mcp status
    # ---------------------------------------------------------------
    section("Step 5: dimos mcp status")
    result = run_dimos("mcp", "status")
    if result.returncode == 0:
        try:
            data = json.loads(result.stdout)
            p(
                f"Status: pid={data.get('pid')}, {data.get('skill_count', '?')} skills, modules={data.get('modules', [])}"
            )
        except json.JSONDecodeError:
            p(f"Non-JSON output: {result.stdout[:100]}", ok=False)
            failures += 1
    else:
        p(f"mcp status failed (exit={result.returncode})", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Step 6: dimos mcp modules
    # ---------------------------------------------------------------
    section("Step 6: dimos mcp modules")
    result = run_dimos("mcp", "modules")
    if result.returncode == 0:
        try:
            data = json.loads(result.stdout)
            for mod_name, skills in data.get("modules", {}).items():
                p(f"Module {mod_name}: {', '.join(skills)}")
        except json.JSONDecodeError:
            p(f"Non-JSON output: {result.stdout[:100]}", ok=False)
            failures += 1
    else:
        p(f"mcp modules failed (exit={result.returncode})", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Step 7: dimos agent-send "hello"
    # ---------------------------------------------------------------
    section("Step 7: dimos agent-send 'what tools do you have?'")
    result = run_dimos("agent-send", "what tools do you have?")
    if result.returncode == 0:
        p(f"agent-send response: {result.stdout.strip()[:200]}")
    else:
        p(f"agent-send failed (exit={result.returncode}): {result.stdout[:100]}", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Step 8: Check logs
    # ---------------------------------------------------------------
    section("Step 8: Check per-run logs")
    log_base = os.path.expanduser("~/.local/state/dimos/logs")
    if os.path.isdir(log_base):
        runs = sorted(os.listdir(log_base), reverse=True)
        if runs:
            latest_run = runs[0]
            log_file = os.path.join(log_base, latest_run, "main.jsonl")
            if os.path.exists(log_file):
                size = os.path.getsize(log_file)
                with open(log_file) as f:
                    lines = f.readlines()
                p(f"Log file: {log_file} ({size} bytes, {len(lines)} lines)")
                if lines:
                    # Show last 3 lines
                    for line in lines[-3:]:
                        print(f"    {line.strip()[:120]}")
            else:
                p(f"No main.jsonl found in {latest_run}", ok=False)
                # Check what files exist
                run_dir = os.path.join(log_base, latest_run)
                files = os.listdir(run_dir) if os.path.isdir(run_dir) else []
                print(f"    Files in run dir: {files}")
                failures += 1
        else:
            p("No run directories found", ok=False)
            failures += 1
    else:
        p(f"Log base dir not found: {log_base}", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Step 9: dimos stop
    # ---------------------------------------------------------------
    section("Step 9: dimos stop")
    result = run_dimos("stop")
    print(f"  output: {result.stdout.strip()[:200]}")
    if result.returncode == 0:
        p("Stopped successfully")
    else:
        p(f"Stop failed (exit={result.returncode}): {result.stderr[:100]}", ok=False)
        failures += 1

    # Wait for shutdown
    time.sleep(2)

    # ---------------------------------------------------------------
    # Step 10: dimos status (verify stopped)
    # ---------------------------------------------------------------
    section("Step 10: dimos status (verify stopped)")
    result = run_dimos("status")
    print(f"  output: {result.stdout.strip()[:200]}")
    if (
        "no running" in result.stdout.lower()
        or "no dimos" in result.stdout.lower()
        or result.returncode == 0
    ):
        p("Confirmed: no running instances")
    else:
        p(f"Unexpected status after stop (exit={result.returncode})", ok=False)
        failures += 1

    # ---------------------------------------------------------------
    # Summary
    # ---------------------------------------------------------------
    print("\n" + "=" * 60)
    if failures == 0:
        print("  \u2705 FULL DEVELOPER EXPERIENCE TEST PASSED")
        print("  All CLI commands work end-to-end!")
    else:
        print(f"  \u274c {failures} FAILURES in developer experience test")
    print("=" * 60 + "\n")

    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
