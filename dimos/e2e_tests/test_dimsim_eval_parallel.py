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

"""Parallel eval tests for DimSim simulation.

3 dimos instances + 3 headless browser pages, 1 eval workflow each.
Runs all workflows concurrently, cutting wall-clock time to ~1 min.

    pytest dimos/e2e_tests/test_dimsim_eval_parallel.py -v -s -m slow
"""

from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import pytest
import websocket

from dimos.e2e_tests.dimos_cli_call import DimosCliCall

PORT = 8090
EVALS_DIR = Path.home() / ".dimsim" / "evals"
NUM_CHANNELS = 3
LCM_BASE_PORT = 7667


def _force_kill_port(port: int) -> None:
    """Kill any process listening on the given port."""
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        pids = result.stdout.strip().split()
        for pid in pids:
            if pid:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


def _wait_for_port(port: int, timeout: float = 120) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=2):
                return True
        except OSError:
            time.sleep(1)
    return False


def _wait_for_all_pages(log_path: Path, num_pages: int, timeout: float = 90) -> None:
    """Wait until dimsim's log shows all headless pages are sending sensor data.

    Checks for sensor messages from the last channel (page-N-1) which proves
    all pages have loaded their Three.js scenes and are rendering.
    """
    last_channel = f"page-{num_pages - 1}"
    marker = f"bridge:{last_channel}] sensor"
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            text = log_path.read_text()
            if marker in text:
                print(f"  All {num_pages} headless pages confirmed ready (sensor data flowing)")
                return
        except FileNotFoundError:
            pass
        time.sleep(2)
    print(
        f"  WARNING: page-{num_pages - 1} sensor data not seen after {timeout}s, proceeding anyway"
    )


def _wait_for_port_free(port: int, timeout: float = 30) -> bool:
    """Wait until nothing is listening on *port*."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=1):
                time.sleep(1)  # still occupied
        except OSError:
            return True  # connection refused → port is free
    return False


def _load_workflow(env: str, name: str) -> dict:
    path = EVALS_DIR / env / f"{name}.json"
    return json.loads(path.read_text())


# ── Eval clients ─────────────────────────────────────────────────────────────


class EvalClient:
    """Talks to the browser eval harness via the bridge WebSocket."""

    def __init__(self, port: int = PORT):
        self.ws = websocket.WebSocket()
        self.ws.connect(f"ws://localhost:{port}")

    def _send(self, msg: dict) -> None:
        self.ws.send(json.dumps(msg))

    def _wait_for(self, msg_type: str, timeout: float = 120) -> dict:
        self.ws.settimeout(timeout)
        while True:
            raw = self.ws.recv()
            if isinstance(raw, bytes):
                continue
            msg = json.loads(raw)
            if msg.get("type") == msg_type:
                return msg

    def wait_for_harness(self, timeout: float = 60) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                self._send({"type": "ping"})
                self.ws.settimeout(3)
                raw = self.ws.recv()
                if isinstance(raw, str):
                    msg = json.loads(raw)
                    if msg.get("type") == "pong":
                        return True
            except (websocket.WebSocketTimeoutException, Exception):
                time.sleep(1)
        return False

    def run_workflow(self, workflow: dict) -> dict:
        """Send loadEnv + startWorkflow, wait for workflowComplete."""
        timeout = workflow.get("timeoutSec", 120) + 30
        self._send({"type": "loadEnv", "scene": workflow.get("environment", "apt")})
        self._wait_for("envReady", timeout=30)
        self._send({"type": "startWorkflow", "workflow": workflow})
        return self._wait_for("workflowComplete", timeout=timeout)

    def close(self):
        self.ws.close()


class ChannelEvalClient(EvalClient):
    """EvalClient that connects to a specific channel's control WS.

    Inherits run_workflow (loadEnv + startWorkflow) from EvalClient.
    Only overrides _send/_wait_for for channel routing.
    """

    def __init__(self, port: int, channel: str):
        self._port = port
        self.channel = channel
        self.ws = websocket.WebSocket()
        self.ws.connect(f"ws://localhost:{port}?channel={channel}")

    def wait_for_harness(self, timeout: float = 60) -> bool:
        """Override: drain pose/sensor messages to find the pong.

        The bridge sends pose updates at 50Hz to all control clients.
        The base class reads only one message per loop and misses the pong.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                self._send({"type": "ping"})
                self._wait_for("pong", timeout=5)
                return True
            except Exception:
                time.sleep(1)
        return False

    def _send(self, msg: dict) -> None:
        msg["channel"] = self.channel
        self.ws.send(json.dumps(msg))

    def _wait_for(self, msg_type: str, timeout: float = 120) -> dict:
        self.ws.settimeout(timeout)
        while True:
            raw = self.ws.recv()
            if isinstance(raw, bytes):
                continue
            if not raw:
                continue
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            msg_ch = msg.get("channel", "")
            if msg.get("type") == msg_type and (not msg_ch or msg_ch == self.channel):
                return msg


# ── Fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture(scope="class")
def parallel_env():
    """Start dimos-0 (launches dimsim with 3 channels), then dimos-1/2 (connect-only)."""
    _force_kill_port(PORT)
    assert _wait_for_port_free(PORT, timeout=10), f"Port {PORT} still in use after force-kill"

    calls: list[DimosCliCall] = []
    log_files: list = []
    venv_bin = str(Path(sys.prefix) / "bin")
    base_env = {
        **os.environ,
        "DIMSIM_HEADLESS": "1",
        "DIMSIM_RENDER": "gpu",
        "PATH": venv_bin + os.pathsep + os.environ.get("PATH", ""),
    }
    log_dir_env = os.environ.get("DIMSIM_EVAL_LOG_DIR", "")
    log_dir = Path(log_dir_env) if log_dir_env else None
    if log_dir:
        log_dir.mkdir(parents=True, exist_ok=True)

    # dimos-0: launches dimsim normally with --channels 3
    env0 = {
        **base_env,
        "LCM_DEFAULT_URL": f"udpm://239.255.76.67:{LCM_BASE_PORT}?ttl=0",
        "DIMSIM_CHANNELS": str(NUM_CHANNELS),
        "EVAL_INSTANCE_ID": "0",
    }
    log0 = open(log_dir / "dimos-0.log", "w") if log_dir else None
    log_files.append(log0)
    call0 = DimosCliCall()
    call0.demo_args = ["sim-parallel-eval"]
    call0.process = subprocess.Popen(
        ["dimos", "--simulation", "run", "sim-parallel-eval"],
        env=env0,
        stdout=log0 or subprocess.DEVNULL,
        stderr=log0 or subprocess.DEVNULL,
        start_new_session=True,
    )
    calls.append(call0)

    try:
        # Wait for dimsim bridge + all headless pages before starting dimos-1/2
        assert _wait_for_port(PORT, timeout=120), f"Bridge not ready on port {PORT}"
        if log_dir:
            _wait_for_all_pages(log_dir / "dimos-0.log", NUM_CHANNELS, timeout=90)
        else:
            time.sleep(30)  # No log file to check — wait a fixed period for pages to load

        # dimos-1 and dimos-2: connect-only (skip dimsim launch)
        for i in range(1, NUM_CHANNELS):
            env_i = {
                **base_env,
                "DIMSIM_CONNECT_ONLY": "1",
                "LCM_DEFAULT_URL": f"udpm://239.255.76.67:{LCM_BASE_PORT + i}?ttl=0",
                "EVAL_INSTANCE_ID": str(i),
            }
            log_i = open(log_dir / f"dimos-{i}.log", "w") if log_dir else None
            log_files.append(log_i)
            call_i = DimosCliCall()
            call_i.demo_args = ["sim-parallel-eval"]
            call_i.process = subprocess.Popen(
                ["dimos", "--simulation", "run", "sim-parallel-eval"],
                env=env_i,
                stdout=log_i or subprocess.DEVNULL,
                stderr=log_i or subprocess.DEVNULL,
                start_new_session=True,
            )
            calls.append(call_i)

        # Give connect-only instances time to start and establish LCM connections
        time.sleep(10)

        if log_dir:
            print(f"\n  dimos logs → {log_dir}/dimos-{{0,1,2}}.log")
        yield calls
    finally:
        # Teardown: stop in reverse (connect-only first, then the one that owns dimsim)
        for call in reversed(calls):
            call.stop()
        for f in log_files:
            if f:
                f.close()
        _force_kill_port(PORT)


def _connect_channel(port: int, channel: str, timeout: float = 120) -> ChannelEvalClient:
    """Connect to a channel with retries (bridge may be under load)."""
    deadline = time.time() + timeout
    last_err = None
    while time.time() < deadline:
        try:
            client = ChannelEvalClient(port, channel)
            if client.wait_for_harness(timeout=min(30, deadline - time.time())):
                return client
            client.close()
        except Exception as e:
            last_err = e
            time.sleep(2)
    raise ConnectionError(f"Could not connect to {channel} after {timeout}s: {last_err}")


@pytest.fixture(scope="class")
def parallel_eval_clients(parallel_env):
    """Create one ChannelEvalClient per channel, wait for each harness."""
    clients: list[ChannelEvalClient] = []
    for i in range(NUM_CHANNELS):
        channel = f"page-{i}"
        client = _connect_channel(PORT, channel, timeout=120)
        print(f"  {channel}: harness ready")
        clients.append(client)
    yield clients
    for client in clients:
        client.close()


# ── Test ─────────────────────────────────────────────────────────────────────


@pytest.mark.skipif_in_ci
@pytest.mark.slow
class TestSimEvalParallel:
    """Run DimSim evals in parallel — 3 dimos instances, 3 browser pages."""

    WORKFLOWS = [
        ("apt", "television"),
        ("apt", "go-to-couch"),
        ("apt", "go-to-kitchen"),
    ]

    def test_parallel_evals(self, parallel_eval_clients: list[ChannelEvalClient]) -> None:
        """Run all 3 eval workflows concurrently, one per channel."""
        results: dict[str, dict] = {}
        errors: dict[str, str] = {}

        def run_one(idx: int) -> tuple[str, dict]:
            # Stagger starts so loadEnv doesn't hammer the bridge simultaneously
            time.sleep(idx * 3)
            env, name = self.WORKFLOWS[idx]
            workflow = _load_workflow(env, name)
            result = parallel_eval_clients[idx].run_workflow(workflow)
            return name, result

        with ThreadPoolExecutor(max_workers=NUM_CHANNELS) as pool:
            futures = {pool.submit(run_one, i): i for i in range(NUM_CHANNELS)}
            for future in as_completed(futures):
                try:
                    name, result = future.result()
                    results[name] = result
                except Exception as exc:
                    idx = futures[future]
                    errors[self.WORKFLOWS[idx][1]] = str(exc)

        # Report and assert all results
        failures: list[str] = []

        for name, result in results.items():
            scores = result.get("rubricScores", {})
            od = scores.get("objectDistance", {})
            passed = od.get("pass", False)
            details = od.get("details", result.get("reason", "unknown"))
            print(f"  {name}: {'PASS' if passed else 'FAIL'} — {details}")
            if not passed:
                failures.append(f"{name}: {details}")

        for name, err in errors.items():
            print(f"  {name}: ERROR — {err}")
            failures.append(f"{name}: ERROR — {err}")

        assert not failures, f"{len(failures)}/{NUM_CHANNELS} evals failed:\n" + "\n".join(
            f"  - {f}" for f in failures
        )
