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

"""Sequential eval tests — 1 dimos instance, run workflows one at a time.

Uses sim-eval blueprint (includes rerun/browser for manual observation).
Run individual evals or all of them:

    pytest dimos/e2e_tests/test_dimsim_eval.py -v -s -m slow
    pytest dimos/e2e_tests/test_dimsim_eval.py::TestSimEvalSequential::test_go_to_tv -v -s -m slow
"""

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

from dimos.core.transport import pLCMTransport
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy

PORT = 8090
EVALS_DIR = Path.home() / ".dimsim" / "evals"


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


def _wait_for_port_free(port: int, timeout: float = 30) -> bool:
    """Wait until nothing is listening on *port*."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=1):
                time.sleep(1)
        except OSError:
            return True
    return False


def _send_task_to_agent(message: str) -> None:
    """Publish the task directly to /human_input LCM so McpClient picks it up.

    Mirrors what McpServer.agent_send does internally — bypasses MCP/HTTP so
    we don't depend on the FastAPI skill registry being populated.
    """
    transport: pLCMTransport[str] = pLCMTransport("/human_input")
    try:
        transport.start()
        transport.publish(message)
    finally:
        transport.stop()


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
        """Send loadEnv + startWorkflow, prompt the agent over LCM, wait for workflowComplete."""
        timeout = workflow.get("timeoutSec", 120) + 30
        self._send({"type": "loadEnv", "scene": workflow.get("environment", "apt")})
        self._wait_for("envReady", timeout=30)
        self._send({"type": "startWorkflow", "workflow": workflow})
        # Prompt the dimos agent with the workflow task. The harness teleports
        # the agent on startWorkflow; we then publish the task to /human_input
        # over LCM, which McpClient (the agent brain in sim_agentic) consumes
        # to drive the agent stack.
        task = workflow.get("task", "").strip()
        if task:
            _send_task_to_agent(task)
        return self._wait_for("workflowComplete", timeout=timeout)

    def close(self):
        self.ws.close()


def _load_workflow(env: str, name: str) -> dict:
    path = EVALS_DIR / env / f"{name}.json"
    return json.loads(path.read_text())


@pytest.fixture(scope="class")
def sim_eval():
    """Start dimos sim-eval headless, tear down after."""
    _force_kill_port(PORT)
    assert _wait_for_port_free(PORT, timeout=10), f"Port {PORT} still in use after force-kill"
    log_dir = os.environ.get("DIMSIM_EVAL_LOG_DIR", "")
    if log_dir:
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path / "dimos-sequential.log", "w")
        print(f"\n  dimos logs → {log_path}/dimos-sequential.log")
    else:
        log_file = None

    spy = LcmSpy()
    spy.save_topic("/color_image#sensor_msgs.Image")
    spy.save_topic("/odom#geometry_msgs.PoseStamped")
    spy.start()

    venv_bin = str(Path(sys.prefix) / "bin")
    env = {
        **os.environ,
        "DIMSIM_HEADLESS": "1",
        "DIMSIM_RENDER": "gpu",
        "PATH": venv_bin + os.pathsep + os.environ.get("PATH", ""),
    }
    call = DimosCliCall()
    call.demo_args = ["sim-eval"]
    call.process = subprocess.Popen(
        ["dimos", "--simulation", "run", "sim-eval"],
        env=env,
        stdout=log_file or subprocess.DEVNULL,
        stderr=log_file or subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        assert _wait_for_port(PORT, timeout=120), f"Bridge not ready on port {PORT}"
        spy.wait_for_saved_topic("/color_image#sensor_msgs.Image", timeout=60.0)
        spy.wait_for_saved_topic("/odom#geometry_msgs.PoseStamped", timeout=60.0)

        yield call
    finally:
        call.stop()
        spy.stop()
        if log_file:
            log_file.close()
        _force_kill_port(PORT)


@pytest.fixture(scope="class")
def eval_client(sim_eval):
    """Connect to bridge WS, wait for eval harness, then give the dimos agent
    thread time to initialize before any test runs (LangChain agent setup +
    MCP tool fetch typically completes within ~15s after module deploy)."""
    client = EvalClient(PORT)
    assert client.wait_for_harness(timeout=60), "Eval harness not responding"
    time.sleep(20)
    yield client
    client.close()


# Runs in CI: spawns dimos sim-eval headless, exercises the eval harness over WS.
# Requires multicast routing, network egress (dimsim binary download), and headless Chrome.
class TestSimEvalSequential:
    """Run DimSim evals sequentially against a live dimos sim-eval instance."""

    def _run_and_assert(self, eval_client: EvalClient, env: str, workflow_name: str) -> None:
        workflow = _load_workflow(env, workflow_name)
        result = eval_client.run_workflow(workflow)
        scores = result.get("rubricScores", {})
        od = scores.get("objectDistance", {})
        passed = od.get("pass", False)
        details = od.get("details", result.get("reason", "unknown"))
        print(f"  {workflow_name}: {'PASS' if passed else 'FAIL'} — {details}")
        assert passed, f"Eval '{workflow_name}' failed: {details}"

    def test_go_to_tv(self, eval_client) -> None:
        self._run_and_assert(eval_client, "apt", "television")

    def test_go_to_couch(self, eval_client) -> None:
        self._run_and_assert(eval_client, "apt", "go-to-couch")

    def test_go_to_kitchen(self, eval_client) -> None:
        self._run_and_assert(eval_client, "apt", "go-to-kitchen")
