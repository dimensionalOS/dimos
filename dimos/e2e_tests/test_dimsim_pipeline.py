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

"""Pipeline test for dimos sim-basic — the dimos Module/Blueprint stack on
top of DimSim, without the nav/agent layers (TestSimNav covers that).

The fixture captures the subprocess's stdout/stderr to a tempfile and dumps
it on teardown so failures in CI carry diagnostic context.
"""

import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest

from dimos.e2e_tests._subprocess_log import (
    dump_log,
    launch_with_streaming_log,
    wait_for_log_pattern,
)
from dimos.e2e_tests.lcm_spy import LcmSpy

BRIDGE_PORT = 8090


# ── Helpers ──────────────────────────────────────────────────────────────────


def _force_kill_port(port: int) -> None:
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"], capture_output=True, text=True, timeout=5
        )
        for pid in result.stdout.strip().split():
            if pid:
                try:
                    os.kill(int(pid), 9)
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


def _wait_for_port_free(port: int, timeout: float = 10) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=1):
                time.sleep(1)
        except OSError:
            return True
    return False


# ── dimos sim-basic ──────────────────────────────────────────────────────────


@pytest.fixture(scope="class")
def dimos_sim_basic():
    """Launch dimos sim-basic — adds the dimos blueprint stack on top of dimsim."""
    _force_kill_port(BRIDGE_PORT)
    assert _wait_for_port_free(BRIDGE_PORT, timeout=10), (
        f"Port {BRIDGE_PORT} still busy after force-kill"
    )

    venv_bin = str(Path(sys.prefix) / "bin")
    env = {
        **os.environ,
        "DIMSIM_HEADLESS": "1",
        "DIMSIM_RENDER": os.environ.get("DIMSIM_RENDER", "cpu"),
        "DIMSIM_VERBOSE": "1",
        "PYTHONUNBUFFERED": "1",
        "PATH": venv_bin + os.pathsep + os.environ.get("PATH", ""),
    }

    proc, log_path = launch_with_streaming_log(
        "dimos_sim_basic",
        ["dimos", "--simulation", "run", "sim-basic"],
        env=env,
    )

    try:
        if not _wait_for_port(BRIDGE_PORT, timeout=120):
            dump_log("dimos sim-basic", log_path)
            pytest.fail(f"dimos sim-basic never opened port {BRIDGE_PORT}")
        # Gate on actual server-side physics activation. The "Sensor publishing
        # active" log line lies — it only confirms the WS is connected. The
        # 26MB Rapier snapshot still has to ship and be restored before
        # /odom and /lidar publish. Under GPU rendering this lands in ~10s.
        if not wait_for_log_pattern(
            log_path,
            r"(Rapier snapshot restored|\[bridge:[^\]]*\] ready)",
            timeout=60.0,
        ):
            dump_log("dimos sim-basic", log_path)
            pytest.fail("Rapier snapshot never restored — server physics dead")
        yield proc
    finally:
        try:
            os.killpg(proc.pid, 15)
        except (ProcessLookupError, PermissionError):
            try:
                proc.terminate()
            except ProcessLookupError:
                pass
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, 9)
            except (ProcessLookupError, PermissionError):
                proc.kill()
            proc.wait()
        dump_log("dimos sim-basic", log_path)
        log_path.unlink(missing_ok=True)
        _force_kill_port(BRIDGE_PORT)


@pytest.fixture(scope="class")
def dimos_sim_basic_spy(dimos_sim_basic):
    spy = LcmSpy()
    spy.save_topic("/color_image#sensor_msgs.Image")
    spy.save_topic("/odom#geometry_msgs.PoseStamped")
    spy.start()
    try:
        yield spy
    finally:
        spy.stop()


class TestDimosSimBasic:
    """dimos sim-basic — verifies the blueprint stack (transports, TF, headless
    gating) on top of DimSim. TestSimNav (in test_dimsim_nav.py) layers nav on
    top of this.
    """

    def test_color_image_publishes(self, dimos_sim_basic_spy) -> None:
        dimos_sim_basic_spy.wait_for_saved_topic(
            "/color_image#sensor_msgs.Image", timeout=30.0
        )

    def test_odom_publishes(self, dimos_sim_basic_spy) -> None:
        dimos_sim_basic_spy.wait_for_saved_topic(
            "/odom#geometry_msgs.PoseStamped", timeout=30.0
        )
