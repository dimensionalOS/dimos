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

"""Functional test: boot the bench-load blueprint via the real coordinator and assert the
custom /bench/* topics actually flow on the bus. Complements the import-only validation in
test_all_blueprints.py. Marked self_hosted because it spawns the coordinator + worker pool.

Uses LCM (deterministic, no router) regardless of the platform default transport.
"""

import os
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.self_hosted
def test_bench_load_topics_flow() -> None:
    env = {**os.environ, "DIMOS_TRANSPORT": "lcm"}
    # Invoke the CLI in a child interpreter so we don't depend on `dimos` being on PATH.
    code = (
        "import sys; sys.argv=['dimos','run','bench-load','-o','benchload.autostart=true'];"
        "from dimos.robot.cli.dimos import cli_main; cli_main()"
    )
    proc = subprocess.Popen(
        [sys.executable, "-c", code],
        env=env,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    counts = {"/bench/p0": 0, "/bench/grid": 0}
    subs: list = []
    try:
        time.sleep(14)  # coordinator boot + worker spawn + autostart
        assert proc.poll() is None, "bench-load process exited before publishing"

        from dimos.core.transport import LCMTransport
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

        for name, typ in (("/bench/p0", PoseStamped), ("/bench/grid", OccupancyGrid)):
            t = LCMTransport(name, typ)
            t.start()
            t.subscribe(lambda _m, n=name: counts.__setitem__(n, counts[n] + 1))
            subs.append(t)

        time.sleep(4)  # sample window
    finally:
        for t in subs:
            t.stop()  # join the _lcm_loop threads so the test leaks nothing
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        proc.wait(timeout=15)

    # 100Hz pose / 20Hz grid over a 4s window, minus startup slack.
    assert counts["/bench/p0"] > 50, f"too few PoseStamped on /bench/p0: {counts}"
    assert counts["/bench/grid"] > 2, f"too few OccupancyGrid on /bench/grid: {counts}"
