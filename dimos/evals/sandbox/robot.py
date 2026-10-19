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

"""The robot end of a sandbox: a zenoh router in front of whatever runs the robot.

The agent dials one address and sees the same keys whether the router fronts a
blueprint in the sandbox's robot container or forwards to a real robot that runs
this same entrypoint (``dimos evals robot -- run <blueprint>``).
"""

from __future__ import annotations

from collections.abc import Sequence
import json
import os
import signal
import subprocess
import sys
import threading
import time
from types import FrameType

import zenoh

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Where the router listens unless told otherwise: zenoh's own port, every interface.
ROUTER_LISTEN = "tcp/0.0.0.0:7447"
# Seconds dimos gets to shut down after the router is asked to stop.
DIMOS_SHUTDOWN_S = 20.0


def router_config(listen: Sequence[str], connect: Sequence[str]) -> zenoh.Config:
    """A router reachable on *listen* that dials *connect*; no multicast scouting."""
    config = zenoh.Config()
    config.insert_json5("mode", '"router"')
    config.insert_json5("listen/endpoints", json.dumps(list(listen)))
    config.insert_json5("connect/endpoints", json.dumps(list(connect)))
    config.insert_json5("scouting/multicast/enabled", "false")
    # The admin space would list who is connected, and so tell a sim from a robot.
    config.insert_json5("adminspace/enabled", "false")
    return config


def loopback(endpoint: str) -> str:
    """``tcp/0.0.0.0:7447`` -> ``tcp/127.0.0.1:7447``: how a local process dials a listener."""
    protocol, _, address = endpoint.partition("/")
    return f"{protocol}/127.0.0.1:{address.rpartition(':')[2]}"


def serve(
    listen: str, forward: Sequence[str], dimos_args: Sequence[str], expose_dimos: bool = True
) -> int:
    """Run the router, and ``dimos <dimos_args>`` behind it, until SIGTERM or dimos exits.

    dimos runs on zenoh with every session dialing the router, so each topic the
    blueprint publishes or subscribes reaches whoever dials the router. *forward*
    adds zenoh endpoints the router dials itself: a real robot's router, or a peer
    that only listens (raw-robot-bridge). With expose_dimos off, dimos's own sessions
    stay off the router, so only the forwarded peers are reachable.
    """
    stop = threading.Event()

    def on_signal(signum: int, frame: FrameType | None) -> None:
        stop.set()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    with zenoh.open(router_config([listen], forward)):
        logger.info("sandbox router up", listen=listen, forward=list(forward))
        if not dimos_args:
            stop.wait()
            return 0
        env = {**os.environ, "DIMOS_TRANSPORT": "zenoh"}
        if expose_dimos:
            env["ZENOH_CONNECT"] = loopback(listen)
        # The dimos of this interpreter, whether or not its console script is on PATH.
        command = [sys.executable, "-m", "dimos.cli.dimos", *dimos_args]
        with subprocess.Popen(command, env=env) as proc:
            while proc.poll() is None and not stop.wait(0.5):
                pass
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)  # dimos's own clean shutdown
                try:
                    proc.wait(timeout=DIMOS_SHUTDOWN_S)
                except subprocess.TimeoutExpired:
                    proc.kill()
            return proc.wait()


def await_key(key_expr: str | None, endpoint: str, timeout_s: float) -> bool:
    """Whether *key_expr* publishes on the router at *endpoint* within *timeout_s*; with no key,
    whether the router answers at all."""
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            session = zenoh.open(config)
            break
        except zenoh.ZError:  # the router isn't up yet
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.5)
    with session:
        if key_expr is None:
            return True
        published = threading.Event()
        subscriber = session.declare_subscriber(key_expr, lambda _: published.set())
        try:
            return published.wait(max(0.0, deadline - time.monotonic()))
        finally:
            subscriber.undeclare()
