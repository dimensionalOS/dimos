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

import os
import signal
import subprocess
import time

import requests


def wait_for_http(call: "DimosCliCall", url: str, timeout_s: float) -> None:
    """Poll `url` until it answers 2xx, failing fast when the process died."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            requests.get(url, timeout=2).raise_for_status()
            return
        except requests.RequestException:
            assert call.process is not None and call.process.poll() is None, (
                f"dimos exited with {call.process and call.process.returncode} before {url} came up"
            )
            if time.monotonic() > deadline:
                raise TimeoutError(f"{url} not reachable after {timeout_s} s") from None
            time.sleep(0.5)


class DimosCliCall:
    process: subprocess.Popen[bytes] | None
    demo_args: list[str] | None = None
    mcp_port: int | None = None
    # None: no --simulation flag (e.g. replay runs via --robot-ip fake).
    simulator: str | None = "mujoco"

    def __init__(self) -> None:
        self.process = None
        # Root-level CLI flags (before the subcommand), e.g. ["--robot-ip", "fake"].
        self.global_args: list[str] = []
        self.extra_env: dict[str, str] = {}

    def start(self) -> None:
        if self.demo_args is None:
            raise ValueError("Demo args must be set before starting the process.")

        args = list(self.demo_args)
        if len(args) == 1:
            args = ["run", *args]

        # If a port was supplied, override `global_config.mcp_port` (used by
        # `McpServer.start` to bind) and `McpClient.mcp_server_url` (which
        # defaults to a hard-coded `http://localhost:9990/mcp`) so server
        # and client agree on the same port.
        #
        # The McpClient URL goes through an env var rather than a dynamic
        # blueprint flag: BlueprintConfigParser skips environment overrides
        # whose module is absent from the blueprint, but rejects unknown CLI
        # configuration flags. Blueprints without an mcpclient (e.g.
        # `coordinator-mock`) would otherwise fail configuration validation.
        global_overrides: list[str] = list(self.global_args)
        env = os.environ.copy()
        env.update(self.extra_env)
        if self.mcp_port is not None:
            global_overrides += ["--mcp-port", str(self.mcp_port)]
            env["MCPCLIENT__MCP_SERVER_URL"] = f"http://localhost:{self.mcp_port}/mcp"
        if self.simulator is not None:
            global_overrides += ["--simulation", self.simulator]

        self.process = subprocess.Popen(
            [
                "dimos",
                *global_overrides,
                *args,
            ],
            start_new_session=True,
            env=env,
        )

    def stop(self) -> None:
        # Detach before signalling so stop() is idempotent: once the first
        # call reaps the process its pgid can be recycled, and a second
        # killpg could hit an unrelated process group.
        process, self.process = self.process, None
        if process is None:
            return

        try:
            # Send SIGTERM to the entire process group so child processes
            # (e.g. the mujoco viewer subprocess) are also terminated.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                # The group is already gone: the process exited on its own
                # and an earlier poll()/wait() reaped it.
                return

            # Record the time when we sent the kill signal
            shutdown_start = time.time()

            # Wait for the process to terminate with a 30-second timeout
            try:
                process.wait(timeout=30)
                shutdown_duration = time.time() - shutdown_start

                # Verify it shut down in time
                assert shutdown_duration <= 30, (
                    f"Process took {shutdown_duration:.2f} seconds to shut down, "
                    f"which exceeds the 30-second limit"
                )
            except subprocess.TimeoutExpired:
                # If we reach here, the process didn't terminate in 30 seconds
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()  # Clean up
                raise AssertionError(
                    "Process did not shut down within 30 seconds after receiving SIGTERM"
                )

        except Exception:
            # Clean up if something goes wrong
            if process.poll() is None:  # Process still running
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
            raise
