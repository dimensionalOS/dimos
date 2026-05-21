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
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import IO

# Blueprint stdio is written under this directory; the CI workflow cats it
# on failure since pytest's captured output is lost when --timeout-method=thread
# forces os._exit().
LOG_DIR = Path(os.environ.get("DIMOS_E2E_LOG_DIR", tempfile.gettempdir())) / "dimos-e2e-logs"


class DimosCliCall:
    process: subprocess.Popen[bytes] | None
    demo_args: list[str] | None = None
    mcp_port: int | None = None
    simulator: str = "mujoco"
    log_path: Path | None
    _log_file: IO[bytes] | None

    def __init__(self) -> None:
        self.process = None
        self.log_path = None
        self._log_file = None

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
        # The McpClient URL goes through an env var rather than a `-o`
        # blueprint override: `load_config_args` silently skips env-var
        # overrides whose module is absent from the blueprint, but rejects
        # unknown `-o` keys outright. Blueprints without an mcpclient (e.g.
        # `coordinator-mock`) would otherwise fail config validation.
        global_overrides: list[str] = []
        env = os.environ.copy()
        if self.mcp_port is not None:
            global_overrides += ["--mcp-port", str(self.mcp_port)]
            env["MCPCLIENT__MCP_SERVER_URL"] = f"http://localhost:{self.mcp_port}/mcp"

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        fd, log_path_str = tempfile.mkstemp(prefix="blueprint-", suffix=".log", dir=str(LOG_DIR))
        os.close(fd)
        self.log_path = Path(log_path_str)
        self._log_file = open(self.log_path, "wb")
        # Print before spawn so the path is in the test log even if the test
        # later os._exit()s on pytest-timeout.
        print(f"[dimos blueprint] stdio -> {self.log_path}", file=sys.stderr, flush=True)

        # Force unbuffered Python output in the subprocess so we get lines as
        # they happen rather than at process exit.
        env = {**os.environ, "PYTHONUNBUFFERED": "1"}

        self.process = subprocess.Popen(
            [
                "dimos",
                *global_overrides,
                "--simulation",
                self.simulator,
                *args,
            ],
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
            env=env,
        )

    def check_alive(self) -> None:
        """Raise if the dimos subprocess has exited.

        Tests block waiting for LCM topics from the blueprint; without this
        an early crash (missing API key, module init failure) just looks like
        a 20-minute hang.
        """
        if self.process is None:
            return
        rc = self.process.poll()
        if rc is not None:
            raise RuntimeError(
                f"dimos blueprint subprocess exited early with code {rc}. "
                f"See {self.log_path} for output."
            )

    def stop(self) -> None:
        if self.process is None:
            return

        try:
            # Send SIGTERM to the entire process group so child processes
            # (e.g. the mujoco viewer subprocess) are also terminated.
            os.killpg(self.process.pid, signal.SIGTERM)

            # Record the time when we sent the kill signal
            shutdown_start = time.time()

            # Wait for the process to terminate with a 30-second timeout
            try:
                self.process.wait(timeout=30)
                shutdown_duration = time.time() - shutdown_start

                # Verify it shut down in time
                assert shutdown_duration <= 30, (
                    f"Process took {shutdown_duration:.2f} seconds to shut down, "
                    f"which exceeds the 30-second limit"
                )
            except subprocess.TimeoutExpired:
                # If we reach here, the process didn't terminate in 30 seconds
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()  # Clean up
                raise AssertionError(
                    "Process did not shut down within 30 seconds after receiving SIGTERM"
                )

        except Exception:
            # Clean up if something goes wrong
            if self.process.poll() is None:  # Process still running
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait()
            raise
        finally:
            self._dump_log()

    def _dump_log(self) -> None:
        if self._log_file is not None:
            try:
                self._log_file.close()
            except Exception:
                pass
            self._log_file = None
        if self.log_path is None or not self.log_path.exists():
            return
        try:
            content = self.log_path.read_text(errors="replace")
        except OSError:
            return
        if not content.strip():
            return
        # Dump to stderr so pytest's "Captured stderr" includes it on failure.
        # The on-disk copy at log_path remains for CI to cat after job timeout.
        print(
            f"\n--- dimos blueprint output ({self.log_path}) ---",
            file=sys.stderr,
            flush=True,
        )
        sys.stderr.write(content)
        sys.stderr.write("\n--- end dimos blueprint output ---\n")
        sys.stderr.flush()
