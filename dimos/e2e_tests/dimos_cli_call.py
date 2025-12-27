# Copyright 2025 Dimensional Inc.
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

import signal
import subprocess
import time


class DimosCliCall:
    process: subprocess.Popen[bytes] | None
    demo_name: str | None = None

    def __init__(self) -> None:
        self.process = None

    def start(self) -> None:
        if self.demo_name is None:
            raise ValueError("Demo name must be set before starting the process.")

        self.process = subprocess.Popen(
            ["dimos", "--simulation", "run", self.demo_name],
        )

    def stop(self) -> None:
        if self.process is None:
            return

        try:
            # Send the kill signal (SIGTERM for graceful shutdown)
            self.process.send_signal(signal.SIGTERM)

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
                self.process.kill()  # Force kill
                self.process.wait()  # Clean up
                raise AssertionError(
                    "Process did not shut down within 30 seconds after receiving SIGTERM"
                )

        except Exception:
            # Clean up if something goes wrong
            if self.process.poll() is None:  # Process still running
                self.process.kill()
                self.process.wait()
            raise
