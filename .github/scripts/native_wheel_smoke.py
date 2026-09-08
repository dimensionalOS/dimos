#!/usr/bin/env python3
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

"""Record and read native artifacts from a core-only wheel installation.

Run with the installed environment's Python, from outside the checkout. Nix
is required; preparation may download or build the pinned native package.
"""

from dataclasses import replace
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import uuid

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.native_package import ensure_native_package, source_revision
from dimos.core.transport import LCMTransport
from dimos.experimental.memory.rust_cli_recorder import RustRecordingSession, make_plan
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu


def main() -> None:
    assert not (DIMOS_PROJECT_ROOT / ".git").exists(), "must import the installed wheel"
    assert not (DIMOS_PROJECT_ROOT / "dimos/experimental/memory/rust").exists()
    executable = ensure_native_package("dimos-memory-recorder")
    assert executable.is_file()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    url = f"udpm://239.255.76.67:{port}?ttl=0"
    transport = LCMTransport(f"/native-wheel-{uuid.uuid4().hex}", Imu, url=url)
    with tempfile.TemporaryDirectory(prefix="dimos-native-recording-") as directory:
        try:
            transport.start()
            for kind, suffix in (("sqlite", "db"), ("mcap", "mcap")):
                global_config.update(
                    record_engine="rust", record=kind, record_topics="*", replay=False
                )
                plan = make_plan({("imu", Imu): transport})
                # Artifact location is explicit; executable preparation stays unmodified.
                plan = replace(plan, path=Path(directory) / f"memory.{suffix}")
                session = RustRecordingSession(plan)
                try:
                    session.start()
                    for _ in range(3):
                        transport.broadcast(
                            None, Imu(ts=22.5, frame_id="imu", angular_velocity=Vector3(1, 2, 3))
                        )
                        time.sleep(0.05)
                finally:
                    session.stop()
                result = subprocess.run(
                    [sys.executable, "-m", "dimos.cli.dimos", "mem", "summary", str(plan.path)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                assert "imu" in result.stdout, result.stdout
                if kind == "mcap":
                    subprocess.run(
                        [
                            sys.executable,
                            "-m",
                            "dimos.cli.dimos",
                            "mem",
                            "rerun",
                            str(plan.path),
                            "--no-gui",
                        ],
                        check=True,
                    )
                    assert plan.path.with_suffix(".rrd").is_file()
        finally:
            transport.stop()
    print(f"Native wheel smoke passed: {source_revision()} -> {executable}")


if __name__ == "__main__":
    main()
