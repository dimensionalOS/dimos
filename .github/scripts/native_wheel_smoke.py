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
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import uuid

from mcap.reader import make_reader

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.native_package import ensure_native_package, source_revision
from dimos.core.transport import LCMTransport
from dimos.experimental.memory.rust_cli_recorder import RustRecordingSession, make_plan
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.std_msgs.String import String


def main() -> None:
    assert not (DIMOS_PROJECT_ROOT / ".git").exists(), "must import the installed wheel"
    assert not (DIMOS_PROJECT_ROOT / "dimos/experimental/memory/rust").exists()
    executable = ensure_native_package("dimos-memory-recorder")
    assert executable.is_file()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    url = f"udpm://239.255.76.67:{port}?ttl=0"
    transport = LCMTransport(f"/wheel-imu-{uuid.uuid4().hex[:8]}", Imu, url=url)
    status_transport = LCMTransport(f"/wheel-json-{uuid.uuid4().hex[:8]}", String, url=url)
    schema = {"type": "object", "properties": {"sent": {"type": "number"}}}
    expected_imu = Imu(ts=22.5, frame_id="imu", angular_velocity=Vector3(1, 2, 3))
    expected_status = String(json.dumps({"sent": 23.5, "state": "ready"}))
    with tempfile.TemporaryDirectory(prefix="dimos-native-recording-") as directory:
        try:
            transport.start()
            status_transport.start()
            for kind, suffix in (("sqlite", "db"), ("mcap", "mcap")):
                global_config.update(
                    record_engine="rust", record=kind, record_topics="*", replay=False
                )
                plan = make_plan({("imu", Imu): transport, ("status", String): status_transport})
                plan = replace(
                    plan,
                    streams=[
                        spec.model_copy(
                            update={
                                "codec": "json",
                                "timestamp_field": "sent",
                                "json_schema": schema,
                            }
                        )
                        if spec.name == "status"
                        else spec
                        for spec in plan.streams
                    ],
                )
                # Artifact location is explicit; executable preparation stays unmodified.
                plan = replace(plan, path=Path(directory) / f"memory.{suffix}")
                session = RustRecordingSession(plan)
                try:
                    session.start()
                    for _ in range(3):
                        transport.broadcast(None, expected_imu)
                        status_transport.broadcast(None, expected_status)
                        time.sleep(0.05)
                finally:
                    session.stop()
                store_type = SqliteStore if kind == "sqlite" else McapStore
                with store_type(path=str(plan.path)) as store:
                    assert store.list_streams() == ["imu", "status"]
                    imu = store.stream("imu").order_by("ts").first()
                    status = store.stream("status").order_by("ts").first()
                    assert imu.ts == 22.5
                    assert imu.data.lcm_encode() == expected_imu.lcm_encode()
                    assert status.ts == 23.5
                    assert json.loads(status.data.data) == json.loads(expected_status.data)
                if kind == "mcap":
                    with plan.path.open("rb") as artifact:
                        summary = make_reader(artifact).get_summary()
                        assert summary is not None
                        channels = {channel.topic: channel for channel in summary.channels.values()}
                        for name, payload_type, codec in (
                            ("imu", Imu, "lcm"),
                            ("status", String, "json"),
                        ):
                            channel = channels[name]
                            assert channel.message_encoding == codec
                            assert channel.metadata["dimos.payload_type"] == (
                                f"{payload_type.__module__}.{payload_type.__qualname__}"
                            )
                            assert channel.metadata["dimos.observation_time"] == "publish_time"
                            assert channel.metadata["dimos.stream_name"] == name
                        json_schema = summary.schemas[channels["status"].schema_id]
                        assert json_schema.encoding == "jsonschema"
                        assert json.loads(json_schema.data) == schema
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
            status_transport.stop()
            transport.stop()
    print(f"Native wheel smoke passed: {source_revision()} -> {executable}")


if __name__ == "__main__":
    main()
