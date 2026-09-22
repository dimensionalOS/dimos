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

"""Record an independently generated custom type with an unchanged Rust binary."""

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
from typing import Any
import uuid

from demo_native import free_port, stop_process
from external_telemetry.demo_msgs.msg import Telemetry
from mcap.reader import make_reader

from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.experimental.memory.rust_recorder import RustStreamSpec
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool


def record(backend: str, executable: Path, output: Path) -> None:
    endpoint = f"tcp/127.0.0.1:{free_port(socket.SOCK_STREAM)}"
    url = f"udpm://239.255.76.67:{free_port(socket.SOCK_DGRAM)}?ttl=0"
    ready, written = threading.Event(), threading.Event()
    transcript: list[str] = []
    with ExitStack() as stack:
        pool = ZenohSessionPool()
        stack.callback(pool.close_all)
        topic = f"demo/{uuid.uuid4().hex[:8]}"
        publisher: Any
        if backend == "lcm":
            publisher = LCMTransport(topic, Telemetry, url=url)
        else:
            publisher = ZenohTransport(
                topic,
                Telemetry,
                session_pool=pool,
                scouting=False,
                multicast=False,
                gossip=False,
                listen=[endpoint],
                connect=[],
            )
        stack.callback(publisher.stop)
        publisher.start()
        spec = RustStreamSpec.from_type(
            port="telemetry", name="telemetry", payload_type=Telemetry, codec="cdr"
        )
        session = ZenohConfig(
            mode="client",
            connect=[endpoint],
            listen=[],
            multicast=False,
            gossip=False,
            scouting=False,
            connect_timeout=10,
        ).to_wire()
        process = stack.enter_context(
            subprocess.Popen(
                [str(executable.resolve())],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env={
                    **os.environ,
                    "DIMOS_TRANSPORT": backend,
                    "LCM_DEFAULT_URL": url,
                    "RUST_LOG": "info,dimos_memory_recorder=debug",
                },
            )
        )

        def read_logs() -> None:
            assert process.stdout is not None
            for raw in process.stdout:
                line = raw.decode(errors="replace")
                transcript.append(line)
                if "memory recorder ready" in line:
                    ready.set()
                if "memory recorder batch written" in line:
                    written.set()

        reader = threading.Thread(target=read_logs, name="recorder-demo-logs")
        reader.start()
        stack.callback(reader.join, 5)
        stack.callback(stop_process, process)
        assert process.stdin is not None
        launch = {
            "topics": {"telemetry": publisher.channel},
            "session": session if backend == "zenoh" else {},
            "config": {
                "store": {"kind": "mcap", "path": str(output.resolve())},
                "encoding_threads": 2,
                "streams": [spec.model_dump()],
            },
        }
        process.stdin.write(json.dumps(launch).encode() + b"\n")
        process.stdin.close()
        assert ready.wait(15), "".join(transcript)
        expected = []
        for index in range(3):
            message = Telemetry(application_note=f"locally-added-field-{index}")
            message.header.stamp.sec = 1700000000
            message.header.stamp.nanosec = 123456789 + index
            expected.append(message)
            written.clear()
            publisher.publish(message)
            assert written.wait(5), "".join(transcript)
        stop_process(process)
        assert process.returncode == 0, "".join(transcript)
    with output.open("rb") as stream:
        records = list(make_reader(stream).iter_messages())
    assert len(records) == len(expected)
    for (schema, channel, row), original in zip(records, expected, strict=True):
        assert schema is not None and schema.encoding == "ros2msg"
        assert b"application_note" in schema.data
        assert channel.message_encoding == "cdr"
        assert row.publish_time == row.log_time  # Unknown stamp layout uses reception time.
        decoded = Telemetry.decode(row.data)
        assert decoded == original
        print(
            f"{backend}: {decoded.application_note}, payload stamp={decoded.header.stamp.sec}.{decoded.header.stamp.nanosec:09d}"
        )
    output.with_suffix(".log").write_text("".join(transcript))
    print(f"Recorded {len(records)} independently generated messages with full schema: {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--executable", type=Path, default=Path("target/debug/dimos-memory-recorder")
    )
    parser.add_argument("--output", type=Path, default=Path("build/message-codegen/demo/evidence"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for backend in ("lcm", "zenoh"):
        record(backend, args.executable, args.output / f"external-native-recording-{backend}.mcap")
    print("PASS: custom field recorded on both transports without a recorder decoder or rebuild")


if __name__ == "__main__":
    main()
