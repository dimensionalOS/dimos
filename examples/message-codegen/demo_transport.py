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

"""Verify Python/native raw LCM interoperability, including fragmented payloads."""

import argparse
from pathlib import Path
import selectors
import socket
import subprocess
import time

import lcm


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    args = parser.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port_socket:
        port_socket.bind(("127.0.0.1", 0))
        port = port_socket.getsockname()[1]
    transport = lcm.LCM(f"udpm://239.255.76.67:{port}?ttl=0&recv_buf_size=4194304")
    received: list[bytes] = []
    subscription = transport.subscribe("REPLY", lambda _channel, data: received.append(data))
    process = subprocess.Popen(
        [str(args.executable.resolve()), str(port)], stdout=subprocess.PIPE, text=True
    )
    try:
        assert process.stdout is not None
        with selectors.DefaultSelector() as ready:
            ready.register(process.stdout, selectors.EVENT_READ)
            assert ready.select(timeout=10), "Native transport did not become ready"
            assert process.stdout.readline().strip() == "READY"
        for size in (128, 65536, 1048576):
            payload = bytes(index % 251 for index in range(size))
            transport.publish("INPUT", payload)
            deadline = time.monotonic() + 10
            while not received and time.monotonic() < deadline:
                transport.handle_timeout(100)
            assert received, f"No reply for {size} bytes"
            assert received.pop(0) == payload
            print(f"Python -> native Rust -> Python: {size} raw bytes match", flush=True)
        assert process.wait(timeout=10) == 0
    finally:
        transport.unsubscribe(subscription)
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=10)
        if process.stdout is not None:
            process.stdout.close()


if __name__ == "__main__":
    main()
