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

"""Open the phone page on loopback and inspect five decoded browser CDR frames."""

import argparse
from queue import Queue
import socket
import threading

from dimos_generated.geometry_msgs.msg import TwistStamped
from dimos_message_build.registry import decode as cdr_decode
import uvicorn

from dimos.teleop.phone.phone_teleop_module import PhoneTeleopModule


class PhonePreview(PhoneTeleopModule):
    def __init__(self) -> None:
        self.received: Queue[TwistStamped] = Queue()
        super().__init__()

    def _on_sensors_bytes(self, data: bytes) -> None:
        super()._on_sensors_bytes(data)
        self.received.put(cdr_decode(data, TwistStamped))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    module = PhonePreview()
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
        server = uvicorn.Server(uvicorn.Config(module._web_server.app, log_level="warning"))
        thread = threading.Thread(target=server.run, kwargs={"sockets": [listener]})
        thread.start()
        try:
            print(f"Open http://127.0.0.1:{port}/teleop and click Connect", flush=True)
            message = None
            for _ in range(5):
                message = module.received.get(timeout=args.timeout)
            assert message is not None
            assert message.header.frame_id == "phone"
            assert message.header.stamp.sec > 0
            print(
                f"Browser → {message.__msgtype__}: stamp={message.header.stamp.sec}s + "
                f"{message.header.stamp.nanosec}ns",
                flush=True,
            )
            print(
                "PASS: browser uses advertised schemas, generic CDR writer and explicit channel/type frames",
                flush=True,
            )
        finally:
            server.should_exit = True
            thread.join(timeout=10)
            assert not thread.is_alive(), "demo web server did not stop"
            module.stop()


if __name__ == "__main__":
    main()
