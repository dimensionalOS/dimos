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

"""Set basic_server's usage mode (TCP 30001, Type 1101 / Command 5, Items {Mode: n}), as m20tool's setUsageMode does.

usage: set_usage_mode.py <mode>   (0 regular, 1 navigation). Not a motion command: it only picks which inputs basic_server takes.
"""

import json
import socket
import struct
import sys
import time

AOS = "10.21.31.103"
mode = int(sys.argv[1])
payload = json.dumps(
    {
        "PatrolDevice": {
            "Type": 1101,
            "Command": 5,
            "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "Items": {"Mode": mode},
        }
    },
    separators=(",", ":"),
).encode()
message_id = 4242
frame = (
    bytes([0xEB, 0x91, 0xEB, 0x90])
    + struct.pack("<HH", len(payload), message_id)
    + bytes([1])
    + bytes(7)
    + payload
)
with socket.create_connection((AOS, 30001), timeout=3) as s:
    s.sendall(frame)
    header = b""
    while len(header) < 16:
        header += s.recv(16 - len(header))
    length = struct.unpack_from("<H", header, 4)[0]
    body = b""
    while len(body) < length:
        body += s.recv(length - len(body))
print("reply id", struct.unpack_from("<H", header, 6)[0], body.decode(errors="replace"))
