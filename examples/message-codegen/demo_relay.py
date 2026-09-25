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

"""Show one locally defined message crossing Python, C++, Rust, then Python."""

import argparse
from pathlib import Path
import subprocess

from dimos_generated.demo_msgs.msg import Telemetry


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve()
    outputs = build / "evidence"
    outputs.mkdir(exist_ok=True)
    message = Telemetry(sequence=40, hops=[1], payload=[0, 127, 255])
    message.header.frame_id = "map"
    message.position.x = 4.25
    (outputs / "python.cdr").write_bytes(message.encode())
    print(
        f"Python sends: sequence={message.sequence} label={message.label} hops={message.hops}",
        flush=True,
    )
    subprocess.run(
        [str(build / "cpp-relay"), "edit", str(outputs / "python.cdr"), str(outputs / "cpp.cdr")],
        check=True,
    )
    subprocess.run(
        [
            str(build / "rust" / "target" / "debug" / "relay"),
            "edit",
            str(outputs / "cpp.cdr"),
            str(outputs / "rust.cdr"),
        ],
        check=True,
    )
    received = Telemetry.decode((outputs / "rust.cdr").read_bytes())
    print(
        f"Python receives: sequence={received.sequence} label={received.label} hops={received.hops}"
    )
    assert received.sequence == 42
    assert received.label == "start/cpp/rust"
    assert received.hops == [1, 2, 3]
    assert received.payload == [0, 127, 255]
    assert received.reading.temperature == 21.5
    assert received.position.x == 4.25
    print(f"Nested fields, fixed arrays, bytes, defaults, and edits survived. Evidence: {outputs}")


if __name__ == "__main__":
    main()
