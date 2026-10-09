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

"""Use an application-local generated message in a Python module and file relay."""

import argparse
from pathlib import Path
import subprocess

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode, encode
from story_messages.story_msgs.msg import DeviceReading

from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out


class ReadingProcessor(Module):
    reading: In[DeviceReading]
    processed: Out[DeviceReading]

    def handle_reading(self, message: DeviceReading) -> None:
        result = decode(encode(message), DeviceReading)
        result.value += 1
        result.label += "/python"
        self.processed.publish(result)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    args = parser.parse_args()
    # This file-relay acceptance needs local RPC only, never LAN discovery.
    global_config.update(
        transport="zenoh",
        robot_ip="",
        robot_ips="",
        zenoh_connect="",
        zenoh_scouting=False,
        zenoh_multicast=False,
        zenoh_gossip=False,
    )
    processor = ReadingProcessor()
    outputs: list[DeviceReading] = []
    unsubscribe = processor.processed.subscribe(outputs.append)
    try:
        reading = DeviceReading(
            header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="sensor"),
            sequence=40,
            value=20.5,
            label="new-local-type",
        )
        processor.handle_reading(decode(encode(reading), DeviceReading))
        assert len(outputs) == 1
        output = outputs[0]
        print(f"Python module: {output.__msgtype__}, value={output.value}, label={output.label}")
        evidence = args.build / "evidence"
        evidence.mkdir(parents=True, exist_ok=True)
        (evidence / "python.cdr").write_bytes(encode(output))
        subprocess.run(
            [
                str(args.build / "cpp-consumer"),
                str(evidence / "python.cdr"),
                str(evidence / "cpp.cdr"),
            ],
            check=True,
        )
        subprocess.run(
            [
                str(args.build / "rust/target/debug/story-consumer"),
                str(evidence / "cpp.cdr"),
                str(evidence / "rust.cdr"),
            ],
            check=True,
        )
        final = decode((evidence / "rust.cdr").read_bytes(), DeviceReading)
        assert final.sequence == 42
        assert final.value == 23.5
        assert final.label == "new-local-type/python/cpp/rust"
        assert final.header.stamp.nanosec == 123456789
        print(
            f"Python receives: sequence={final.sequence}, value={final.value}, label={final.label}"
        )
        print("PASS: application-local message crosses Python module, C++, Rust and Python")
    finally:
        unsubscribe()
        processor.stop()


if __name__ == "__main__":
    main()
