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

"""Create or verify the same value across ordinary installed language packages."""

import argparse
from pathlib import Path

from acme_telemetry.telemetry_msgs.msg import DeviceReading
from dimos_cdrgen.registry import decode, encode

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("write", "verify"))
    parser.add_argument("file", type=Path)
    args = parser.parse_args()
    if args.action == "write":
        value = DeviceReading(Header(Time(17, 23), "map"), 7, 2.5, "sensor")
        args.file.write_bytes(encode(value))
    else:
        value = decode(args.file.read_bytes(), DeviceReading)
        assert value == DeviceReading(Header(Time(17, 23), "map"), 7, 4.5, "sensor/cpp/rust")
        assert type(value.header) is Header
        print("PASS: Python -> C++ -> Rust -> Python; one shared Header identity")


if __name__ == "__main__":
    main()
