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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Run a locally extended, independently packaged message through both native SDKs."""

import argparse
from pathlib import Path

from demo_native import exchange_native
from external_telemetry.demo_msgs.msg import Telemetry


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, default=Path("build/message-codegen/external-native"))
    args = parser.parse_args()
    evidence = args.build / "evidence"
    evidence.mkdir(parents=True, exist_ok=True)
    for backend in ("lcm", "zenoh"):
        message = Telemetry(application_note="added-locally")
        message.header.stamp.sec = 1700000000
        message.header.stamp.nanosec = 123456789
        message.header.frame_id = "application"
        result = exchange_native(
            backend,
            args.build / "cpp/external_native_relay",
            args.build / "rust/target/debug/external-native-relay",
            evidence,
            {"telemetry": message},
        )["telemetry"]
        assert result.application_note == "added-locally/cpp-native/rust-native"
        assert result.header.stamp.sec == 1700000000
        assert result.header.stamp.nanosec == 123456789
        assert result.header.frame_id == "application"
        message.application_note = result.application_note
        assert result.encode() == message.encode()
        print(
            f"{backend}: external Python → native C++ → native Rust → Python: {result.application_note}"
        )
    print(
        "SDK codecs accepted the external generated type directly; no handwritten codec or upstream schema PR."
    )


if __name__ == "__main__":
    main()
