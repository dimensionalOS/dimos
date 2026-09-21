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

"""Run from a fresh environment containing only the external message wheel."""

import argparse
from importlib.metadata import entry_points
from pathlib import Path
import subprocess

from external_telemetry.demo_msgs.msg import Telemetry


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve()
    provider = next(
        entry.load()
        for entry in entry_points(group="dimos.messages")
        if entry.name == "external_telemetry"
    )
    assert provider.message_types()[Telemetry.msg_name] is Telemetry
    assert (provider.schema_root() / "demo_msgs/msg/Telemetry.msg").is_file()
    value = Telemetry(application_note="added-locally")
    print(f"Installed Python package sends new field: {value.application_note}", flush=True)
    first, second, third = [build / name for name in ("python.cdr", "cpp.cdr", "rust.cdr")]
    first.write_bytes(value.encode())
    subprocess.run([str(build / "cpp-consumer/relay"), str(first), str(second)], check=True)
    subprocess.run(
        [str(build / "rust-consumer/target/debug/external-consumer"), str(second), str(third)],
        check=True,
    )
    decoded = Telemetry.decode(third.read_bytes())
    assert decoded.application_note == "added-locally/cpp/rust"
    print(f"Installed Python package receives: {decoded.application_note}")
    print(
        "A locally added field crossed three installed native packages; no DimOS source change or upstream PR."
    )


if __name__ == "__main__":
    main()
