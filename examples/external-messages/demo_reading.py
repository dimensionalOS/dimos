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

"""Register an external message and reuse the installed built-in Header."""

from importlib.metadata import entry_points

from acme_telemetry.telemetry_msgs.msg import DeviceReading
from acme_telemetry_schemas.provider import package_root
from dimos_cdrgen.registry import decode, encode

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header


def main() -> None:
    assert (
        len([ep for ep in entry_points(group="dimos.messages") if ep.name == "acme_telemetry"]) == 1
    )
    value = DeviceReading(Header(Time(17, 23), "map"), 7, 2.5, "sensor")
    restored = decode(encode(value), DeviceReading)
    assert restored == value
    assert type(restored.header) is Header
    print(f"Installed external message sources: {package_root()}")
    print("PASS: external message registered; CDR roundtrip reuses built-in Header")


if __name__ == "__main__":
    main()
