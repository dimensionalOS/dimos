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

"""Verify the external message package built by build_example.py."""

from importlib.metadata import distribution
from importlib.util import find_spec

from dimos_cdrgen.registry import decode, encode
from dimos_messages.builtin_interfaces.msg import Time
from dimos_messages.std_msgs.msg import Header
from dimos_messages_schemas.provider import package_root

from acme_telemetry.demo_msgs.msg import Reading
from acme_telemetry_schemas.provider import package_root as external_root


def main() -> None:
    assert find_spec("dimos_generated") is None
    header = Header(Time(17, 23), "map")
    reading = Reading(header=header, value=21.5)
    restored = decode(encode(reading), Reading)
    assert restored == reading
    assert type(restored.header) is Header
    assert (external_root() / "schemas/std_msgs/msg/Header.msg").is_file()
    assert (external_root() / "schemas/demo_msgs/msg/Reading.msg").is_file()
    assert not (external_root() / "cpp/demo_msgs").exists()
    assert all("dimos-generated" not in req for req in distribution("acme-telemetry").requires or ())
    print(f"Reused built-in schemas from: {package_root()}")
    print("PASS: external Reading message shares the installed dimos_messages Header identity")


if __name__ == "__main__":
    main()
