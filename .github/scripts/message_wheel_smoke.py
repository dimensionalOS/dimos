#!/usr/bin/env python3
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

"""Verify the standalone built-in wheel without installing the robot runtime."""

import json
from pathlib import Path

import dimos_generated as messages
from dimos_generated_schemas.provider import package_root

assert messages.__dimos_version__ == "0.1.0"
header = messages.std_msgs.msg.Header()
assert type(header).decode(header.encode()).encode() == header.encode()
root = Path(package_root())
metadata = json.loads((root / "message-package.json").read_text())
assert metadata["shared"]
assert "std_msgs/msg/Header" in metadata["owned"]
assert (root / "schemas/std_msgs/msg/Header.msg").is_file()
assert (root / "lib/cmake/dimos_generated/dimos_generatedConfig.cmake").is_file()
assert (root / "rust/Cargo.toml").is_file()
