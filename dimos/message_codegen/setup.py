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

"""Build already-prepared upstream sources without fetching at install time."""

from hashlib import sha256
import json
from pathlib import Path

from setuptools import setup

if not Path(__file__).with_name(".upstream").joinpath("rosidl_adapter/parser.py").is_file():
    raise RuntimeError(
        "Upstream parser is not prepared. Run python scripts/prepare_message_parser.py "
        "from the repository root (use --offline for a populated cache), then build/install. "
        "Published backend wheels and sdists already contain the verified source."
    )
lock = json.loads(Path(__file__).with_name("parser-source.json").read_text())
parser = Path(__file__).with_name(".upstream").joinpath("rosidl_adapter/parser.py")
if sha256(parser.read_bytes()).hexdigest() != lock["parser_sha256"]:
    raise RuntimeError("Prepared upstream parser differs from its pinned hash; prepare again")
setup()
