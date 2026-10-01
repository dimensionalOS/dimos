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

from pathlib import Path
import re


def test_documented_sources_match_compiled_example():
    root = Path(__file__).resolve().parents[2]
    document = (root / "docs/development/messages-external-project.md").read_text()
    blocks = re.findall(r"<!-- source: ([^ ]+) -->\n```[^\n]*\n(.*?)```", document, re.S)
    assert len(blocks) >= 8
    for name, snippet in blocks:
        source = (root / name).read_text()
        if name.endswith((".py", ".cpp", ".rs")):
            lines = source.splitlines()
            comment = "#" if name.endswith(".py") else "//"
            while lines and (lines[0].startswith(comment) or not lines[0].strip()):
                lines.pop(0)
            if lines and lines[0].startswith(chr(34) * 3):
                lines.pop(0)
                while lines and not lines[0].strip():
                    lines.pop(0)
            source = "\n".join(lines) + "\n"
        assert snippet == source, name
