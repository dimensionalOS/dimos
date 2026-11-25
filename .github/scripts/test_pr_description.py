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
import unittest

from check_pr_description import MAX_LENGTH, check_description

TEMPLATE = (Path(__file__).resolve().parents[1] / "pull_request_template.md").read_text()
CLA = "- [x] I have read and approved the CLA."
LEGACY = f"## Problem\n#42\n## Solution\nPin protobuf.\n## Contributor License Agreement\n{CLA}"


class TestPrDescription(unittest.TestCase):
    def test_current_template(self) -> None:
        self.assertEqual(len(check_description(TEMPLATE)), 3)
        filled = TEMPLATE.replace("## Why do we need this?", "## Why do we need this?\n#42")
        filled = filled.replace(
            "## What is this feature?", "## What is this feature?\nPin protobuf."
        )
        self.assertEqual(check_description(filled.replace("[ ]", "[X]")), [])

    def test_length_and_punctuation_ignore_comments(self) -> None:
        body = LEGACY + "\n" + "x" * (MAX_LENGTH - len(LEGACY) - 1)
        self.assertEqual(check_description(body + "<!--\n\u2014\n-->"), [])
        self.assertEqual(len(check_description(body + "\u2014")), 2)

    def test_require_real_answers_and_cla(self) -> None:
        self.assertEqual(check_description(LEGACY), [])
        empty = LEGACY.replace("#42", "Closes DIM-XXX").replace("Pin protobuf.", "<!-- TODO -->")
        for fake_cla in ("- [x] I ran tests.", f"<!-- {CLA} -->", f"```md\n{CLA}\n```"):
            with self.subTest(fake_cla=fake_cla):
                self.assertEqual(len(check_description(empty.replace(CLA, fake_cla))), 3)
        self.assertEqual(len(check_description(f"~~~~md\n{LEGACY}\n~~~~")), 3)

    def test_automated_commands_only_in_manual_steps(self) -> None:
        for heading in ("How to manually run this", "How to Test"):
            manual = f"\n## {heading}\nOpen the viewer and confirm the image appears.\n"
            self.assertEqual(
                check_description(LEGACY + manual + "CI also has pytest coverage."), []
            )
            for command in ("pytest", "uv run pytest -q", "python3 -m pytest", "cargo test"):
                with self.subTest(heading=heading, command=command):
                    block = f"```sh\n## A shell comment\n{command}\n```"
                    self.assertEqual(len(check_description(LEGACY + manual + block)), 1)
                    self.assertEqual(check_description(LEGACY + manual + "## CI\n" + block), [])
