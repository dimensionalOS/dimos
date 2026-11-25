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
VALID = (
    f"## Why do we need this?\n#42\n## What is this feature?\nPin protobuf.\n## Checklist\n{CLA}"
)


class TestPrDescription(unittest.TestCase):
    def test_current_template(self) -> None:
        self.assertEqual(len(check_description(TEMPLATE)), 3)
        filled = TEMPLATE.replace("## Why do we need this?", "## Why do we need this?\n#42")
        filled = filled.replace(
            "## What is this feature?", "## What is this feature?\nPin protobuf."
        )
        self.assertEqual(check_description(filled.replace("[ ]", "[X]")), [])

    def test_length_and_punctuation_ignore_comments(self) -> None:
        body = VALID + "\n" + "x" * (MAX_LENGTH - len(VALID) - 1)
        self.assertEqual(check_description(body + "<!--\n—\n-->"), [])
        self.assertEqual(len(check_description(body + "—")), 2)

    def test_require_real_answers_and_cla(self) -> None:
        self.assertEqual(check_description(VALID), [])
        empty = VALID.replace("#42", "N/A").replace("Pin protobuf.", "<!-- TODO -->")
        for fake_cla in ("- [x] I ran tests.", f"<!-- {CLA} -->", f"```md\n{CLA}\n```"):
            with self.subTest(fake_cla=fake_cla):
                self.assertEqual(len(check_description(empty.replace(CLA, fake_cla))), 3)
        self.assertEqual(len(check_description(f"~~~~md\n{VALID}\n~~~~")), 3)

    def test_pytest_only_in_manual_steps(self) -> None:
        manual = "\n## How to manually run this\nOpen the viewer and confirm the image appears.\n"
        self.assertEqual(check_description(VALID + manual), [])
        for command in ("pytest", "uv run pytest -q", "python3 -m pytest", "./bin/pytest-slow"):
            with self.subTest(command=command):
                block = f"```sh\n## A shell comment\n{command}\n```"
                self.assertEqual(len(check_description(VALID + manual + block)), 1)
                self.assertEqual(check_description(VALID + manual + "## CI\n" + block), [])
