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

"""Check a pull request description supplied as Markdown on standard input."""

from collections import defaultdict
import re
import string
import sys

MAX_LENGTH = 5000
CLA = re.compile(
    r" {0,3}[-*+] \[([ xX])\] I have read and approved the "
    r"(?:CLA|\[CLA\]\([^)]+\))\.?[ \t]*",
    re.IGNORECASE,
)
PLACEHOLDERS = {"", "n/a", "na", "none", "tbd", "todo"}


def check_description(body: str) -> list[str]:
    body = re.sub(r"<!--.*?(?:-->|\Z)", "", body, flags=re.DOTALL).replace("\r\n", "\n")
    length = len(body.strip())
    errors = []
    if length > MAX_LENGTH:
        errors.append(f"Shorten the description to {MAX_LENGTH} characters (currently {length}).")
    if "\u2014" in body:
        errors.append("Remove em dashes from the description.")

    sections: dict[str, list[str]] = defaultdict(list)
    section = ""
    fence = ""
    checked_cla = False
    for line in body.splitlines():
        if fence:
            if re.fullmatch(rf" {{0,3}}{fence[0]}{{{len(fence)},}}[ \t]*", line):
                fence = ""
                continue
        elif marker := re.match(r"^ {0,3}(`{3,}|~{3,})", line):
            fence = marker[1]
            continue
        elif heading := re.match(r"^ {0,3}(#{1,2})(?:[ \t]+(.*)|$)", line):
            title = re.sub(r"[ \t]+#+[ \t]*$", "", heading[2] or "")
            section = title.strip().rstrip("?:").lower() if heading[1] == "##" else ""
            continue
        elif checkbox := CLA.fullmatch(line):
            checked_cla |= checkbox[1].lower() == "x"
            continue
        sections[section].append(line)

    for required in ("Why do we need this?", "What is this feature?"):
        answer = "\n".join(sections[required.lower().rstrip("?")])
        if answer.strip(string.whitespace + string.punctuation).lower() in PLACEHOLDERS:
            errors.append(f'Fill in "## {required}".')

    if re.search(r"\bpytest\b", "\n".join(sections["how to manually run this"]), re.IGNORECASE):
        errors.append(
            'Remove pytest commands from "## How to manually run this". '
            "Give manual verification steps or leave the section empty."
        )
    if not checked_cla:
        errors.append("Check the template's CLA acknowledgment with [x].")
    return errors


if __name__ == "__main__":
    errors = check_description(sys.stdin.read())
    for error in errors:
        print(f"::error::{error}")
    sys.exit(bool(errors))
