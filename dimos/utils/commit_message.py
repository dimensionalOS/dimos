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

import os
from pathlib import Path
import re
import subprocess
import sys

# A personal name alone is not enough to identify an AI co-author.
AI_NAME = (
    r"Claude(?:[ \t]+(?:Code|Opus|Sonnet|Haiku)\b[^<>\r\n]*)?"
    r"|(?:OpenAI[ \t]+)?Codex|(?:GitHub[ \t]+)?Copilot"
    r"|Cursor(?:[ \t]+Agent)?|(?:Google[ \t]+)?Gemini|Windsurf|Devin|Aider"
)
AI_COAUTHOR = re.compile(
    r"^[ \t]*Co-authored-by[ \t]*:[ \t]*"
    r"(?:"
    r"[^<>\r\n]*<(?:noreply@(?:anthropic\.com|openai\.com|cursor\.com)|cursoragent@cursor\.com"
    r"|(?:codex|copilot|devin-ai-integration)@users\.noreply\.github\.com)>"
    rf"|(?:{AI_NAME})[ \t]*<(?:bot|noreply|no-reply)@[^<>\r\n]+>"
    r"|(?:Claude[ \t]+Code|OpenAI[ \t]+Codex|GitHub[ \t]+Copilot"
    r"|Cursor[ \t]+Agent|Google[ \t]+Gemini)[ \t]*<[^<>\r\n]+>"
    r")",
    re.IGNORECASE | re.MULTILINE,
)
AI_SIGNATURE_NAME = (
    r"Claude(?: Code)?|(?:OpenAI )?Codex|(?:GitHub )?Copilot|Cursor(?: Agent)?"
    r"|(?:Google )?Gemini|Windsurf|Devin|Aider"
)
GENERATED_SIGNATURE = re.compile(
    r"^[ \t]*(?:🤖[ \t]+)?Generated with[ \t]+"
    rf"(?:\[(?:{AI_SIGNATURE_NAME})\]\(https?://[^)\s]+\)|(?:{AI_SIGNATURE_NAME}))[ \t]*$",
    re.IGNORECASE,
)


def filter_text(text: str) -> tuple[str, str | None]:
    """Return (filtered_text, first_matched_pattern_or_None)."""
    lines = text.splitlines(keepends=True)
    filtered_lines: list[str] = []
    matched: str | None = None
    for line in lines:
        hit = AI_COAUTHOR.search(line) or GENERATED_SIGNATURE.search(line)
        if hit is not None:
            matched = hit.group().strip()
            break
        filtered_lines.append(line)
    return "".join(filtered_lines), matched


def rewrite_file(path: Path) -> int:
    if not path.exists():
        return 0
    filtered, _ = filter_text(path.read_text())
    path.write_text(filtered)
    return 0


def check_commits() -> int:
    """Check incoming commits using the base/head range supplied by CI."""
    from_ref = os.environ.get("PRE_COMMIT_FROM_REF")
    to_ref = os.environ.get("PRE_COMMIT_TO_REF")
    if not (from_ref and to_ref):
        print("Both PRE_COMMIT_FROM_REF and PRE_COMMIT_TO_REF are required.", file=sys.stderr)
        return 1

    try:
        rev_list = subprocess.run(
            ["git", "rev-list", "--reverse", f"{from_ref}..{to_ref}"],
            capture_output=True,
            text=True,
            check=True,
        )
    except subprocess.CalledProcessError as e:
        print(
            f"git rev-list {from_ref}..{to_ref} failed: {e.stderr.strip() or e}",
            file=sys.stderr,
        )
        return 1

    failures: list[tuple[str, str]] = []
    for sha in rev_list.stdout.split():
        try:
            msg = subprocess.run(
                ["git", "log", "-1", "--format=%B", sha],
                capture_output=True,
                text=True,
                check=True,
            ).stdout
        except subprocess.CalledProcessError as e:
            print(
                f"git log -1 {sha} failed: {e.stderr.strip() or e}",
                file=sys.stderr,
            )
            return 1
        matched = AI_COAUTHOR.search(msg)
        if matched is not None:
            failures.append((sha, matched.group().strip()))

    if failures:
        for sha, pattern in failures:
            print(
                f"{sha[:12]}: AI co-author: {pattern!r}",
                file=sys.stderr,
            )
        print(
            "\nAmend the offending commits to remove AI co-author trailers, "
            "then push the updated branch.",
            file=sys.stderr,
        )
        return 1
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(
            "Usage: python -m dimos.utils.commit_message <commit-msg-file> | --check",
            file=sys.stderr,
        )
        return 1

    if sys.argv[1] == "--check":
        return check_commits()

    return rewrite_file(Path(sys.argv[1]))


if __name__ == "__main__":
    sys.exit(main())
