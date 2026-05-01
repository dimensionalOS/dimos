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

from dimos.constants import DIMOS_PROJECT_ROOT

PATTERN = "= logging.getLogger"

IGNORED_DIRS = {
    ".venv",
    "venv",
    "__pycache__",
    "node_modules",
    ".git",
    "dist",
    "build",
    ".egg-info",
    ".tox",
}

# Lines that match the pattern but are legitimate uses.
WHITELIST = [
    ("dimos/utils/logging_config.py", "logger_obj = logging.getLogger(logger_name)"),
    ("dimos/utils/logging_config.py", "stdlib_logger = logging.getLogger(name)"),
    ("dimos/core/coordination/python_worker.py", "lg = logging.getLogger(name)"),
    ("dimos/robot/foxglove_bridge.py", "logger = logging.getLogger(logger)"),
    (
        "dimos/hardware/sensors/camera/gstreamer/gstreamer_sender.py",
        'logger = logging.getLogger("gstreamer_tcp_sender")',
    ),
    ("dimos/core/test_async_module_main.py", 'target = logging.getLogger("dimos/core/module.py")'),
]


def _is_ignored_dir(dirpath: str) -> bool:
    parts = dirpath.split(os.sep)
    return bool(IGNORED_DIRS.intersection(parts))


def _is_whitelisted(rel_path: str, line: str) -> bool:
    for allowed_path, allowed_substr in WHITELIST:
        if rel_path == allowed_path and allowed_substr in line:
            return True
    return False


def find_get_logger_usages() -> list[tuple[str, int, str]]:
    """Return a list of (rel_path, line_number, line_text) for every violation."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    violations: list[tuple[str, int, str]] = []
    # Skip this test file.
    self_path = os.path.realpath(__file__)

    for dirpath, dirnames, filenames in os.walk(dimos_dir):
        dirnames[:] = [d for d in dirnames if d not in IGNORED_DIRS]

        if _is_ignored_dir(dirpath):
            continue

        for fname in filenames:
            if not fname.endswith(".py"):
                continue

            full_path = os.path.join(dirpath, fname)
            if os.path.realpath(full_path) == self_path:
                continue
            rel_path = os.path.relpath(full_path, DIMOS_PROJECT_ROOT)

            try:
                with open(full_path, encoding="utf-8", errors="replace") as f:
                    for lineno, line in enumerate(f, start=1):
                        stripped = line.rstrip("\n")
                        if PATTERN not in stripped:
                            continue
                        if _is_whitelisted(rel_path, stripped):
                            continue
                        violations.append((rel_path, lineno, stripped))
            except (OSError, UnicodeDecodeError):
                continue

    return violations


def test_no_get_logger():
    """
    Fail if any file uses `= logging.getLogger` outside the whitelist.
    """
    violations = find_get_logger_usages()
    if violations:
        report_lines = [
            f"Found {len(violations)} forbidden use(s) of `logging.getLogger`. "
            "Use `setup_logger` instead:",
            "",
            "    from dimos.utils.logging_config import setup_logger",
            "",
            "    logger = setup_logger()",
            "",
            "If the usage is legitimate (e.g. standalone script, logging "
            "infrastructure, or third-party logger suppression), add it to the "
            "WHITELIST in dimos/project/test_get_logger.py.",
            "",
        ]
        for path, lineno, text in violations:
            report_lines.append(f"  {path}:{lineno}: {text.strip()}")
        raise AssertionError("\n".join(report_lines))
