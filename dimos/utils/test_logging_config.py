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

"""Tests for the compact console log formatter."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import time

import pytest

from dimos.core.log_viewer import format_line
from dimos.utils import logging_config
from dimos.utils.logging_config import _compact_console_processor


@pytest.mark.parametrize("level,debug_enabled", [("INFO", False), ("DEBUG", True)])
def test_setup_logger_level_check_matches_output(monkeypatch, tmp_path, level, debug_enabled):
    monkeypatch.setenv("DIMOS_LOG_LEVEL", level)
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(tmp_path))
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import logging
from dimos.utils.logging_config import setup_logger
logger = setup_logger()
print(logger.isEnabledFor(logging.DEBUG))
logger.debug("body details", joints=2)
logger.info("body acquired")
""",
        ],
        capture_output=True,
        text=True,
        timeout=20,
        check=True,
    )
    assert result.stdout.splitlines()[0] == str(debug_enabled)
    assert ("body details" in result.stdout) == debug_enabled
    records = [json.loads(line) for line in (tmp_path / "main.jsonl").read_text().splitlines()]
    assert [record["event"] for record in records] == (
        ["body details", "body acquired"] if debug_enabled else ["body acquired"]
    )


def test_module_key_leads_the_kv_tail(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(logging_config, "_CONSOLE_USE_COLORS", False)
    line = _compact_console_processor(
        None,
        "info",
        {
            "timestamp": "2026-08-17T12:00:00.123Z",
            "level": "info",
            "logger": "dimos/utils/thing.py",
            "event": "tick",
            "alpha": 1,
            "module": "planner",
        },
    )
    assert line.endswith("tick module=planner alpha=1")


@pytest.fixture
def tokyo_time(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("TZ", "Asia/Tokyo")
    time.tzset()
    yield
    monkeypatch.undo()
    time.tzset()


def test_console_time_is_local(monkeypatch: pytest.MonkeyPatch, tokyo_time: None) -> None:
    monkeypatch.setattr(logging_config, "_CONSOLE_USE_COLORS", False)
    line = _compact_console_processor(
        None,
        "info",
        {"timestamp": "2026-08-17T12:00:00.123Z", "level": "info", "logger": "x.py", "event": "e"},
    )
    assert line.startswith("21:00:00.123 ")


def test_log_viewer_time_is_local(tokyo_time: None) -> None:
    raw = json.dumps({"timestamp": "2026-08-17T12:00:00.123456Z", "level": "info", "event": "e"})
    assert format_line(raw).startswith("21:00:00 ")


def test_log_file_is_created_on_the_first_record(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    log_file = tmp_path / "dimos.jsonl"
    monkeypatch.setattr(logging_config, "_LOG_FILE_PATH", log_file)

    logger = logging_config.setup_logger()
    assert not log_file.exists()

    logger.info("first record")
    assert json.loads(log_file.read_text())["event"] == "first record"


def test_set_run_log_dir_moves_earlier_records(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    early_file = tmp_path / "dimos.jsonl"
    monkeypatch.setattr(logging_config, "_LOG_FILE_PATH", early_file)
    monkeypatch.setattr(logging_config, "_RUN_LOG_DIR", None)
    monkeypatch.delenv("DIMOS_RUN_LOG_DIR", raising=False)

    logger = logging_config.setup_logger()
    logger.info("before")
    logging_config.set_run_log_dir(tmp_path / "run")
    logger.info("after")

    lines = (tmp_path / "run" / "main.jsonl").read_text().splitlines()
    assert [json.loads(line)["event"] for line in lines] == ["before", "after"]
    assert not early_file.exists()
