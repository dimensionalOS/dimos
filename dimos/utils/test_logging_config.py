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

"""Tests for structured logging, file routing, and the compact console formatter."""

from __future__ import annotations

import json
import logging
import os
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


def _run_logging_script(tmp_path: Path, code: str) -> None:
    # Logging configuration and handler migration affect the entire interpreter.
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import json
from pathlib import Path
from dimos.utils import logging_config

root = Path.cwd()
logging_config.LOG_DIR = root / "logs"
"""
            + code,
        ],
        cwd=tmp_path,
        env=os.environ | {"DIMOS_RUN_LOG_DIR": "", "DIMOS_LOG_LEVEL": "INFO"},
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stderr == "", result.stderr


# Subprocess scripts isolate global logging state and handler replacement from pytest.
# _run_logging_script turns failed child assertions into test failures.
def test_log_file_is_created_on_the_first_record(tmp_path: Path) -> None:
    _run_logging_script(
        tmp_path,
        """
logger = logging_config.setup_logger()
logger.debug("filtered record")
assert list(logging_config.LOG_DIR.glob("*.jsonl")) == []

logger.info("first record")
log_file, = logging_config.LOG_DIR.glob("*.jsonl")
assert json.loads(log_file.read_text())["event"] == "first record"
""",
    )


def test_set_run_log_dir_moves_earlier_records(tmp_path: Path) -> None:
    _run_logging_script(
        tmp_path,
        """
logger = logging_config.setup_logger().bind(module="planner")
logger.info("before", value=7)
early_file, = logging_config.LOG_DIR.glob("*.jsonl")
logging_config.set_run_log_dir(root / "run")
try:
    raise ValueError("diagnostic details")
except ValueError:
    logger.exception("after")

records = [json.loads(line) for line in (root / "run/main.jsonl").read_text().splitlines()]
assert [record["event"] for record in records] == ["before", "after"]
assert all(record["module"] == "planner" for record in records)
assert all({"timestamp", "level", "logger", "func_name", "lineno"} <= record.keys() for record in records)
assert records[0]["value"] == 7
assert "ValueError: diagnostic details" in records[1]["exception"]
assert not early_file.exists()
""",
    )


def test_switching_runs_preserves_the_previous_log(tmp_path: Path) -> None:
    _run_logging_script(
        tmp_path,
        """
logger = logging_config.setup_logger()
logging_config.set_run_log_dir(root / "first")
logger.info("first run")
logging_config.set_run_log_dir(root / "second")
logger.info("second run")

assert json.loads((root / "first/main.jsonl").read_text())["event"] == "first run"
assert json.loads((root / "second/main.jsonl").read_text())["event"] == "second run"
""",
    )


def test_child_run_preserves_inherited_parent_log(tmp_path: Path) -> None:
    _run_logging_script(
        tmp_path,
        '''
import subprocess
import sys

logger = logging_config.setup_logger()
logging_config.set_run_log_dir(root / "parent")
logger.info("parent before")
subprocess.run(
    [sys.executable, "-c", """
from pathlib import Path
from dimos.utils.logging_config import setup_logger, set_run_log_dir
logger = setup_logger()
set_run_log_dir(Path.cwd() / "child")
logger.info("child record")
"""],
    check=True,
    timeout=10,
)
logger.info("parent after")

parent_records = [json.loads(line) for line in (root / "parent/main.jsonl").read_text().splitlines()]
assert [record["event"] for record in parent_records] == ["parent before", "parent after"]
assert json.loads((root / "child/main.jsonl").read_text())["event"] == "child record"
''',
    )


def test_forked_child_cannot_move_parent_startup_log(tmp_path: Path) -> None:
    _run_logging_script(
        tmp_path,
        """
import multiprocessing

logger = logging_config.setup_logger()
logger.info("parent before")
early_file, = logging_config.LOG_DIR.glob("*.jsonl")

def child():
    logging_config.set_run_log_dir(root / "child")
    logger.info("child record")

process = multiprocessing.get_context("fork").Process(target=child)
process.start()
try:
    process.join(timeout=5)
    assert process.exitcode == 0
finally:
    if process.is_alive():
        process.terminate()
        process.join(timeout=5)
    process.close()
logger.info("parent after")

parent_records = [json.loads(line) for line in early_file.read_text().splitlines()]
assert [record["event"] for record in parent_records] == ["parent before", "parent after"]
assert json.loads((root / "child/main.jsonl").read_text())["event"] == "child record"
""",
    )


@pytest.mark.parametrize("destination", ['Path("run")', 'root / "alias"'])
def test_equivalent_run_paths_preserve_records(tmp_path: Path, destination: str) -> None:
    _run_logging_script(
        tmp_path,
        """
logger = logging_config.setup_logger()
logging_config.set_run_log_dir(root / "run")
logger.info("before")
(root / "alias").symlink_to(root / "run", target_is_directory=True)
"""
        + f"logging_config.set_run_log_dir({destination})\n"
        + """
logger.info("after")
records = [json.loads(line) for line in (root / "run/main.jsonl").read_text().splitlines()]
assert [record["event"] for record in records] == ["before", "after"]
""",
    )


@pytest.mark.parametrize("link_method", ["symlink_to", "hardlink_to"])
def test_startup_file_alias_is_not_copied_or_deleted(tmp_path: Path, link_method: str) -> None:
    _run_logging_script(
        tmp_path,
        """
logger = logging_config.setup_logger()
logger.info("before")
early_file, = logging_config.LOG_DIR.glob("*.jsonl")
(root / "run").mkdir()
"""
        + f'(root / "run/main.jsonl").{link_method}(early_file)\n'
        + """
logging_config.set_run_log_dir(root / "run")
logger.info("after")
assert early_file.exists()
records = [json.loads(line) for line in (root / "run/main.jsonl").read_text().splitlines()]
assert [record["event"] for record in records] == ["before", "after"]
""",
    )


@pytest.mark.parametrize(
    "setup",
    [
        'logger = logging_config.setup_logger(level=30)\nlogging_config.set_run_log_dir(root / "run")',
        'logging_config.set_run_log_dir(root / "run")\nlogger = logging_config.setup_logger(level=30)',
    ],
    ids=["logger-before-run", "run-before-logger"],
)
def test_quiet_run_can_be_followed_before_its_first_record(tmp_path: Path, setup: str) -> None:
    _run_logging_script(
        tmp_path,
        setup
        + """
from contextlib import closing
import os
from dimos.core import run_registry
from dimos.core.log_viewer import follow_log, resolve_log_path

run_registry.REGISTRY_DIR = root / "registry"
run_registry.RunEntry("quiet", os.getpid(), "demo", "2026-11-11T00:00:00Z", str(root / "run")).save()
logger.info("filtered startup")
path = resolve_log_path("quiet")
assert path == root / "run/main.jsonl"
assert path.read_text() == ""

def emit_warning():
    # The follower calls stop after opening the file and seeking to its end.
    logger.warning("first warning")
    return False

with closing(follow_log(path, stop=emit_warning)) as records:
    assert json.loads(next(records))["event"] == "first warning"
""",
    )


def test_console_prints_traceback_of_exc_info(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setenv("DIMOS_LOG_LEVEL", "INFO")
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(tmp_path))
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
from dimos.utils.logging_config import setup_logger
logger = setup_logger()
try:
    1 / 0
except ZeroDivisionError:
    logger.error("Error during worker shutdown", exc_info=True)
""",
        ],
        capture_output=True,
        text=True,
        timeout=20,
        check=True,
    )
    lines = result.stdout.splitlines()
    assert lines[0].endswith("Error during worker shutdown")
    assert lines[1] == "Traceback (most recent call last):"
    assert lines[-1] == "ZeroDivisionError: division by zero"


@pytest.mark.parametrize(
    "level,disable_level",
    [
        ("INFO", logging.NOTSET),
        ("ERROR", logging.NOTSET),
        ("CRITICAL", logging.NOTSET),
        ("FATAL", logging.NOTSET),
        ("INFO", logging.CRITICAL),
    ],
)
def test_uncaught_exception_prints_traceback_once(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, level: str, disable_level: int
) -> None:
    monkeypatch.setenv("DIMOS_LOG_LEVEL", level)
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(tmp_path))
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import logging
import sys
from dimos.utils.logging_config import setup_exception_handler

setup_exception_handler()
logging.disable(int(sys.argv[1]))
raise RuntimeError("uncaught sentinel")
""",
            str(disable_level),
        ],
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )

    assert result.returncode == 1
    output = result.stdout + result.stderr
    assert output.count("Traceback (most recent call last):") == 1
    assert output.count("RuntimeError: uncaught sentinel") == 1
