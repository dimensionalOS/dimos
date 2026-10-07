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
import subprocess
import sys

from dimos.core.run_registry import RunEntry
from dimos.gateway import logs


def test_parses_structlog_and_raw_lines() -> None:
    record = logs.parse_line(
        '{"timestamp":"t","level":"WARNING","logger":"nav","event":"stuck","x":1}'
    )
    assert record is not None
    assert (record["level"], record["logger"], record["extra"]) == ("warning", "nav", {"x": 1})
    assert logs.is_problem(record)
    raw = logs.parse_line("Traceback (most recent call last):")
    assert raw is not None and raw["level"] == "raw" and not logs.is_problem(raw)
    assert logs.parse_line("   ") is None


def test_tails_with_offsets(tmp_path: Path) -> None:
    file = tmp_path / "main.jsonl"
    file.write_text(
        '{"level":"info","event":"a"}\n{"level":"error","event":"b"}\n{"level":"info","ev'
    )
    page = logs.read_file(file, "r1", None, 100, logs.Filter())
    assert [r["event"] for r in page["records"]] == ["a", "b"]
    errors = logs.read_file(file, "r1", None, 100, logs.Filter(min_level="error"))
    assert [r["event"] for r in errors["records"]] == ["b"]
    with file.open("a") as handle:
        handle.write('ent":"c"}\n')
    after = logs.read_file(file, "r1", page["offset"], 100, logs.Filter())
    assert [r["event"] for r in after["records"]] == ["c"]


def test_reads_the_newest_run_or_one_by_id(tmp_path: Path, server_home: Path) -> None:
    for run in ("20260101-000000-a", "20260102-000000-b"):
        (tmp_path / "logs" / run).mkdir(parents=True)
        (tmp_path / "logs" / run / "main.jsonl").write_text(f'{{"event":"{run}","logger":"x"}}\n')
    latest = logs.read(tmp_path, "latest", None, 10, logs.Filter())
    assert latest["runId"] == "20260102-000000-b" and latest["loggers"] == ["x"]
    assert (
        logs.read(tmp_path, "20260101-000000-a", None, 10, logs.Filter())["records"][0]["event"]
        == "20260101-000000-a"
    )
    assert logs.read(tmp_path, "nope", None, 10, logs.Filter()) == {
        "runId": "nope",
        "records": [],
        "offset": 0,
        "loggers": [],
    }
    limited = logs.read(tmp_path, None, None, 10, logs.Filter(query="NOTHING"))
    assert limited["records"] == []


def test_reads_what_dimos_logs(tmp_path: Path, server_home: Path) -> None:
    """A record written by dimos's own logger, the way `dimos run` sets it up (a run's log folder), reads back."""
    log_dir = logs.LOG_DIR / "20260103-000000-real"
    script = (
        "from dimos.utils.logging_config import set_run_log_dir, setup_logger\n"
        f"log = setup_logger()\nset_run_log_dir({str(log_dir)!r})\n"
        "log.warning('stuck', x=1)\nlog.info('fine')\n"
    )
    subprocess.run([sys.executable, "-c", script], check=True, capture_output=True)
    page = logs.read(tmp_path / "elsewhere", "latest", None, 10, logs.Filter(min_level="warning"))
    assert page["runId"] == "20260103-000000-real"
    [record] = page["records"]
    assert (record["level"], record["event"], record["extra"]["x"]) == ("warning", "stuck", 1)
    assert record["timestamp"] and record["logger"] in page["loggers"]


def test_a_registered_run_is_read_from_its_log_dir(tmp_path: Path, server_home: Path) -> None:
    log_dir = tmp_path / "somewhere" / "r9"
    log_dir.mkdir(parents=True)
    (log_dir / "main.jsonl").write_text('{"event":"here","level":"info"}\n')
    RunEntry("r9", 1, "x", "t", str(log_dir)).save()
    assert logs.read(tmp_path, "r9", None, 10, logs.Filter())["records"][0]["event"] == "here"
