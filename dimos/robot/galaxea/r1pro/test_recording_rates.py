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

"""Rates read off a synthetic recording with known gaps."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3

import pytest

from dimos.robot.galaxea.r1pro.recording_rates import (
    check_requirements,
    main,
    parse_requirement,
    stream_rates,
    window_hz,
)


def _write_recording(path: Path, streams: dict[str, list[float]]) -> Path:
    """The tables the reader looks at, and nothing else.

    `_streams` and a per-stream `(id, ts)` table are the whole contract with
    :class:`RawRecording`; the payload type is only imported, never decoded,
    because a rate is a property of the stamps alone.
    """
    connection = sqlite3.connect(path)
    connection.execute("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")
    for name, stamps in streams.items():
        connection.execute(
            "INSERT INTO _streams (name, config) VALUES (?, ?)",
            (name, json.dumps({"payload_module": "builtins.bytes", "codec_id": "lcm"})),
        )
        connection.execute(
            f'CREATE TABLE "{name}" (id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL)'
        )
        connection.executemany(f'INSERT INTO "{name}" (ts) VALUES (?)', [(t,) for t in stamps])
    connection.commit()
    connection.close()
    return path


def _steady(hz: float, seconds: float, start: float = 1000.0) -> list[float]:
    return [start + i / hz for i in range(int(seconds * hz) + 1)]


@pytest.fixture()
def recording(tmp_path: Path) -> Path:
    stalled = [t for t in _steady(30.0, 60.0) if not (1020.0 <= t < 1030.0)]
    return _write_recording(
        tmp_path / "rec.db",
        {
            "head_left_color": _steady(30.0, 60.0),
            "head_right_color": stalled,
            "head_left_info": _steady(1.0, 60.0),
            "lone": [1000.0],
            "empty": [],
        },
    )


def test_a_steady_stream_reads_as_its_rate_everywhere(recording: Path) -> None:
    rate = stream_rates(recording)["head_left_color"]
    assert rate.count == 1801
    assert rate.first_ts == pytest.approx(1000.0)
    assert rate.last_ts == pytest.approx(1060.0)
    assert rate.duration_s == pytest.approx(60.0)
    assert rate.mean_hz == pytest.approx(30.0)
    assert rate.window_min_hz == pytest.approx(30.0, abs=0.2)
    assert rate.window_median_hz == pytest.approx(30.0, abs=0.2)
    assert rate.window_max_hz == pytest.approx(30.0, abs=0.2)


def test_a_stall_shows_in_the_window_minimum_not_only_the_mean(recording: Path) -> None:
    # Ten seconds missing from sixty: the mean is off by a sixth, but the
    # window lined up with the stall is where it is actually visible.
    rate = stream_rates(recording, window_s=10.0)["head_right_color"]
    assert rate.mean_hz == pytest.approx(30.0 * 50 / 60, abs=0.1)
    assert rate.window_min_hz == pytest.approx(0.0)
    assert rate.window_max_hz == pytest.approx(30.0, abs=0.2)


def test_a_slow_stream_is_slow(recording: Path) -> None:
    rate = stream_rates(recording)["head_left_info"]
    assert rate.count == 61
    assert rate.mean_hz == pytest.approx(1.0)


def test_one_message_or_none_has_no_rate(recording: Path) -> None:
    rates = stream_rates(recording)
    lone, empty = rates["lone"], rates["empty"]
    assert (lone.count, lone.first_ts, lone.mean_hz, lone.window_min_hz) == (1, 1000.0, None, None)
    assert (empty.count, empty.first_ts, empty.last_ts, empty.mean_hz) == (0, None, None, None)


def test_streams_can_be_narrowed_and_an_unknown_one_is_an_error(recording: Path) -> None:
    assert list(stream_rates(recording, streams=["lone", "empty"])) == ["lone", "empty"]
    with pytest.raises(KeyError, match="nope"):
        stream_rates(recording, streams=["nope"])


def test_window_hz_over_a_recording_shorter_than_a_window_is_the_overall_rate() -> None:
    assert window_hz(_steady(10.0, 3.0), window_s=10.0) == [pytest.approx(10.0)]
    assert window_hz([5.0], window_s=10.0) == []
    assert window_hz([5.0, 5.0], window_s=10.0) == []


def test_window_hz_slides_by_half_a_window() -> None:
    # 30 s at 2 Hz, 10 s windows stepping 5 s: starts at 0, 5, ..., 20.
    rates = window_hz(_steady(2.0, 30.0, start=0.0), window_s=10.0)
    assert len(rates) == 5
    assert all(r == pytest.approx(2.0, abs=0.1) for r in rates)


def test_requirements_pass_fail_and_name_the_missing(recording: Path) -> None:
    rates = stream_rates(recording)
    assert check_requirements(rates, {"head_left_color": 28.0}) == []
    failures = check_requirements(rates, {"head_right_color": 28.0, "absent": 1.0, "lone": 1.0})
    assert len(failures) == 3
    assert failures[0].startswith("head_right_color: 25.")
    assert "28 required" in failures[0]
    assert "absent: not in the recording" in failures[1]
    assert failures[2].startswith("lone: 1 message(s), no rate")


def test_parse_requirement() -> None:
    assert parse_requirement("head_left_color=28") == ("head_left_color", 28.0)
    with pytest.raises(SystemExit):
        main(["x.db", "--require", "head_left_color"])
    with pytest.raises(SystemExit):
        main(["x.db", "--require", "head_left_color=fast"])


def test_cli_prints_a_row_per_stream_and_exits_zero(recording: Path, capsys) -> None:
    assert main([str(recording)]) == 0
    out = capsys.readouterr().out
    for name in ("head_left_color", "head_right_color", "head_left_info", "lone", "empty"):
        assert name in out
    assert "n/a" in out  # the lone and empty streams
    assert "min10s" in out


def test_cli_fails_on_an_unmet_requirement(recording: Path, capsys) -> None:
    code = main(
        [str(recording), "--require", "head_left_color=28", "--require", "head_right_color=28"]
    )
    captured = capsys.readouterr()
    assert code == 1
    assert "head_left_color" in captured.out
    assert "rate requirements not met" in captured.err
    assert "head_right_color" in captured.err
    assert "head_left_color" not in captured.err


def test_cli_reports_a_missing_file_or_stream(tmp_path: Path, recording: Path, capsys) -> None:
    assert main([str(tmp_path / "missing.db")]) == 2
    assert "missing.db" in capsys.readouterr().err
    assert main([str(recording), "--streams", "head_left_color,nope"]) == 2
    assert "nope" in capsys.readouterr().err
