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

from contextlib import closing
from threading import Event

import pytest

from dimos.core.log_viewer import follow_log


@pytest.mark.parametrize(
    ("count", "history"),
    [(None, ["one\n", "two\n", "three\n"]), (2, ["two\n", "three\n"]), (0, [])],
)
def test_follow_shows_requested_history_then_appended_lines(tmp_path, count, history, mocker):
    path = tmp_path / "main.jsonl"
    path.write_text("one\ntwo\nthree\n")
    stopped = Event()

    def append_line(_delay):
        with path.open("a") as stream:
            stream.write("four\n")

    mocker.patch("dimos.core.log_viewer.time.sleep", side_effect=append_line)

    with closing(follow_log(path, stop=stopped.is_set, count=count)) as lines:
        assert [next(lines) for _ in history] == history
        assert next(lines) == "four\n"
        stopped.set()
        assert list(lines) == []


@pytest.mark.parametrize("count", [None, 3])
def test_follow_can_stop_during_history_replay(tmp_path, count):
    path = tmp_path / "main.jsonl"
    path.write_text("one\ntwo\nthree\n")
    stopped = Event()

    with closing(follow_log(path, stop=stopped.is_set, count=count)) as lines:
        assert next(lines) == "one\n"
        stopped.set()
        assert list(lines) == []


@pytest.mark.parametrize("count", [None, 3, 0])
def test_follow_can_stop_while_reading_history(tmp_path, count, mocker):
    stopped = Event()

    def read_lines():
        yield "one\n"
        stopped.set()
        yield "two\n"
        pytest.fail("Reading history continued after cancellation")

    open_log = mocker.mock_open()
    open_log.return_value.__iter__.side_effect = read_lines
    mocker.patch("builtins.open", open_log)

    assert list(follow_log(tmp_path / "main.jsonl", stop=stopped.is_set, count=count)) == []
