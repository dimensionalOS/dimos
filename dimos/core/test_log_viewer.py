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

import pytest

from dimos.core.log_viewer import follow_log


@pytest.mark.parametrize(
    ("count", "history"),
    [(None, ["one\n", "two\n", "three\n"]), (2, ["two\n", "three\n"]), (0, [])],
)
def test_follow_shows_requested_history_then_appended_lines(tmp_path, count, history):
    path = tmp_path / "main.jsonl"
    path.write_text("one\ntwo\nthree\n")
    appended = False

    def stop():
        nonlocal appended
        if appended:
            return True
        with path.open("a") as stream:
            stream.write("four\n")
        appended = True
        return False

    assert list(follow_log(path, stop=stop, count=count)) == [*history, "four\n"]
