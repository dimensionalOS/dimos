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

from dimos.core.stream import In
from dimos.memory.module import Recorder
from dimos.memory.store.sqlite import SqliteStore


class NumberRecorder(Recorder):
    numbers: In[int]


@pytest.mark.parametrize(("mode", "expected"), [("resume", [1, 2]), ("append", [2])])
def test_resume_preserves_rows_and_legacy_append_replaces_selected_streams(
    tmp_path, mocker, mode, expected
):
    path = tmp_path / "recording.db"
    store = SqliteStore(path=path)
    try:
        store.stream("numbers", int).append(1, ts=1.0)
        store.stream("unrelated", int).append(9, ts=1.0)
    finally:
        store.stop()
    recorder = NumberRecorder(db_path=path, on_existing=mode, record_tf=False)
    mocker.patch.object(recorder, "_port_to_stream")
    try:
        recorder.start()
        recorder.store.stream("numbers", int).append(2, ts=2.0)
        assert [item.data for item in recorder.store.stream("numbers")] == expected
        assert [item.data for item in recorder.store.stream("unrelated")] == [9]
    finally:
        recorder.stop()
