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
import queue
import time

import pytest_mock

from dimos.imitation.collection.recorder import CollectionRecorder, _RecordItem
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream


def test_collection_recorder_fifo_drains_all_accepted_messages(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    db_path = tmp_path / "recording.db"
    recorder = CollectionRecorder(db_path=db_path, queue_maxsize=100)
    stream = recorder.store.stream("numbers", int)
    resolve_pose = mocker.patch.object(recorder, "_resolve_pose")
    recorder._start_record_queue()

    for value in range(20):
        recorder._submit_record(_RecordItem("numbers", stream, value, time.time()))

    recorder.stop()

    assert recorder.recording_metrics() == {
        "numbers": {"received": 20, "written": 20, "queued": 0, "dropped": 0, "failed": 0}
    }
    resolve_pose.assert_not_called()
    with SqliteStore(path=str(db_path), must_exist=True) as reopened:
        observations = reopened.stream("numbers", int).to_list()
    assert [observation.data for observation in observations] == list(range(20))


def test_collection_recorder_drop_new_saturation_is_counted(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    recorder = CollectionRecorder(
        db_path=tmp_path / "recording.db",
        queue_maxsize=1,
        drop_warning_interval=0,
    )
    recorder._record_queue = queue.Queue(maxsize=1)
    recorder._accepting_records = True
    stream = mocker.MagicMock(spec=Stream)

    recorder._submit_record(_RecordItem("numbers", stream, 1, time.time()))
    recorder._submit_record(_RecordItem("numbers", stream, 2, time.time()))

    metrics = recorder.recording_metrics()["numbers"]
    assert metrics["received"] == 2
    assert metrics["queued"] == 1
    assert metrics["dropped"] == 1
    recorder._record_queue = None
    recorder.stop()
