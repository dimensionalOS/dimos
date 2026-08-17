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

"""Grid tests for StreamModule — same e2e logic across all pipeline styles."""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
from pathlib import Path
import time

import pytest
import pytest_mock

from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory.module import Recorder, RecorderConfig, StreamModule, _RecordItem
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.memory.transform import Transformer
from dimos.memory.type.observation import Observation

# -- Shared transformer ---------------------------------------------------


class Double(Transformer[int, int]):
    def __init__(self, factor: int = 2) -> None:
        self.factor = factor

    def __call__(self, upstream: Iterator[Observation[int]]) -> Iterator[Observation[int]]:
        for obs in upstream:
            yield obs.derive(data=obs.data * self.factor)


# -- Pipeline styles -------------------------------------------------------


class StaticStreamModule(StreamModule[int, int]):
    """Pipeline as a static Stream chain on the class."""

    pipeline = Stream().transform(Double())
    numbers: In[int]
    doubled: Out[int]


class StaticTransformerModule(StreamModule[int, int]):
    """Pipeline as a bare Transformer on the class."""

    pipeline = Double()
    numbers: In[int]
    doubled: Out[int]


class MethodPipelineConfig(ModuleConfig):
    factor: int = 2


class MethodPipelineModule(StreamModule[int, int]):
    """Pipeline as a method with access to self.config."""

    config: MethodPipelineConfig

    def pipeline(self, stream: Stream[int]) -> Stream[int]:
        return stream.transform(Double(factor=self.config.factor))

    numbers: In[int]
    doubled: Out[int]


# -- Grid ------------------------------------------------------------------

module_cases = [
    pytest.param(StaticStreamModule, id="static-stream"),
    pytest.param(StaticTransformerModule, id="static-transformer"),
    pytest.param(MethodPipelineModule, id="method-pipeline"),
]


@pytest.mark.parametrize("module_cls", module_cases)
def test_blueprint_ports(module_cls: type[StreamModule]) -> None:
    """All pipeline styles produce a blueprint with the correct In/Out ports."""
    bp = module_cls.blueprint()

    assert len(bp.blueprints) == 1
    atom = bp.blueprints[0]
    stream_names = {s.name for s in atom.streams}
    assert "numbers" in stream_names
    assert "doubled" in stream_names


async def test_poseless_stream_skips_tf_lookup(mocker: pytest_mock.MockerFixture) -> None:
    recorder = mocker.MagicMock(spec=Recorder)
    recorder.config = RecorderConfig(poseless_streams=["commands"])
    recorder._pose_setters = {}

    pose = await Recorder._resolve_pose(recorder, "commands", object(), 1.0)

    assert pose is None
    recorder.tf.get.assert_not_called()


def test_recorder_fifo_drains_all_accepted_messages(tmp_path: Path) -> None:
    db_path = tmp_path / "recording.db"
    recorder = Recorder(
        db_path=db_path,
        record_tf=False,
        poseless_streams=["numbers"],
        queue_maxsize=100,
    )
    stream = recorder.store.stream("numbers", int)
    recorder._start_record_queue()

    for value in range(20):
        recorder._submit_record(_RecordItem("numbers", stream, value, time.time()))

    recorder.stop()

    assert recorder.recording_metrics() == {
        "numbers": {"received": 20, "written": 20, "queued": 0, "dropped": 0, "failed": 0}
    }
    with SqliteStore(path=str(db_path), must_exist=True) as reopened:
        observations = reopened.stream("numbers", int).to_list()
    assert [observation.data for observation in observations] == list(range(20))


def test_recorder_drop_new_saturation_is_counted(mocker: pytest_mock.MockerFixture) -> None:
    recorder = Recorder(record_tf=False, queue_maxsize=1, drop_warning_interval=0)
    recorder._record_queue = asyncio.Queue(maxsize=1)
    recorder._accepting_records = True
    stream = mocker.MagicMock(spec=Stream)
    first = _RecordItem("numbers", stream, 1, time.time())
    second = _RecordItem("numbers", stream, 2, time.time())
    recorder._submit_record(first)
    recorder._submit_record(second)
    time.sleep(0.01)

    metrics = recorder.recording_metrics()["numbers"]
    assert metrics["received"] == 2
    assert metrics["dropped"] == 1
    recorder._record_queue = None
    recorder.stop()
