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

from collections.abc import Iterator
from contextlib import ExitStack
import threading

import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport_factory import make_transport
from dimos.memory.buffer import Unbounded
from dimos.memory.module import Recorder, StreamModule
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.memory.transform import Transformer
from dimos.memory.type.observation import Observation
from dimos.messages.builtin_interfaces.msg import Time as GeneratedTime
from dimos.messages.dimos_msgs.msg import LineSegments3D as GeneratedSegments
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header as GeneratedHeader
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.threadpool import get_max_workers


class Double(Transformer[int, int]):
    def __init__(self, factor: int = 2) -> None:
        self.factor = factor

    def __call__(self, upstream: Iterator[Observation[int]]) -> Iterator[Observation[int]]:
        for obs in upstream:
            yield obs.derive(data=obs.data * self.factor)


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


@pytest.mark.asyncio
async def test_recorder_reads_generated_header_without_network(mocker, tmp_path):
    # Isolate the module's RPC and event-loop resource boundaries.
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    mocker.patch("dimos.protocol.service.lcmservice.lcm_mod.LCM")
    mocker.patch.object(LCMRPC, "start")
    mocker.patch.object(LCMRPC, "serve_module_rpc")
    recorder = Recorder(rpc_transport=LCMRPC, db_path=tmp_path / "unused.db", record_tf=False)
    transform_tree = mocker.Mock()
    mocker.patch.object(recorder, "_tf", transform_tree)
    transform_tree.get.return_value = TransformStamped(
        GeneratedHeader(GeneratedTime(0, 0), "world"),
        "sensor_frame",
        Transform(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
    message = GeneratedSegments(GeneratedHeader(GeneratedTime(0, 0), "sensor_frame"), [])
    try:
        assert recorder._resolve_ts("segments", message) == 0.0
        pose = await recorder._resolve_pose("segments", message, 0.0)
        assert type(pose) is Pose
        assert pose.position.x == 1.0
        assert pose.position.y == 2.0
        assert pose.position.z == 3.0
        assert pose.orientation.w == 1.0
        transform_tree.get.assert_called_once_with(
            "world",
            "sensor_frame",
            time_point=0.0,
            time_tolerance=0.5,
        )
    finally:
        recorder.stop()
    assert not (tmp_path / "unused.db").exists()


class MemqRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


@pytest.fixture
def tf_recording_scheduler(monkeypatch):
    scheduler = ThreadPoolScheduler(max_workers=get_max_workers())
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    monkeypatch.setattr("dimos.utils.threadpool.get_scheduler", lambda: scheduler)
    yield scheduler
    scheduler.executor.shutdown(wait=True)


def test_recorder_tf_cdr_boundary_records_each_transform_without_changing_header(
    mocker, monkeypatch, tmp_path, tf_recording_scheduler
):
    monkeypatch.setenv("LCM_DEFAULT_URL", "memq://")
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    mocker.patch.object(LCMRPC, "start")
    mocker.patch.object(LCMRPC, "serve_module_rpc")
    path = tmp_path / "tf.db"
    transforms = [
        TransformStamped(
            GeneratedHeader(GeneratedTime(0, nanos), "world"),
            "base_link",
            Transform(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        )
        for nanos in (0, 1)
    ]
    ready = threading.Event()
    observed = []

    def receive(observation):
        observed.append(observation)
        if len(observed) == 2:
            ready.set()

    with ExitStack() as cleanup:
        recorder = Recorder(rpc_transport=MemqRPC, db_path=path)
        cleanup.callback(recorder.stop)
        transport = make_transport("tf", TFMessage, g=GlobalConfig(transport="lcm"))
        cleanup.callback(transport.stop)
        recorder.set_transport("tf", transport)
        recorder._record_tf()
        live = recorder.store.streams.tf.live(buffer=Unbounded())
        cleanup.callback(live.stop)
        cleanup.enter_context(live.subscribe(receive))
        transport.publish(TFMessage(transforms=transforms))
        assert ready.wait(5), "TF recorder did not append both transforms"
        assert [observation.ts for observation in observed] == [0.0, 1e-9]
    with SqliteStore(path=str(path), must_exist=True) as store:
        values = list(store.streams.tf)
        assert [item.data for item in values] == [TFMessage([value]) for value in transforms]
        assert [item.data.transforms[0].header.stamp.nanosec for item in values] == [0, 1]
        assert transforms[1].header.frame_id == "world"
