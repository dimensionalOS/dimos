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

import asyncio
from contextlib import ExitStack

import numpy as np
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.imitation.dataprep.core import extract_episodes, resolve_field
from dimos.imitation.dataprep.schema import EpisodeExtractor, FeatureSpec
from dimos.memory.store.sqlite import SqliteStore
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import Image, JointState
from dimos.messages.std_msgs.msg import Header, String, UInt32
from dimos.teleop.webxr.controller_types import BUTTON_ALIASES, Buttons
from dimos.utils.testing.waiting import wait_until


async def test_poseless_collection_stream_skips_pose_lookup(tmp_path) -> None:
    with ExitStack() as cleanup:
        recorder = CollectionRecorder(
            db_path=tmp_path / "poseless.db", poseless_streams=["commands"]
        )
        cleanup.callback(recorder.stop)
        pose = await recorder._resolve_pose("commands", object(), 1.0)
        assert pose is None


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_canonical_collection_records_button_boundaries_and_named_features_after_reopen(
    tmp_path,
    backend,
    mocker,
):
    options = (
        {"url": "memq://"}
        if backend == "lcm"
        else {
            "scouting": False,
            "multicast": False,
            "gossip": False,
            "connect": [],
            "listen": ["tcp/127.0.0.1:0"],
        }
    )
    path = tmp_path / "collection.db"
    clock = [10.0]
    mocker.patch(
        "dimos.imitation.collection.episode_monitor.time.time", side_effect=lambda: clock[0]
    )
    with ExitStack() as cleanup:
        monitor = EpisodeMonitorModule(task="pick")
        recorder = CollectionRecorder(
            db_path=path,
            record_tf=False,
            poseless_streams=["color_image", "coordinator_joint_state", "status"],
        )

        def close_modules():
            monitor.stop()
            if recorder._disposables is not None:
                recorder._disposables.dispose()
            if recorder._loop is not None and recorder._loop.is_running():
                asyncio.run_coroutine_threadsafe(asyncio.sleep(0), recorder._loop).result(timeout=5)
            recorder.stop()

        # Stop modules before their external transport resources, including on failure.
        wires = {}
        for name, kind in [
            ("teleop_buttons", UInt32),
            ("status", String),
            ("color_image", Image),
            ("coordinator_joint_state", JointState),
        ]:
            wire = CdrTransport("/collection/" + name, kind, backend=backend, **options)
            cleanup.callback(wire.stop)
            wires[name] = wire
            if name in recorder.inputs:
                recorder.set_transport(name, wire)
            if name in monitor.inputs or name in monitor.outputs:
                monitor.set_transport(name, wire)
        cleanup.callback(close_modules)
        recorder.start()
        monitor.start()
        wait_until(lambda: recorder.store.stream("status").count() == 1, timeout=5)
        # Use the configured event mapping rather than a hard-coded button alias.
        alias = monitor.config.button_map["toggle"]
        bit = 1 << Buttons.BITS[BUTTON_ALIASES.get(alias, alias)]
        wires["teleop_buttons"].publish(UInt32(bit))
        wait_until(lambda: recorder.store.stream("status").count() == 2, timeout=5)
        image = Image(
            Header(Time(10, 1), "camera"),
            2,
            2,
            "rgb8",
            0,
            8,
            np.asarray([1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 99, 99], dtype=np.uint8),
        )
        state = JointState(
            Header(Time(10, 1), "arm"),
            ["right", "left"],
            np.asarray([2.0, 1.0]),
            np.zeros(2),
            np.zeros(2),
        )
        wires["color_image"].publish(image)
        wires["coordinator_joint_state"].publish(state)
        wait_until(
            lambda: (
                recorder.store.stream("color_image").count() == 1
                and recorder.store.stream("coordinator_joint_state").count() == 1
            ),
            timeout=5,
        )
        clock[0] = 12.0
        monitor.command("save")
        wait_until(lambda: recorder.store.stream("status").count() == 3, timeout=5)
    with SqliteStore(path=path, must_exist=True) as store:
        (episode,) = extract_episodes(store, EpisodeExtractor())
        assert (episode.start_ts, episode.end_ts, episode.task_label, episode.success) == (
            10.0,
            12.0,
            "pick",
            True,
        )
        saved = store.stream("coordinator_joint_state", JointState).first().data
        assert type(saved) is JointState and saved.header.stamp.nanosec == 1
        values = resolve_field(
            saved,
            FeatureSpec(
                stream="state",
                field="position",
                dtype="float32",
                shape=(2,),
                names=["left", "right"],
            ),
        )
        np.testing.assert_array_equal(values, [1.0, 2.0])
        stored_image = store.stream("color_image", Image).first().data
        pixels = resolve_field(
            stored_image,
            FeatureSpec(
                stream="camera", field="data", dtype="video", shape=(2, 2, 3), names=["h", "w", "c"]
            ),
        )
        np.testing.assert_array_equal(pixels, np.arange(1, 13, dtype=np.uint8).reshape(2, 2, 3))
        assert stored_image.header.stamp.nanosec == 1
        assert type(store.stream("status", String).first().data) is String
