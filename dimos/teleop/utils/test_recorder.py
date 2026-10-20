# Copyright 2025-2026 Dimensional Inc.
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

"""Canonical hosted/WebXR ports survive real recording and offline reporting."""

import asyncio
from contextlib import ExitStack
import json
from threading import Event

from dimos_cdrgen.registry import encode
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.core.transport import pLCMTransport, pZenohTransport
from dimos.memory.store.sqlite import SqliteStore
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    TwistStamped,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import Joy
from dimos.messages.std_msgs.msg import Header, UInt32
from dimos.teleop.hosted.hosted_stats import HostedStatsModule
from dimos.teleop.utils.recorder import TeleopRecorder
from dimos.web.relay_bridge.protocol import FrameHeader, encode_data_frame


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_hosted_and_webxr_ports_record_canonical_values_and_report_after_reopen(tmp_path, backend):
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
    names = [
        "left_controller_output",
        "right_controller_output",
        "teleop_buttons",
        "cmd_vel_stamped",
        "video_stats",
        "robot_telemetry",
    ]
    events = {name: Event() for name in names}
    with ExitStack() as cleanup:
        stats = HostedStatsModule()
        cleanup.callback(stats.stop)
        recorder = TeleopRecorder(
            db_path=tmp_path / "recording_teleop.db", record_tf=False, poseless_streams=names
        )

        def close_recorder():
            # Await the owned dispatchers' cancellation before the module closes its loop.
            if recorder._disposables is not None:
                recorder._disposables.dispose()
            if recorder._loop is not None and recorder._loop.is_running():
                asyncio.run_coroutine_threadsafe(asyncio.sleep(0), recorder._loop).result(timeout=5)
            recorder.stop()

        cleanup.callback(close_recorder)
        wires = {}
        for name in names:
            if name == "robot_telemetry":
                factory = pLCMTransport if backend == "lcm" else pZenohTransport
                wire = factory("teleop_recording_" + name, **options)
            else:
                wire = CdrTransport(
                    "teleop_recording_" + name,
                    getattr(recorder, name).type,
                    backend=backend,
                    **options,
                )
            cleanup.callback(wire.stop)
            wires[name] = wire
            recorder.set_transport(name, wire)
        stats.set_transport("cmd_vel_stamped", wires["cmd_vel_stamped"])
        stats.set_transport("video_stats", wires["video_stats"])
        recorder.start()
        path = recorder._db_path
        for name in names:
            live = recorder.store.streams[name].live()
            cleanup.callback(live.stop)
            cleanup.enter_context(
                live.subscribe(lambda observation, event=events[name]: event.set())
            )
        pose = PoseStamped(
            Header(Time(42, 1), "right"), Pose(Point(1, 2, 3), Quaternion(0, 0, 0, 1))
        )
        for name in ("left_controller_output", "right_controller_output"):
            wires[name].publish(pose)
            assert events[name].wait(5)
        wires["teleop_buttons"].publish(UInt32(0x100))
        assert events["teleop_buttons"].wait(5)
        for nanos in (0, 50_000_000):
            event = events["cmd_vel_stamped"]
            event.clear()
            value = TwistStamped(
                Header(Time(42, nanos), "operator"), Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
            )
            frame = encode_data_frame(
                FrameHeader(
                    ch="twist",
                    seq=0,
                    ts=42,
                    delivery="latest",
                    meta={"encoding": "cdr", "type": TwistStamped.__msgtype__},
                ),
                encode(value),
            )
            stats._on_cmd_raw(frame)
            assert event.wait(5)
        stats._on_state_json(
            json.dumps(
                {
                    "type": "video_stats",
                    "ts": 42.25,
                    "fps": 30,
                    "kbps": 1000,
                    "width": 640,
                    "height": 480,
                    "frames_dropped": 2,
                    "freezes": 1,
                }
            ).encode()
        )
        assert events["video_stats"].wait(5)
        wires["robot_telemetry"].publish(json.dumps(stats._telemetry_payload()).encode())
        assert events["robot_telemetry"].wait(5)
        assert recorder.inputs["teleop_buttons"].type is UInt32
        assert recorder.inputs["video_stats"].type is Joy
    assert path is not None
    with SqliteStore(path=str(path), must_exist=True) as store:
        assert [item.data for item in store.streams.right_controller_output] == [pose]
        assert [item.data for item in store.streams.teleop_buttons] == [UInt32(0x100)]
        recorded = list(store.streams.cmd_vel_stamped)
        assert [v.data.header.stamp for v in recorded] == [Time(42, 0), Time(42, 50_000_000)]
        assert [v.ts for v in recorded] == [42.0, 42.05]
        assert all(type(v.data) is TwistStamped and v.data.twist.linear.x == 0.5 for v in recorded)
        assert type(next(iter(store.streams.video_stats)).data) is Joy
    reports = list(tmp_path.glob("report_*.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text())
    assert report["duration_s"] == 0.25
    assert report["streams"]["cmd_vel_stamped"]["count"] == 2
    assert report["streams"]["cmd_vel_stamped"]["rate_hz"] == pytest.approx(20)
    assert report["streams"]["teleop_buttons"]["count"] == 1
    assert report["streams"]["teleop_buttons"]["rate_hz"] is None
    assert report["video"]["resolution"] == "640x480"
    assert report["video"]["fps"]["p50"] == 30
    assert report["video"]["frames_dropped"] == 2
    assert report["video"]["freezes"] == 1
    assert report["telemetry"]["count"] == 1
