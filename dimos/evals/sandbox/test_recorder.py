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

import json
from pathlib import Path
import socket
import threading
import time

import zenoh

from dimos.evals.sandbox.recorder import NEW_KEY, Recording, record
from dimos.evals.sandbox.robot import router_config
from dimos.msgs.geometry_msgs.Twist import Twist


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _client(endpoint: str) -> zenoh.Session:
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    return zenoh.open(config)


def test_recorder_sees_both_directions_through_the_router(tmp_path: Path, capsys) -> None:
    endpoint = f"tcp/127.0.0.1:{_free_port()}"
    path = tmp_path / "zenoh.db"
    stop = threading.Event()
    with zenoh.open(router_config([endpoint], [])):
        recorder = threading.Thread(target=record, args=(endpoint, path, stop))
        recorder.start()
        with _client(endpoint) as robot, _client(endpoint) as agent:
            commands = []
            subscriber = robot.declare_subscriber("robot/cmd_vel/json", commands.append)
            time.sleep(1.0)  # subscriptions propagate through the router
            robot.put("dimos/cmd_vel/geometry_msgs.Twist", Twist(linear=(0.2, 0, 0)).lcm_encode())
            agent.put("robot/cmd_vel/json", b'{"vx": 0.2}', attachment=b"a")
            time.sleep(1.0)
            subscriber.undeclare()
        stop.set()
        recorder.join(timeout=5)
    assert len(commands) == 1
    out = capsys.readouterr().out
    assert NEW_KEY + "robot/cmd_vel/json" in out
    recording = Recording(path)
    assert recording.keys() == ["dimos/cmd_vel/geometry_msgs.Twist", "robot/cmd_vel/json"]
    (sample,) = recording.samples("robot/**")
    assert sample.decode() == {"vx": 0.2} and sample.attachment == b"a" and sample.kind == "PUT"
    twist = recording.last("dimos/cmd_vel/*")
    assert twist is not None and twist.decode().linear.x == 0.2
