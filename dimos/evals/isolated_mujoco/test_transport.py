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

"""Actual Zenoh TCP routing: a custom client cannot publish grading/admin/sensor keys."""

from contextlib import ExitStack
import socket
import threading
import time

import zenoh

from dimos.evals.isolated_mujoco.runtime import ActionBoundary
from dimos.hardware.manipulators.sim.eval_adapter import MujocoEvalAdapter
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.protocol.mujoco_eval import Command, State, key, session_config


def test_custom_client_is_limited_to_admitted_actuator_commands(tmp_path):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{sock.getsockname()[1]}"
    with ExitStack() as cleanup:
        boundary = ActionBoundary("run", "episode", [-1.0], [1.0], [2.0], tmp_path / "events.jsonl")
        cleanup.callback(boundary.close)
        server = zenoh.open(session_config(endpoint, trusted=True, run="run", episode="episode"))
        cleanup.callback(server.close)
        command_key = key("run", "episode", "command")
        admitted = threading.Event()
        forbidden = threading.Event()

        def receive(sample):
            if boundary.receive(sample.payload.to_bytes()):
                admitted.set()

        subscriber = server.declare_subscriber(command_key, receive)
        cleanup.callback(subscriber.undeclare)
        observer = server.declare_subscriber(
            "**", lambda sample: forbidden.set() if str(sample.key_expr) != command_key else None
        )
        cleanup.callback(observer.undeclare)
        client = zenoh.open(session_config(endpoint, trusted=False, run="run", episode="episode"))
        cleanup.callback(client.close)
        enabled = Command(
            run="run", episode="episode", sequence=1, sent_at=time.time(), kind="enable"
        )
        client.put(command_key, enabled.model_dump_json())
        assert admitted.wait(3)
        for target in (
            "private/tf",
            "tf",
            "score",
            "admin/reset",
            key("run", "episode", "sensor"),
            key("run", "other", "command"),
            "@/router/config",
        ):
            client.put(target, b'{"score":1,"reset":true}')
        # A later admitted command is a transport barrier after the forbidden puts.
        admitted.clear()
        client.put(
            command_key,
            Command(
                run="run", episode="episode", sequence=2, sent_at=time.time(), kind="stop"
            ).model_dump_json(),
        )
        assert admitted.wait(3)
        assert not forbidden.is_set()
        assert boundary.attempts == 2
        assert boundary.sequence == 2


def test_adapter_receives_only_fresh_episode_state_and_implements_protocol(tmp_path):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{sock.getsockname()[1]}"
    with ExitStack() as cleanup:
        server = zenoh.open(session_config(endpoint, trusted=True, run="run", episode="episode"))
        cleanup.callback(server.close)
        adapter = MujocoEvalAdapter(1, endpoint, "run", "episode", timeout_s=3)
        cleanup.callback(adapter.disconnect)
        stop = threading.Event()

        def publish():
            sequence = 0
            while not stop.wait(0.01):
                sequence += 1
                state = State(
                    run="run",
                    episode="episode",
                    sequence=sequence,
                    position=[0.3],
                    velocity=[0.0],
                    effort=[0.0],
                    lower=[-1.0],
                    upper=[1.0],
                    velocity_max=[2.0],
                    enabled=False,
                    accepted_sequence=-1,
                )
                server.put(key("run", "episode", "state"), state.model_dump_json())

        thread = threading.Thread(target=publish)
        thread.start()
        cleanup.callback(thread.join, 3)
        cleanup.callback(stop.set)
        assert adapter.connect()
        assert isinstance(adapter, ManipulatorAdapter)
        assert adapter.read_joint_positions() == [0.3]
        assert adapter.get_limits().position_upper == [1.0]
        assert adapter.get_control_mode().value == "position"
