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

"""Record the native relay blueprint to MCAP, stop it, then replay every CDR value."""

import argparse
from contextlib import ExitStack
from functools import partial
from pathlib import Path
import socket
import threading
from typing import Any
from unittest.mock import patch

from demo_blueprint import PythonRelayDemo, blueprint
from dimos_generated.dimos_msgs.msg import LineSegments3D
from dimos_generated.geometry_msgs.msg import PoseStamped
from dimos_generated.sensor_msgs.msg import Image
from mcap.reader import make_reader
import numpy as np
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.stream import In
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig, RustRecorder
from dimos.memory.recording_policy import OnExisting
from dimos.memory.replay_module import replay_module
from dimos.memory.store.mcap import McapStore
from dimos.msgs.time import to_nanoseconds
from dimos.protocol.service.system_configurator.base import configure_system
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool


class RelayRecorder(RustRecorder):
    l2: In[LineSegments3D]
    i2: In[Image]
    p0: In[PoseStamped]


def verify_and_replay(artifact: Path, samples: int) -> None:
    expected: dict[str, list[bytes]] = {}
    independent = get_typestore(Stores.ROS2_JAZZY)
    with artifact.open("rb") as source:
        for schema, channel, row in make_reader(source).iter_messages():
            assert schema is not None and schema.encoding == "ros2msg"
            assert channel.message_encoding == "cdr"
            independent.register(get_types_from_msg(schema.data.decode(), schema.name))
            independent.deserialize_cdr(row.data, schema.name)
    with McapStore(path=str(artifact)) as store:
        for name in ("l2", "i2", "p0"):
            messages = [obs.data for obs in store.stream(name)]
            expected[name] = [message.encode() for message in messages]
            observed_samples = set()
            for message in messages:
                delta = to_nanoseconds(message.header.stamp) - 1700000000123456789
                sample, remainder = divmod(delta, 100_000_000)
                assert remainder == 0
                assert 0 <= sample < samples
                observed_samples.add(sample)
                if name == "l2":
                    assert message.segments[0].weight == sample + 2
                    assert message.segments[0].start.x == sample
                elif name == "p0":
                    assert (
                        message.pose.position.x,
                        message.pose.position.y,
                        message.pose.position.z,
                    ) == (sample, 0, 0)
                    assert message.pose.orientation.w == 1
                else:
                    assert message.encoding == "rgb8"
                    assert (message.width, message.height, message.step) == (640, 480, 1920)
                    np.testing.assert_array_equal(
                        message.data.view(), np.full(921600, sample % 256, dtype=np.uint8)
                    )
            assert observed_samples == set(range(samples)), (name, observed_samples)
            print(
                f"MCAP {name}: verified {len(messages)} values, samples={sorted(observed_samples)}"
            )

    module = replay_module(str(artifact))(dataset=str(artifact))
    received: dict[str, list[bytes]] = {name: [] for name in expected}
    done = threading.Event()
    lock = threading.Lock()

    def receive(name: str, message: Any) -> None:
        with lock:
            received[name].append(message.encode())
            if all(len(received[key]) >= len(values) for key, values in expected.items()):
                done.set()

    with ExitStack() as stack:
        for name in expected:
            stack.callback(
                module.outputs[name].subscribe(lambda message, name=name: receive(name, message))
            )
        stack.callback(module.stop)
        module.start()
        assert done.wait(15), {name: len(values) for name, values in received.items()}
        assert received == expected
    print("Replay: every recorded CDR value matched in per-stream order after producers stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=("lcm", "zenoh"), default="zenoh")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--cpp", type=Path, default=Path("build/native-cpp-examples/cdr_relay"))
    parser.add_argument("--rust", type=Path, default=Path("target/debug/cdr_relay"))
    parser.add_argument("--recorder", type=Path, default=Path("target/debug/dimos-memory-recorder"))
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    if args.output.exists():
        parser.error("--output must be a new file; existing recordings are preserved")
    for executable in (args.cpp, args.rust, args.recorder):
        if not executable.is_file():
            parser.error(f"Build the native executable first: {executable}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    recording = RelayRecorder.blueprint(
        executable=str(args.recorder.resolve()),
        build_command="",
        cwd=str(args.recorder.resolve().parent),
        store=RustMcapStoreConfig(path=str(args.output.resolve())),
        on_existing=OnExisting.ERROR,
        record_tf=False,
        encoding_threads=2,
    )
    with ExitStack() as stack:
        stack.enter_context(
            patch(
                "dimos.protocol.service.system_configurator.base.configure_system",
                partial(configure_system, check_only=True),
            )
        )
        bp = blueprint(args.transport, args.cpp, args.rust, recording=recording)
        if args.transport == "zenoh":
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
            pool = ZenohSessionPool()
            stack.callback(pool.close_all)
            pool.acquire(
                ZenohConfig(
                    mode="router", listen=[endpoint], connect=[], multicast=False, gossip=False
                )
            )
            bp = bp.global_config(
                zenoh_mode="client", zenoh_connect=endpoint, zenoh_multicast=False
            )
        coordinator = ModuleCoordinator.build(bp)
        stack.callback(coordinator.stop)
        producer = coordinator.get_instance(PythonRelayDemo)
        for sample in range(args.samples):
            print(producer.exchange(sample), flush=True)
    verify_and_replay(args.output.resolve(), args.samples)
    print(f"Retained self-describing {args.transport} recording: {args.output}")


if __name__ == "__main__":
    main()
