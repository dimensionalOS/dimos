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

"""Publish generated messages through the real cockpit bridge and local relay.

Keep publishing for the browser page until Ctrl-C.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import threading
from typing import Any

import numpy as np

from dimos.core.transport import ZenohTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from dimos.messages.sensor_msgs.msg import Image, Imu, JointState
from dimos.messages.std_msgs.msg import Empty, Header, Int64, String
from dimos.protocol.pubsub.impl.zenohpubsub import Topic
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.web.cockpit import Channel, cockpit
from dimos.web.relay_bridge.config import RelayBridgeConfig


def build_messages() -> list[tuple[str, Any]]:
    header = Header(stamp=Time(sec=1757400000, nanosec=500000123), frame_id="map")
    return [
        (
            "pose_stamped",
            PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=1.5, y=-2.5, z=0.25),
                    orientation=Quaternion(z=0.6, w=0.8, x=0.0, y=0.0),
                ),
            ),
        ),
        (
            "custom_segments",
            LineSegments3D(
                header=header,
                segments=[
                    LineSegment3D(
                        start=Point(x=1, y=0.0, z=0.0), end=Point(y=2, x=0.0, z=0.0), weight=4
                    )
                ],
            ),
        ),
        (
            "image",
            Image(
                header=header,
                width=2,
                height=1,
                encoding="rgb8",
                step=6,
                data=np.array([0, 1, 2, 253, 254, 255], dtype=np.uint8),
                is_bigendian=0,
            ),
        ),
        (
            "imu",
            Imu(
                header=header,
                orientation_covariance=np.asarray(range(9), dtype=np.float64),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                angular_velocity=Vector3(x=0.0, y=0.0, z=0.0),
                angular_velocity_covariance=np.zeros(9, dtype=np.float64),
                linear_acceleration=Vector3(x=0.0, y=0.0, z=0.0),
                linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
            ),
        ),
        (
            "joints",
            JointState(
                header=header,
                name=["joint_a", "joint_b"],
                position=np.array([1.5, -2.5], dtype=np.float64),
                velocity=np.array([], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            ),
        ),
        ("int64", Int64(data=-9007199254740993)),
        ("empty", Empty()),
        ("string", String(data="robot 🦾 café")),
    ]


class LoopbackRPC(ZenohRPC):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            mode="peer",
            listen=["tcp/127.0.0.1:0"],
            scouting=False,
            multicast=False,
            gossip=False,
            **kwargs,
        )


def run(port: int) -> None:
    messages = dict(build_messages())
    channels = [
        Channel(name, type(value), encoding=value.__msgtype__ + ".cdr.v1", max_hz=5)
        for name, value in messages.items()
    ]
    (atom,) = cockpit(channels=channels).blueprints
    config = dict(
        mode="peer", listen=["tcp/127.0.0.1:0"], scouting=False, multicast=False, gossip=False
    )
    stopped = threading.Event()
    with ExitStack() as cleanup:
        module = atom.module(
            **atom.kwargs,
            robot_id="cdr-demo",
            robot_name="CDR demo",
            rpc_transport=LoopbackRPC,
            local_port=port,
            open_browser=False,
            rtc=False,
        )
        cleanup.callback(module.stop)
        publishers = {}
        for name, value in messages.items():
            receiver = ZenohTransport(Topic(name, type(value)), **config)
            cleanup.callback(receiver.stop)
            module.inputs[name].transport = receiver
            publishers[name] = ZenohTransport(Topic(name, type(value)), **config)
            cleanup.callback(publishers[name].stop)
        module.start()

        def publish() -> None:
            while not stopped.is_set():
                for name, value in messages.items():
                    publishers[name].broadcast(None, value)
                stopped.wait(0.2)

        thread = threading.Thread(target=publish, name="cdr-demo-publisher", daemon=True)
        thread.start()

        def stop_publisher() -> None:
            stopped.set()
            thread.join(timeout=5)

        cleanup.callback(stop_publisher)
        print("From web/sdk run: deno task fixture --host 127.0.0.1", flush=True)
        print(
            f"Open http://127.0.0.1:5174/demo_cdr.html?live=1&relay={module._url}",
            flush=True,
        )
        try:
            stopped.wait()
        except KeyboardInterrupt:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", type=int, default=RelayBridgeConfig.model_fields["local_port"].default
    )
    args = parser.parse_args()
    run(args.port)


if __name__ == "__main__":
    main()
