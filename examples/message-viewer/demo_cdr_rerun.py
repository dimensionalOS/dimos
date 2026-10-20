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

"""Record a generated PoseStamped over loopback CDR into a real Rerun recording."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from pathlib import Path
import threading
from typing import Any

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header
from dimos.visualization.rerun.cdr import generated_archetypes


def run(output: Path) -> None:
    # Reserve a new destination exclusively; never replace a user's recording.
    with output.open("xb"):
        pass
    message = PoseStamped(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map"),
        pose=Pose(
            position=Point(x=1.5, y=-2.5, z=0.25), orientation=Quaternion(x=0, y=0, z=0.6, w=0.8)
        ),
    )
    config: dict[str, Any] = dict(
        mode="peer", listen=["tcp/127.0.0.1:0"], scouting=False, multicast=False, gossip=False
    )
    publisher = CdrTransport("pose", PoseStamped, backend="zenoh", **config)
    subscriber = CdrTransport("pose", PoseStamped, backend="zenoh", **config)
    recording = rr.RecordingStream("dimos-cdr-pose")
    received = threading.Event()
    errors: list[Exception] = []

    def receive(value: PoseStamped) -> None:
        try:
            stamp = value.header.stamp
            recording.set_time(
                "source", timestamp=np.datetime64(stamp.sec * 1_000_000_000 + stamp.nanosec, "ns")
            )
            entries = generated_archetypes(value, "world/pose")
            assert entries is not None
            for entry in entries:
                recording.log(entry[0], entry[1], strict=True)
        except Exception as error:
            errors.append(error)
        finally:
            received.set()

    with ExitStack() as cleanup:
        cleanup.callback(recording.disconnect)
        recording.save(
            output,
            default_blueprint=rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(origin="world", name="Pose axes"),
                    rrb.TextDocumentView(origin="world/pose/message", name="Exact message values"),
                )
            ),
        )
        # This demo explicitly identifies map with its viewer origin. Runtime
        # bridges use the producer's named TF frame; no coordinates are rewritten.
        recording.log("world", rr.Transform3D(child_frame="tf#/map"), static=True)
        cleanup.callback(subscriber.stop)
        cleanup.callback(subscriber.subscribe(receive))
        cleanup.callback(publisher.stop)
        publisher.broadcast(None, message)
        if not received.wait(timeout=10):
            raise TimeoutError("No loopback CDR PoseStamped received")
        if errors:
            raise errors[0]
        recording.flush(timeout_sec=10)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new .rrd file (must not exist)")
    args = parser.parse_args()
    run(args.output)
    print(f"Recorded actual loopback CDR; open with: rerun {args.output}")


if __name__ == "__main__":
    main()
