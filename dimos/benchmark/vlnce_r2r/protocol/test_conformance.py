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

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import grpc
import pytest

from dimos.benchmark.vlnce_r2r.protocol import vlnce_public_v1_pb2 as pb
from dimos.benchmark.vlnce_r2r.protocol.session import (
    BoundedCommandQueue,
    BoundedObservationQueue,
    ConformanceGateway,
    ProtocolViolationError,
    PublicSession,
)
from dimos.benchmark.vlnce_r2r.protocol.vlnce_public_v1_pb2_grpc import (
    VlncePublicGatewayStub,
    add_VlncePublicGatewayServicer_to_server,
)


def _handshake(**updates: object) -> pb.Handshake:
    values = {
        "attempt_id": "attempt-1",
        "case_id": "case-1",
        "episode_id": "515",
        "protocol_revision": "vlnce-public-v1",
        "world_frame": "map",
        "base_frame": "base_link",
        "camera_frame": "camera_optical",
        "rgb_encoding": "rgb8",
        "depth_encoding": "float32-metre-le",
        "max_linear_x": 0.5,
        "max_linear_y": 0.5,
        "max_angular_z": 1.0,
        "control_period_seconds": 0.1,
        "capabilities": ["rgb", "depth", "static_occupancy", "planar_control"],
    }
    values.update(updates)
    return pb.Handshake(**values)


def _observation(sequence: int) -> pb.Observation:
    return pb.Observation(
        sequence=sequence,
        world_frame="map",
        base_frame="base_link",
        camera_frame="camera_optical",
        world_from_base=pb.Pose(qw=1.0),
        rgb_calibration=pb.Calibration(width=224, height=224, fx=112.0, fy=112.0),
        depth_calibration=pb.Calibration(width=256, height=256, fx=128.0, fy=128.0),
        rgb=b"rgb",
        rgb_encoding="rgb8",
        depth=b"depth",
        depth_encoding="float32-metre-le",
    )


def test_real_uds_stream_negotiates_and_submits_exactly_once(tmp_path: Path) -> None:
    socket = tmp_path / "gateway.sock"
    endpoint = f"unix:{socket}"
    executor = ThreadPoolExecutor(max_workers=2)
    server = grpc.server(executor)
    gateway = ConformanceGateway(_handshake())
    gateway.session.record_observation(1)
    add_VlncePublicGatewayServicer_to_server(gateway, server)
    assert server.add_insecure_port(endpoint) == 1
    server.start()
    try:
        with grpc.insecure_channel(endpoint) as channel:
            grpc.channel_ready_future(channel).result(timeout=2)
            replies = list(
                VlncePublicGatewayStub(channel).Stream(
                    iter(
                        [
                            pb.ClientMessage(handshake=_handshake()),
                            pb.ClientMessage(
                                lifecycle=pb.LifecycleCommand(kind=pb.LifecycleCommand.BEGIN)
                            ),
                            pb.ClientMessage(
                                control=pb.PlanarControl(
                                    command_sequence=1,
                                    observation_sequence=1,
                                    linear_x=0.25,
                                )
                            ),
                            pb.ClientMessage(
                                submit_route=pb.SubmitRoute(
                                    command_sequence=2, observation_sequence=1
                                )
                            ),
                        ]
                    )
                )
            )
    finally:
        server.stop(grace=0).wait(timeout=2)
        executor.shutdown(wait=True, cancel_futures=True)

    assert [reply.WhichOneof("payload") for reply in replies] == [
        "ready",
        "acknowledgement",
        "acknowledgement",
        "acknowledgement",
    ]
    assert replies[-1].acknowledgement.kind == pb.Acknowledgement.ROUTE_SUBMITTED
    assert gateway.session.phase == "submitted"


def test_uds_rejects_incompatible_handshake(tmp_path: Path) -> None:
    socket = tmp_path / "gateway.sock"
    endpoint = f"unix:{socket}"
    executor = ThreadPoolExecutor(max_workers=2)
    server = grpc.server(executor)
    add_VlncePublicGatewayServicer_to_server(ConformanceGateway(_handshake()), server)
    assert server.add_insecure_port(endpoint) == 1
    server.start()
    try:
        with grpc.insecure_channel(endpoint) as channel:
            grpc.channel_ready_future(channel).result(timeout=2)
            with pytest.raises(grpc.RpcError) as caught:
                list(
                    VlncePublicGatewayStub(channel).Stream(
                        iter([pb.ClientMessage(handshake=_handshake(episode_id="foreign"))])
                    )
                )
    finally:
        server.stop(grace=0).wait(timeout=2)
        executor.shutdown(wait=True, cancel_futures=True)

    assert caught.value.code() == grpc.StatusCode.FAILED_PRECONDITION


def test_uds_rejects_reconnect_after_session_identity_is_bound(tmp_path: Path) -> None:
    socket = tmp_path / "gateway.sock"
    endpoint = f"unix:{socket}"
    executor = ThreadPoolExecutor(max_workers=2)
    server = grpc.server(executor)
    gateway = ConformanceGateway(_handshake())
    add_VlncePublicGatewayServicer_to_server(gateway, server)
    assert server.add_insecure_port(endpoint) == 1
    server.start()
    try:
        with grpc.insecure_channel(endpoint) as channel:
            grpc.channel_ready_future(channel).result(timeout=2)
            first = list(
                VlncePublicGatewayStub(channel).Stream(
                    iter([pb.ClientMessage(handshake=_handshake())])
                )
            )
            with pytest.raises(grpc.RpcError) as caught:
                list(
                    VlncePublicGatewayStub(channel).Stream(
                        iter([pb.ClientMessage(handshake=_handshake())])
                    )
                )
    finally:
        server.stop(grace=0).wait(timeout=2)
        executor.shutdown(wait=True, cancel_futures=True)

    assert first[0].WhichOneof("payload") == "ready"
    assert caught.value.code() == grpc.StatusCode.FAILED_PRECONDITION


@pytest.mark.parametrize(
    ("control", "match"),
    [
        (pb.PlanarControl(command_sequence=2, observation_sequence=3), "increase by one"),
        (pb.PlanarControl(command_sequence=1, observation_sequence=2), "latest observation"),
        (
            pb.PlanarControl(command_sequence=1, observation_sequence=3, linear_x=0.6),
            "exceeds negotiated limits",
        ),
    ],
)
def test_session_rejects_bad_sequence_stale_and_unbounded_control(
    control: pb.PlanarControl, match: str
) -> None:
    session = PublicSession(_handshake())
    session.accept(pb.ClientMessage(handshake=_handshake()))
    session.accept(pb.ClientMessage(lifecycle=pb.LifecycleCommand(kind=pb.LifecycleCommand.BEGIN)))
    session.record_observation(3)

    with pytest.raises(ProtocolViolationError, match=match):
        session.accept(pb.ClientMessage(control=control))


def test_session_rejects_duplicate_submission_and_post_terminal_motion() -> None:
    session = PublicSession(_handshake())
    session.accept(pb.ClientMessage(handshake=_handshake()))
    session.accept(pb.ClientMessage(lifecycle=pb.LifecycleCommand(kind=pb.LifecycleCommand.BEGIN)))
    session.record_observation(1)
    session.accept(
        pb.ClientMessage(submit_route=pb.SubmitRoute(command_sequence=1, observation_sequence=1))
    )

    with pytest.raises(ProtocolViolationError, match="requires a running episode"):
        session.accept(
            pb.ClientMessage(
                submit_route=pb.SubmitRoute(command_sequence=2, observation_sequence=1)
            )
        )
    with pytest.raises(ProtocolViolationError, match="requires a running episode"):
        session.accept(
            pb.ClientMessage(control=pb.PlanarControl(command_sequence=2, observation_sequence=1))
        )


def test_public_descriptor_contains_no_private_benchmark_fields() -> None:
    serialized = pb.DESCRIPTOR.serialized_pb.decode("latin1").lower()
    forbidden = {
        "goal_position",
        "reference_path",
        "semantic_label",
        "navigation_error",
        "oracle_success",
        "success",
        "spl",
        "ndtw",
        "progress",
        "score",
    }

    assert not {field for field in forbidden if field in serialized}


def test_observation_queue_publishes_latest_complete_epoch_with_drop_count() -> None:
    queue = BoundedObservationQueue(_handshake(), capacity=2)
    queue.publish(_observation(1))
    queue.publish(_observation(2))
    queue.publish(_observation(3))

    latest = queue.take_latest()

    assert latest is not None
    assert latest.sequence == 3
    assert latest.dropped_observations == 2


def test_observation_queue_rejects_incomplete_and_duplicate_epochs() -> None:
    queue = BoundedObservationQueue(_handshake())
    queue.publish(_observation(1))

    with pytest.raises(ProtocolViolationError, match="increase monotonically"):
        queue.publish(_observation(1))
    with pytest.raises(ProtocolViolationError, match="missing RGB or depth"):
        queue.publish(pb.Observation(sequence=2))


def test_command_queue_rejects_loss_under_backpressure() -> None:
    queue = BoundedCommandQueue(capacity=1)
    command = pb.ClientMessage(control=pb.PlanarControl(command_sequence=1))
    queue.put(command)

    with pytest.raises(ProtocolViolationError, match="queue is full"):
        queue.put(command)

    assert queue.take() == command
