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

import importlib
from pathlib import Path
import sys
from threading import Thread
from types import SimpleNamespace

import grpc
import numpy as np
import pytest

from dimos.benchmark.vlnce_r2r import protocol as host_protocol
from dimos.benchmark.vlnce_r2r.protocol import vlnce_public_v1_pb2 as pb
from dimos.benchmark.vlnce_r2r.protocol.vlnce_public_v1_pb2_grpc import (
    VlncePublicGatewayStub,
)

RUNTIME_ROOT = Path(__file__).parents[2]
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))
sys.modules["vlnce_runtime.protocol"] = host_protocol

gateway_module = importlib.import_module("vlnce_runtime.gateway")
EpisodeGateway = gateway_module.EpisodeGateway
GatewayServer = gateway_module.GatewayServer
expected_handshake = gateway_module.expected_handshake


class FakeEnvironment:
    def __init__(self) -> None:
        self.observations = {
            "rgb": np.zeros((224, 224, 3), dtype=np.uint8),
            "depth": np.ones((256, 256, 1), dtype=np.float32),
        }
        self.pose = SimpleNamespace(
            position=np.array([1.0, 2.0, 3.0]),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.controls = []
        self.submissions = 0

    def static_occupancy(self):
        return {
            "resolution": 0.05,
            "width": 3,
            "height": 2,
            "origin_x": -1.0,
            "origin_z": -2.0,
            "traversability": np.array([[0, 1, 1], [1, 1, 0]], dtype=np.uint8),
        }

    def apply_planar(self, linear_x, linear_y, angular_z, period_seconds):
        self.controls.append((linear_x, linear_y, angular_z, period_seconds))

    def submit_route(self):
        self.submissions += 1
        return {"success": 0}


def _private_case():
    return {
        "attempt_id": "attempt-1",
        "case_id": "case-1",
        "episode_id": "515",
        "protocol_revision": "vlnce-public.v1",
    }


def _messages(handshake):
    yield pb.ClientMessage(handshake=handshake)
    yield pb.ClientMessage(lifecycle=pb.LifecycleCommand(kind=pb.LifecycleCommand.BEGIN))
    yield pb.ClientMessage(
        control=pb.PlanarControl(
            command_sequence=1,
            observation_sequence=1,
            linear_x=0.2,
        )
    )
    yield pb.ClientMessage(submit_route=pb.SubmitRoute(command_sequence=2, observation_sequence=2))


def _drive_gateway(gateway):
    while not gateway.finished.is_set():
        gateway.process_pending(timeout=0.01)


def test_real_uds_gateway_publishes_public_epoch_and_submits_once(tmp_path: Path) -> None:
    environment = FakeEnvironment()
    gateway = EpisodeGateway(_private_case(), environment)
    socket_path = tmp_path / "gateway.sock"
    server = GatewayServer(socket_path, gateway)
    server.start()
    driver = Thread(target=_drive_gateway, args=(gateway,))
    driver.start()
    try:
        with grpc.insecure_channel(f"unix://{socket_path}") as channel:
            responses = list(
                VlncePublicGatewayStub(channel).Stream(_messages(gateway.session.expected))
            )
    finally:
        server.stop()
        driver.join(timeout=2)

    assert [response.WhichOneof("payload") for response in responses] == [
        "ready",
        "acknowledgement",
        "observation",
        "acknowledgement",
        "observation",
        "acknowledgement",
    ]
    initial = responses[2].observation
    assert initial.rgb_calibration.width == 224
    assert initial.depth_calibration.width == 256
    assert len(initial.rgb) == 224 * 224 * 3
    assert len(initial.depth) == 256 * 256 * 4
    assert list(initial.static_map.traversability) == [0, 1, 1, 1, 1, 0]
    assert not responses[4].observation.HasField("static_map")
    assert environment.controls == [(0.2, 0.0, 0.0, 0.1)]
    assert environment.submissions == 1
    assert gateway.terminal_reason == "submitted"
    assert gateway.native_metrics == {"success": 0}


def test_gateway_rejects_second_stream(tmp_path: Path) -> None:
    gateway = EpisodeGateway(_private_case(), FakeEnvironment())
    socket_path = tmp_path / "gateway.sock"
    server = GatewayServer(socket_path, gateway)
    server.start()
    try:
        with grpc.insecure_channel(f"unix://{socket_path}") as channel:
            stub = VlncePublicGatewayStub(channel)
            assert next(
                stub.Stream(iter([pb.ClientMessage(handshake=gateway.session.expected)]))
            ).HasField("ready")
            with pytest.raises(grpc.RpcError) as error:
                next(stub.Stream(iter([pb.ClientMessage(handshake=gateway.session.expected)])))
        assert error.value.code() == grpc.StatusCode.ALREADY_EXISTS
    finally:
        gateway.finished.set()
        server.stop()


def test_timeout_uses_official_stop_once() -> None:
    environment = FakeEnvironment()
    gateway = EpisodeGateway(_private_case(), environment)

    gateway.finish_timeout()
    gateway.finish_timeout()

    assert environment.submissions == 1
    assert gateway.terminal_reason == "timeout"
    assert gateway.native_metrics == {"success": 0}


def test_expected_handshake_contains_only_public_identity() -> None:
    private_case = _private_case()
    private_case.update({"instruction": "private", "reference_path": [[1, 2, 3]]})

    serialized = expected_handshake(private_case).SerializeToString()

    assert b"private" not in serialized
    assert b"reference_path" not in serialized
