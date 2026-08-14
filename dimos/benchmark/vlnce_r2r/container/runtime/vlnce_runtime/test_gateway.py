# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

import importlib
from pathlib import Path
import sys
from threading import Thread
from types import SimpleNamespace

import grpc
import numpy as np

from dimos.benchmark.vlnce_r2r import protocol
from dimos.benchmark.vlnce_r2r.protocol import vlnce_public_v1_pb2 as pb
from dimos.benchmark.vlnce_r2r.protocol.vlnce_public_v1_pb2_grpc import (
    VlncePublicGatewayStub,
)

sys.modules[__package__ + ".protocol"] = protocol
gateway_module = importlib.import_module(__package__ + ".gateway")
EpisodeGateway = gateway_module.EpisodeGateway
GatewayServer = gateway_module.GatewayServer


class FakeEnvironment:
    def __init__(self) -> None:
        self.observations = {
            "rgb": np.zeros((2, 3, 3), dtype=np.uint8),
            "depth": np.ones((2, 3), dtype=np.float32),
        }
        self.pose = SimpleNamespace(
            position=np.array([1.0, 2.0, 3.0]),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.controls: list[tuple[float, ...]] = []
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

    def apply_planar(self, *control):
        self.controls.append(control)

    def submit_route(self):
        self.submissions += 1
        return {"success": 0.0}


def test_real_uds_episode_is_ordered_and_map_is_sent_once(tmp_path: Path) -> None:
    environment = FakeEnvironment()
    gateway = EpisodeGateway({}, environment)
    server = GatewayServer(tmp_path / "gateway.sock", gateway)
    server.start()
    driver = Thread(target=_drive, args=(gateway,))
    driver.start()
    try:
        with grpc.insecure_channel(f"unix://{server.socket_path}") as channel:
            stub = VlncePublicGatewayStub(channel)
            first = stub.Start(pb.StartRequest())
            second = stub.StepPlanar(
                pb.PlanarControl(observation_sequence=first.sequence, linear_x=0.2)
            )
            stub.SubmitRoute(pb.SubmitRouteRequest(observation_sequence=second.sequence))
    finally:
        server.stop()
        driver.join(timeout=2)

    assert (first.sequence, second.sequence) == (1, 2)
    assert first.HasField("static_map") and not second.HasField("static_map")
    assert environment.controls == [(0.2, 0.0, 0.0, 0.1)]
    assert environment.submissions == 1
    assert gateway.terminal_reason == "submitted"


def test_timeout_submits_official_stop_once() -> None:
    environment = FakeEnvironment()
    gateway = EpisodeGateway({}, environment)
    gateway.finish_timeout()
    gateway.finish_timeout()
    assert environment.submissions == 1
    assert gateway.terminal_reason == "timeout"


def _drive(gateway: EpisodeGateway) -> None:
    while not gateway.finished.is_set():
        gateway.process_pending(timeout=0.01)
