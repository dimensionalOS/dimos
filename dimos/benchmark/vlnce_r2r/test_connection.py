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

from concurrent import futures
from threading import Event

import grpc
import numpy as np
import pytest

from dimos.benchmark.vlnce_r2r.connection import (
    VlnceConnection,
    VlnceConnectionError,
    _decode_occupancy,
    _habitat_pose_to_dimos,
    decode_observation,
)
from dimos.benchmark.vlnce_r2r.protocol import vlnce_public_v1_pb2 as pb
from dimos.benchmark.vlnce_r2r.protocol.vlnce_public_v1_pb2_grpc import (
    VlncePublicGatewayServicer,
    add_VlncePublicGatewayServicer_to_server,
)
from dimos.msgs.geometry_msgs.Twist import Twist


class _InputTransport:
    def __init__(self) -> None:
        self.callback = None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def subscribe(self, callback, _stream=None):
        self.callback = callback
        return lambda: None


class _PublicRuntime(VlncePublicGatewayServicer):
    def __init__(self, handshake: pb.Handshake) -> None:
        self.handshake = handshake
        self.controls = []
        self.submissions = 0

    def Stream(self, request_iterator, _context):
        for request in request_iterator:
            payload = request.WhichOneof("payload")
            if payload == "handshake":
                assert request.handshake == self.handshake
                yield pb.ServerMessage(ready=pb.Ready(negotiated=self.handshake))
            elif payload == "lifecycle" and request.lifecycle.kind == pb.LifecycleCommand.BEGIN:
                yield pb.ServerMessage(
                    acknowledgement=pb.Acknowledgement(kind=pb.Acknowledgement.CONTROL_ACCEPTED)
                )
                yield pb.ServerMessage(observation=_observation(sequence=1))
            elif payload == "control":
                self.controls.append(request.control)
                yield pb.ServerMessage(
                    acknowledgement=pb.Acknowledgement(
                        command_sequence=request.control.command_sequence,
                        kind=pb.Acknowledgement.CONTROL_ACCEPTED,
                    )
                )
                yield pb.ServerMessage(observation=_observation(sequence=2, x=1.1))
            elif payload == "submit_route":
                self.submissions += 1
                yield pb.ServerMessage(
                    acknowledgement=pb.Acknowledgement(
                        command_sequence=request.submit_route.command_sequence,
                        kind=pb.Acknowledgement.ROUTE_SUBMITTED,
                    )
                )
                return


def _calibration(width: int, height: int) -> pb.Calibration:
    return pb.Calibration(width=width, height=height, fx=1.0, fy=1.0, cx=0.0, cy=0.0)


def _observation(sequence: int = 1, x: float = 1.0) -> pb.Observation:
    rgb = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    depth = np.array([[1.0, 2.0], [3.0, np.nan]], dtype="<f4")
    return pb.Observation(
        sequence=sequence,
        monotonic_time_ns=2_000_000_000,
        world_frame="habitat_world",
        base_frame="habitat_base",
        camera_frame="camera_optical",
        world_from_base=pb.Pose(x=x, y=2.0, z=3.0, qw=1.0),
        base_from_camera=pb.Pose(y=1.25, qx=1.0),
        rgb_calibration=_calibration(2, 2),
        rgb=rgb.tobytes(),
        rgb_encoding="rgb8",
        depth_calibration=_calibration(2, 2),
        depth=depth.tobytes(),
        depth_encoding="32FC1_LE",
        static_map=pb.OccupancyMap(
            frame_id="habitat_world",
            resolution=0.5,
            width=3,
            height=2,
            origin=pb.Pose(x=-2.0, z=-1.0, qw=1.0),
            traversability=bytes([0, 1, 1, 1, 0, 1]),
            encoding="uint8_traversable",
        ),
    )


def test_decode_observation_produces_coherent_native_geometry() -> None:
    native = decode_observation(_observation(), wall_minus_monotonic=100.0)

    assert native["color_image"].data.shape == (2, 2, 3)
    assert native["depth_image"].data.shape == (2, 2)
    assert native["camera_info"].width == 2
    assert native["depth_camera_info"].width == 2
    assert native["color_image"].ts == 102.0
    assert native["odom"].position.as_tuple == (-3.0, -1.0, 2.0)
    assert native["odom"].orientation.to_tuple() == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert native["odometry"].child_frame_id == "base_link"
    assert [transform.child_frame_id for transform in native["tf"]] == [
        "base_link",
        "camera_optical",
    ]
    assert native["pointcloud"].as_numpy()[0] == pytest.approx(
        np.array([[0.0, 0.0, 1.0], [2.0, 0.0, 2.0], [0.0, 3.0, 3.0]])
    )
    assert native["global_costmap"].grid.tolist() == [[0, 0], [100, 0], [0, 100]]
    assert native["global_costmap"].info.origin.position.as_tuple == (-0.0, 0.5, 0.0)


def test_habitat_basis_conversion_preserves_right_handed_identity() -> None:
    position, orientation = _habitat_pose_to_dimos(pb.Pose(x=4, y=5, z=6, qw=1))

    assert position == pytest.approx([-6, -4, 5])
    assert orientation == pytest.approx([0, 0, 0, 1])


def test_public_map_rejects_non_geometric_cell_values() -> None:
    occupancy = _observation().static_map
    occupancy.traversability = bytes([0, 1, 1, 1, 2, 1])

    with pytest.raises(VlnceConnectionError, match="non-geometric"):
        _decode_occupancy(occupancy, 1.0)


def test_submit_route_is_a_benchmark_scoped_skill_without_arguments() -> None:
    assert getattr(VlnceConnection.submit_route, "__skill__", False)
    assert "VLN-CE STOP" in VlnceConnection.submit_route.__doc__


def test_connection_runs_begin_control_and_submission_over_real_uds(tmp_path) -> None:
    socket_path = tmp_path / "public.sock"
    connection = VlnceConnection(
        socket_path=str(socket_path),
        attempt_id="attempt-1",
        case_id="case-1",
        episode_id="515",
        connect_timeout_seconds=2.0,
    )
    connection.cmd_vel.transport = _InputTransport()
    runtime = _PublicRuntime(connection._expected)
    executor = futures.ThreadPoolExecutor(max_workers=2)
    server = grpc.server(executor)
    add_VlncePublicGatewayServicer_to_server(runtime, server)
    assert server.add_insecure_port(f"unix://{socket_path}") == 1
    server.start()
    poses = []
    second_observation = Event()
    connection.odom.subscribe(poses.append)
    connection.odom.subscribe(lambda _pose: second_observation.set() if len(poses) == 2 else None)
    try:
        connection.start()
        assert connection.wait_ready()
        assert connection.begin()
        assert connection.move(Twist(linear=[0.2, 0.0, 0.0], angular=[0.0, 0.0, 0.1]))
        assert second_observation.wait(timeout=2)
        assert connection.submit_route() == "Route submitted; the VLN-CE evaluation is ending."
        diagnostics = connection.public_diagnostics()
    finally:
        connection.stop()
        server.stop(grace=0).wait(timeout=2)
        executor.shutdown(wait=True)

    assert [pose.position.as_tuple for pose in poses] == [
        (-3.0, -1.0, 2.0),
        (-3.0, -1.1, 2.0),
    ]
    assert len(runtime.controls) == 1
    assert runtime.controls[0].command_sequence == 1
    assert runtime.controls[0].observation_sequence == 1
    assert runtime.submissions == 1
    assert diagnostics["observation_count"] == 2
    assert diagnostics["accepted_control_count"] == 1
    assert diagnostics["last_control"] == {
        "linear_x": 0.2,
        "linear_y": 0.0,
        "angular_z": 0.1,
    }
    assert diagnostics["route_submitted"] is True
