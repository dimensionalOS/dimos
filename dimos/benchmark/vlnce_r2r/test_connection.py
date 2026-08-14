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

from collections.abc import Iterator

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
from dimos.benchmark.vlnce_r2r.protocol.contract import (
    CONTROL_PERIOD_SECONDS,
    MAX_ANGULAR_Z,
)


@pytest.fixture
def connection() -> Iterator[VlnceConnection]:
    connection = VlnceConnection(socket_path="/tmp/unused-vlnce.sock")
    yield connection
    connection.stop()


def test_observation_decodes_to_public_sensor_geometry() -> None:
    calibration = pb.Calibration(width=2, height=1, fx=1, fy=1, cx=0.5, cy=0)
    observation = pb.Observation(
        sequence=1,
        monotonic_time_ns=2_000_000_000,
        world_from_base=pb.Pose(x=1, y=2, z=3, qw=1),
        base_from_camera=pb.Pose(y=1.25, qx=1),
        rgb_calibration=calibration,
        rgb=np.arange(6, dtype=np.uint8).tobytes(),
        depth_calibration=calibration,
        depth=np.array([[1.0, 2.0]], dtype="<f4").tobytes(),
        static_map=pb.OccupancyMap(
            resolution=0.5,
            width=2,
            height=1,
            origin=pb.Pose(x=-1, z=-2, qw=1),
            traversability=bytes([0, 1]),
        ),
    )

    decoded = decode_observation(observation, wall_minus_monotonic=100.0)

    assert decoded["color_image"].data.shape == (1, 2, 3)
    assert decoded["depth_image"].data.tolist() == [[1.0, 2.0]]
    assert decoded["pointcloud"].points_f32().shape == (2, 3)
    assert decoded["odom"].position.to_list() == [-3.0, -1.0, 2.0]
    assert decoded["global_costmap"].grid.shape == (2, 1)


def test_protocol_payload_validation_rejects_bad_images_and_map() -> None:
    calibration = pb.Calibration(width=1, height=1, fx=1, fy=1)
    with pytest.raises(VlnceConnectionError, match="image payload"):
        decode_observation(
            pb.Observation(
                sequence=1,
                world_from_base=pb.Pose(qw=1),
                base_from_camera=pb.Pose(qx=1),
                rgb_calibration=calibration,
                depth_calibration=calibration,
            ),
            0.0,
        )
    with pytest.raises(VlnceConnectionError, match="non-geometric"):
        _decode_occupancy(
            pb.OccupancyMap(width=1, height=1, resolution=1, traversability=bytes([2])),
            0.0,
        )


def test_habitat_coordinates_are_converted_to_dimos_world() -> None:
    position, orientation = _habitat_pose_to_dimos(pb.Pose(x=1, y=2, z=3, qw=1))
    assert position.tolist() == [-3.0, -1.0, 2.0]
    assert np.isclose(np.linalg.norm(orientation), 1.0)


def test_turn_sends_bounded_controls_for_the_requested_angle(connection, mocker) -> None:
    move = mocker.patch.object(connection, "move", return_value=True)

    result = connection.turn(90.0)

    commands = [call.args[0] for call in move.call_args_list]
    assert result == "Turned counterclockwise 90.0 degrees."
    assert sum(command.angular.z * CONTROL_PERIOD_SECONDS for command in commands) == pytest.approx(
        np.pi / 2
    )
    assert all(command.linear.is_zero() for command in commands)
    assert all(0.0 < command.angular.z <= MAX_ANGULAR_Z for command in commands)
