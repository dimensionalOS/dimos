# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

import numpy as np
import pytest

from dimos.benchmark.vlnce_r2r.connection import (
    VlnceConnectionError,
    _decode_occupancy,
    _habitat_pose_to_dimos,
    decode_observation,
)
from dimos.benchmark.vlnce_r2r.protocol import vlnce_public_v1_pb2 as pb


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
