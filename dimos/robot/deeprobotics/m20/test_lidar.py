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

"""M20Lidar and the M20 lidar blueprint: two namespaced lidars on one lidar stream, told apart by frame, each mounting
itself on base_link through the one global tf."""

import math

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.core.coordination.module_coordinator import stream_name_types
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.deeprobotics.m20.constants import FRONT_LIDAR_XYZ_M, REAR_LIDAR_XYZ_M
from dimos.robot.deeprobotics.m20.lidar import M20Lidar, M20LidarConfig, m20_lidars
from dimos.robot.deeprobotics.m20.model import M20_MODEL, STAND_JOINT_POSITIONS

FRONT, REAR = "front_lidar/m20lidar", "rear_lidar/m20lidar"


def _config(**fields: object) -> M20LidarConfig:
    return M20LidarConfig(
        **{"frame_id": "f", "imu_frame_id": "f_imu", "mount_parent_frame_id": "body", **fields}
    )


def _wire(name: str, **overrides: object) -> dict:
    atom = {atom.name: atom for atom in m20_lidars(**overrides).solidify().blueprints}[name]
    kwargs = {key: value for key, value in atom.kwargs.items() if key != "instance_name"}
    return M20LidarConfig(**kwargs).to_config_dict()


def test_which_lidar_picks_its_network_and_mount() -> None:
    front, rear = _config(lidar="front"), _config(lidar="rear")
    assert (front.group, front.msop_port, front.difop_port, front.imu_port) == (
        "224.10.10.201",
        6691,
        7781,
        6681,
    )
    assert (rear.group, rear.msop_port, rear.difop_port, rear.imu_port) == (
        "224.10.10.202",
        6692,
        7782,
        6682,
    )
    assert (front.mount_xyz, rear.mount_xyz) == (FRONT_LIDAR_XYZ_M, REAR_LIDAR_XYZ_M)
    # anything set explicitly wins over the chosen lidar's
    assert _config(lidar="front", msop_port=7000).msop_port == 7000


def test_frames_voxel_and_min_distance_are_config() -> None:
    config = _config(lidar="rear", voxel=0.08)
    assert (config.frame_id, config.imu_frame_id, config.mount_parent_frame_id) == (
        "f",
        "f_imu",
        "body",
    )
    assert (
        config.voxel,
        config.min_distance,
        _config(lidar="rear", min_distance=0.3).min_distance,
    ) == (
        0.08,
        0.1,
        0.3,
    )
    for missing in ("frame_id", "imu_frame_id", "mount_parent_frame_id"):
        fields = {
            "lidar": "front",
            "frame_id": "f",
            "imu_frame_id": "f",
            "mount_parent_frame_id": "b",
        }
        del fields[missing]
        with pytest.raises(ValidationError):
            M20LidarConfig(**fields)
    # the driver gets the Airy's fields only
    assert "lidar" not in _config(lidar="front").to_config_dict()


def test_both_namespaced_lidars_land_on_lidar_and_the_global_tf() -> None:
    blueprint = m20_lidars().solidify()
    assert {atom.name for atom in blueprint.blueprints} == {FRONT, REAR}
    assert {atom.module for atom in blueprint.blueprints} == {M20Lidar}
    for name in (FRONT, REAR):
        assert blueprint.remapping_map[(name, "lidar")] == "lidar"
        assert blueprint.remapping_map[(name, "imu")] == "lidar_imu"
        # tf is exposed, never namespaced
        assert blueprint.remapping_map.get((name, "tf"), "tf") == "tf"
        # the body filter's joint state is exposed too
        assert blueprint.remapping_map[(name, "joint_state")] == "coordinator_joint_state"
    assert stream_name_types(blueprint) >= {("tf", TFMessage)}
    assert {name for name, _ in stream_name_types(blueprint)} == {
        "lidar",
        "lidar_imu",
        "tf",
        "coordinator_joint_state",
    }


def test_the_body_is_cut_in_the_driver_at_the_standing_pose_by_default() -> None:
    for name in (FRONT, REAR):
        wire = _wire(name)
        assert wire["body_filter"] is True
        assert wire["body_filter_joint_source"] == "fixed"
        assert wire["body_filter_joint_positions"] == STAND_JOINT_POSITIONS
        assert wire["body_filter_urdf"] == M20_MODEL.load().xml
        # the URDF's root is the frame the lidar is mounted in, which places the body around the lidar
        assert wire["mount_parent_frame_id"] == M20_MODEL.load().root_link
        live = _wire(name, body_filter_joint_source="joint_state")
        assert live["body_filter_joint_source"] == "joint_state"
    off = _wire(FRONT, body_filter=False)
    assert off["body_filter"] is False
    assert off["body_filter_urdf"] is None


def test_frames_come_from_config_not_the_namespace() -> None:
    for name, frame in ((FRONT, "front_lidar"), (REAR, "rear_lidar")):
        wire = _wire(name, voxel=0.1)
        assert (wire["frame_id"], wire["imu_frame_id"], wire["mount_parent_frame_id"]) == (
            frame,
            frame,
            "base_link",
        )
        assert wire["voxel"] == 0.1
        assert "frame_id_prefix" not in wire


def _matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    q = Quaternion.from_euler(Vector3(*rpy))
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def test_mounts() -> None:
    for name, xyz, outward in [(FRONT, FRONT_LIDAR_XYZ_M, 1.0), (REAR, REAR_LIDAR_XYZ_M, -1.0)]:
        wire = _wire(name)
        assert tuple(wire["mount_xyz"]) == xyz
        rotation = _matrix(tuple(wire["mount_rpy"]))
        # lidar x is up (the IMU's gravity reads +x), its spin axis z looks out its end of the body
        assert np.allclose(rotation @ [1, 0, 0], [0, 0, 1], atol=1e-9)
        assert np.allclose(rotation @ [0, 0, 1], [outward, 0, 0], atol=1e-9)
        assert math.isclose(np.linalg.det(rotation), 1.0)


def test_a_resolved_lidar_config_needs_no_mujoco(monkeypatch: pytest.MonkeyPatch) -> None:
    """The board running the lidars has no mujoco: a config the blueprint resolved must not load the model again."""
    import sys

    atom = next(iter(m20_lidars().solidify().blueprints))
    kwargs = {key: value for key, value in atom.kwargs.items() if key != "instance_name"}
    monkeypatch.setitem(sys.modules, "mujoco", None)
    monkeypatch.setattr("dimos.robot.deeprobotics.m20.lidar._m20_body", _no_model)
    M20LidarConfig(**kwargs)


def _no_model() -> None:
    raise AssertionError("the lidar config loaded the robot model")
