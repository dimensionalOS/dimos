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

"""The M20's two Airys in ``m20``: one lidar stream told apart by frame, each at its network and mount, the body cut in
the driver, each mounting itself on base_link through the one global tf."""

import math

import numpy as np

from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.lidar.robosense.module import Airy, AiryConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.deeprobotics.m20.blueprints.m20 import m20
from dimos.robot.deeprobotics.m20.constants import FRONT_LIDAR_XYZ_M, REAR_LIDAR_XYZ_M
from dimos.robot.deeprobotics.m20.model import M20_MODEL, STAND_JOINT_POSITIONS

FRONT, REAR = "front_lidar/airy", "rear_lidar/airy"


def _real(**choices: str) -> Blueprint:
    return m20.solidify(choices={"M20Mode": "real", **choices})


def _wire(name: str, **choices: str) -> dict:
    atom = {atom.name: atom for atom in _real(**choices).active_blueprints}[name]
    return AiryConfig(
        **{key: value for key, value in atom.kwargs.items() if key != "instance_name"}
    ).to_config_dict()


def test_each_lidar_has_its_network_and_the_vendor_drivers_range_and_clock() -> None:
    front, rear = _wire(FRONT), _wire(REAR)
    assert (
        front["group"],
        front["lidar_ip"],
        front["msop_port"],
        front["difop_port"],
        front["imu_port"],
    ) == (
        "224.10.10.201",
        "10.21.33.201",
        6691,
        7781,
        6681,
    )
    assert (
        rear["group"],
        rear["lidar_ip"],
        rear["msop_port"],
        rear["difop_port"],
        rear["imu_port"],
    ) == (
        "224.10.10.202",
        "10.21.33.202",
        6692,
        7782,
        6682,
    )
    for wire in (front, rear):
        assert (wire["min_distance"], wire["max_distance"], wire["use_lidar_clock"]) == (
            0.1,
            60.0,
            True,
        )


def test_both_namespaced_lidars_land_on_lidar_and_the_global_tf() -> None:
    blueprint = _real()
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    for name in (FRONT, REAR):
        assert atoms[name].module is Airy
        assert blueprint.remapping_map[(name, "lidar")] == "lidar"
        assert blueprint.remapping_map[(name, "imu")] == "lidar_imu"
        # tf is exposed, never namespaced
        assert blueprint.remapping_map.get((name, "tf"), "tf") == "tf"
        # the body filter's joint state is exposed too
        assert blueprint.remapping_map[(name, "joint_state")] == "coordinator_joint_state"


def test_the_body_is_cut_in_the_driver_at_the_standing_pose_or_the_controllers_joint_state() -> (
    None
):
    for name in (FRONT, REAR):
        wire = _wire(name)
        assert wire["body_filter"] is True
        assert wire["body_filter_joint_source"] == "fixed"
        assert wire["body_filter_joint_positions"] == STAND_JOINT_POSITIONS
        assert wire["body_filter_urdf"] == M20_MODEL.load().xml
        # the URDF's root is the frame the lidar is mounted in, which places the body around the lidar
        assert wire["mount_parent_frame_id"] == M20_MODEL.load().root_link
        assert _wire(name, M20Joints="controller")["body_filter_joint_source"] == "joint_state"
        # the Python filter cuts the raw scans instead
        assert _wire(name, M20BodyFilter="python")["body_filter"] is False


def test_frames_come_from_the_blueprint_not_the_namespace() -> None:
    for name, frame in ((FRONT, "front_lidar"), (REAR, "rear_lidar")):
        wire = _wire(name)
        assert (wire["frame_id"], wire["imu_frame_id"], wire["mount_parent_frame_id"]) == (
            frame,
            frame,
            "base_link",
        )


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
