# Copyright 2025-2026 Dimensional Inc.
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

"""The R1 hung off Point-LIO: the mount tree, the pose projection, the network."""

import json
from pathlib import Path

import pytest

from dimos.hardware.sensors.lidar.pointlio.module import PointLioRust
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_pointlio import r1pro_pointlio
from dimos.robot.galaxea.r1pro.connection import LIDAR_MOUNT_XYZ, R1ProConnection
from dimos.robot.galaxea.r1pro.lio import (
    BASE_FRAME,
    CHASSIS_LIDAR_FRAME,
    LIDAR_FRAME,
    R1ProLioMountTf,
    R1ProLioOdomPose,
    R1ProMid360,
    mount_transforms,
)
from dimos.robot.galaxea.r1pro.vendor_lidar import ENV_CONFIG_PATH


@pytest.fixture()
def built():
    """Modules built directly, stopped afterwards so their loop thread exits."""
    modules = []

    def build(module_type, **kwargs):
        module = module_type(**kwargs)
        modules.append(module)
        return module

    yield build
    for module in modules:
        module.stop()


def test_mount_tree_is_rooted_at_pointlio_and_keeps_the_chassis_lidar_edge() -> None:
    lidar_to_base, base_to_chassis = mount_transforms()

    # Point-LIO owns base_link: the URDF's base -> lidar mount, inverted.
    assert (lidar_to_base.frame_id, lidar_to_base.child_frame_id) == (LIDAR_FRAME, BASE_FRAME)
    assert lidar_to_base.translation.x == pytest.approx(-LIDAR_MOUNT_XYZ[0])
    assert lidar_to_base.translation.y == pytest.approx(-LIDAR_MOUNT_XYZ[1])
    assert lidar_to_base.translation.z == pytest.approx(-LIDAR_MOUNT_XYZ[2])

    # The edge publish_odom=False takes away from the connection, put back.
    assert (base_to_chassis.frame_id, base_to_chassis.child_frame_id) == (
        BASE_FRAME,
        CHASSIS_LIDAR_FRAME,
    )
    assert base_to_chassis.translation.x == pytest.approx(LIDAR_MOUNT_XYZ[0])
    assert base_to_chassis.translation.y == pytest.approx(LIDAR_MOUNT_XYZ[1])
    assert base_to_chassis.translation.z == pytest.approx(LIDAR_MOUNT_XYZ[2])

    # Composed, the two put the chassis lidar exactly where Point-LIO's frame is.
    through = lidar_to_base + base_to_chassis
    assert through.translation.x == pytest.approx(0.0, abs=1e-9)
    assert through.translation.y == pytest.approx(0.0, abs=1e-9)
    assert through.translation.z == pytest.approx(0.0, abs=1e-9)


def test_mount_tf_publisher_publishes_exactly_those_edges(built) -> None:
    publisher = built(R1ProLioMountTf)
    assert [(t.frame_id, t.child_frame_id) for t in publisher.transforms()] == [
        (LIDAR_FRAME, BASE_FRAME),
        (BASE_FRAME, CHASSIS_LIDAR_FRAME),
    ]


def test_odometry_is_projected_onto_a_pose_with_the_same_stamp_and_frame(built, mocker) -> None:
    module = built(R1ProLioOdomPose)
    module.pose = mocker.MagicMock()
    odometry = Odometry(
        ts=1234.5,
        frame_id="odom",
        child_frame_id=LIDAR_FRAME,
        pose=Pose(Vector3(1.0, 2.0, 0.5), Quaternion(0.0, 0.0, 0.7071068, 0.7071068)),
    )

    module._on_odometry(odometry)

    published = module.pose.publish.call_args[0][0]
    assert published.ts == 1234.5
    assert published.frame_id == "odom"
    assert (published.position.x, published.position.y, published.position.z) == (1.0, 2.0, 0.5)
    assert published.orientation.z == pytest.approx(0.7071068)
    assert published.orientation.w == pytest.approx(0.7071068)


def test_blueprint_hangs_the_robot_off_pointlio_alone() -> None:
    atoms = {atom.module: atom for atom in r1pro_pointlio.active_blueprints}
    # The wheel odometry is off, so base_link has one parent: the mount tf's.
    assert atoms[R1ProConnection].kwargs["publish_odom"] is False
    assert R1ProLioMountTf in atoms
    assert atoms[PointLioRust].kwargs["sensor_frame_id"] == LIDAR_FRAME
    assert atoms[R1ProMid360].kwargs["frame_id"] == LIDAR_FRAME

    remaps = r1pro_pointlio.remapping_map
    key = r1pro_pointlio._instance_key
    # The driver's raw sweep is only for the estimator; the estimator's cloud
    # is the only copy of the chassis scan that reaches the bus.
    assert remaps[(key(R1ProMid360), "lidar")] == "lidar_raw"
    assert remaps[(key(PointLioRust), "lidar")] == "lidar"
    assert remaps[(key(PointLioRust), "odometry")] == "pointlio_odometry"
    assert remaps[(key(R1ProLioOdomPose), "odometry")] == "pointlio_odometry"
    # The name the planners already read.
    assert remaps[(key(R1ProLioOdomPose), "pose")] == "chassis_odom"


def test_network_from_the_vendor_file_when_nothing_else_says(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, built
) -> None:
    vendor = tmp_path / "MID360_config.json"
    vendor.write_text(
        json.dumps(
            {
                "MID360": {"host_net_info": {"cmd_data_ip": "192.168.2.150"}},
                "lidar_configs": [{"ip": "192.168.2.100"}],
            }
        )
    )
    monkeypatch.setenv(ENV_CONFIG_PATH, str(vendor))

    module = built(R1ProMid360, lidar_ip=None, host_ip=None)
    module._resolve_vendor_network()

    assert module.config.lidar_ip == "192.168.2.100"
    assert module.config.host_ip == "192.168.2.150"


def test_explicit_addresses_win_over_the_vendor_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, built
) -> None:
    monkeypatch.setenv(ENV_CONFIG_PATH, str(tmp_path / "does-not-exist.json"))
    module = built(R1ProMid360, lidar_ip="10.0.0.9", host_ip="10.0.0.1")
    module._resolve_vendor_network()
    assert (module.config.lidar_ip, module.config.host_ip) == ("10.0.0.9", "10.0.0.1")


def test_unknown_address_fails_naming_the_file_and_the_variables(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, built
) -> None:
    missing = tmp_path / "MID360_config.json"
    monkeypatch.setenv(ENV_CONFIG_PATH, str(missing))
    module = built(R1ProMid360, lidar_ip=None, host_ip=None)
    with pytest.raises(RuntimeError) as error:
        module._resolve_vendor_network()
    message = str(error.value)
    assert str(missing) in message
    assert "DIMOS_MID360_LIDAR_IP" in message
    assert ENV_CONFIG_PATH in message
