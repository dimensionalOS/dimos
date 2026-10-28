# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header
from dimos.robot.assets.model import RobotModel


def test_configuration_json_restores_canonical_nested_message_types():
    source = PoseStamped(
        Header(Time(123, 125000000), "world"),
        Pose(Point(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
    config = RobotModelConfig(
        model=RobotModel.from_file(Path("fake.urdf")),
        joint_names=["arm/joint"],
        base_pose=source,
    )
    restored = RobotModelConfig.model_validate_json(
        config.model_dump_json(exclude={"rpc_transport"})
    )
    assert type(restored.base_pose.header.stamp) is Time
    assert restored.base_pose == source


def test_camera_info_configuration_resolves_numpy_annotations():
    import numpy as np

    from dimos.core.module import ModuleConfig
    from dimos.message_utils.camera_info import from_intrinsics
    from dimos.messages.sensor_msgs.msg import CameraInfo

    class Configuration(ModuleConfig):
        camera_info: CameraInfo

    source = from_intrinsics(100.0, 110.0, 32.0, 24.0, 64, 48, header=Header(Time(7, 8), "camera"))
    restored = Configuration(camera_info=source).camera_info
    assert restored is source
    assert type(restored.header.stamp) is Time
    np.testing.assert_array_equal(restored.k, [100.0, 0, 32.0, 0, 110.0, 24.0, 0, 0, 1])
