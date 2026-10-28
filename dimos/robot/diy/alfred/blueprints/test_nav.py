# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.robot.diy.alfred.blueprints._nav import _path_colored


def test_generated_path_preserves_the_alfred_overlay_color_and_height():
    header = Header(Time(17, 1), "world")
    path = Path(
        header, [PoseStamped(header, Pose(Point(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)))]
    )
    value = _path_colored(path, (20, 30, 40))
    assert value.strips.as_arrow_array().to_pylist() == [[[1.0, 2.0, 3.5]]]
    assert value.colors.as_arrow_array().to_pylist() == [337520895]
