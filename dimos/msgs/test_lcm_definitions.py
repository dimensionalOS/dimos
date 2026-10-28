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

import importlib.util
from pathlib import Path

from dimos.msgs.helpers import lcm_msg_type

MSGS = Path(__file__).parent

# Predate this test. Define each in dimos-lcm (or move it to a dimos-only package), then drop it.
KNOWN = {
    # Mustafa Bhadsorawala, https://github.com/dimensionalOS/dimos/pull/831
    "sensor_msgs.JointCommand",
    "sensor_msgs.RobotState",
    "trajectory_msgs.TrajectoryPoint",
    "trajectory_msgs.TrajectoryStatus",
    # Mustafa Bhadsorawala, https://github.com/dimensionalOS/dimos/pull/1954
    "sensor_msgs.MotorCommandArray",
    # pomichterstash, https://github.com/dimensionalOS/dimos/pull/1511
    "visualization_msgs.EntityMarkers",
    # Jeff Hykin, https://github.com/dimensionalOS/dimos/pull/1791
    "nav_msgs.LineSegments3D",
    # Jeff Hykin, https://github.com/dimensionalOS/dimos/pull/3449
    "sensor_msgs.ImuInfo",
    # leshy, https://github.com/dimensionalOS/dimos/pull/4497
    "shape_msgs.TriangleMesh",
}


def test_dimos_lcm_packages_hold_only_dimos_lcm_messages() -> None:
    missing = set()
    for path in MSGS.glob("*/*.py"):
        if path.name.startswith(("test_", "tool_", "_")):
            continue
        package, name = path.parent.name, path.stem
        if importlib.util.find_spec(f"dimos_lcm.{package}") is None:
            continue  # a dimos-only package, free to add to
        try:
            lcm_msg_type(f"{package}.{name}")
        except ImportError:
            missing.add(f"{package}.{name}")
    new = sorted(missing - KNOWN)
    packages = sorted({m.split(".")[0] for m in new})
    assert not new, (
        f"{new} are not defined in dimos-lcm.\n"
        f"{packages} mirror ROS packages from dimos-lcm, which generates every language's "
        "bindings from them, so nothing new goes in there.\n"
        "New messages can be added temporarily, but not in a ROS package:\n"
        "  - first look for an existing ROS message that fits, often one already does\n"
        "  - the right way: add it to dimos-lcm, new ROS namespaces are welcome there\n"
        "  - meanwhile, a package dimos-lcm doesn't have, e.g. dimos/msgs/<yours>_msgs/\n",
        "but keep in mind, this breaks native module support so a lot of the system will not work with it and it will be removed eventually.\n",
    )
    assert not KNOWN - missing, f"{sorted(KNOWN - missing)} are defined now, drop from KNOWN"
