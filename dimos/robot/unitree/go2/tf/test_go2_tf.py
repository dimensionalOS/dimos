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

"""The rust go2_tf module publishes the same tree GO2Zenoh does.

Both sides assert against ``tf_parity.json``: this file derives it from the
python (``GO2Zenoh.transforms()`` + ``Transform.from_pose``, resolved through
the same ``MultiTBuffer`` the planner and follower read tf with), and
``rust/src/tree.rs`` derives it from the crate. Neither can drift alone, and the
number that matters -- where ``base_link`` ends up -- is compared per odometry
pose rather than eyeballed off the two implementations.

Regenerate after an intentional geometry change with
``DIMOS_WRITE_TF_PARITY=1 pytest dimos/robot/unitree/go2/tf``.
"""

import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.unitree.go2.tf.go2_tf import Go2TfConfig
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2Zenoh, GO2ZenohConfig

FIXTURE = Path(__file__).parent / "tf_parity.json"
TOLERANCE = 1e-12

# (name, position, rpy of the lidar frame in odom). Not all-zero rotations: the
# mount is a rotation AND a lever arm, so a pose with only yaw would hide a
# whole class of composition errors.
CASES = [
    ("at rest at the origin", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("driven out and turned", (1.0, 2.0, 0.0), (0.0, 0.0, math.pi / 2)),
    ("backwards, off level", (-3.25, 0.5, 0.1), (0.05, -0.12, 2.7)),
    ("nose down on a ramp", (7.0, -1.5, 0.42), (-0.3, 0.25, -1.1)),
]


def _config() -> GO2ZenohConfig:
    return GO2ZenohConfig(mid360_mount=tuple(Go2TfConfig().mid360_mount_rpy_deg))


def _static_transforms() -> list[Transform]:
    """GO2Zenoh's own transforms(), which only reads self.config."""
    return GO2Zenoh.transforms(SimpleNamespace(config=_config()))


def _odometry(position: tuple[float, float, float], rpy: tuple[float, float, float]) -> Odometry:
    return Odometry(
        frame_id="odom",
        child_frame_id="mid360_link",
        pose=Pose(
            position=Vector3(*position),
            orientation=Quaternion.from_euler(Vector3(*rpy)),
        ),
    )


def _resolve(odom: Odometry, child: str) -> Transform:
    """odom -> child through exactly the tf a consumer would have received."""
    buffer = MultiTBuffer()
    buffer.receive_transform(*_static_transforms())
    buffer.receive_transform(Transform.from_pose(odom.child_frame_id, odom.to_pose_stamped()))
    leg = buffer.get("odom", child)
    assert leg is not None, f"odom -> {child} does not resolve"
    return leg


def _dump(transform: Transform) -> dict[str, object]:
    return {
        "parent": transform.frame_id,
        "child": transform.child_frame_id,
        "translation": [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ],
        "rotation": [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
    }


def _fixture() -> dict[str, object]:
    config = Go2TfConfig()
    cases = []
    for name, position, rpy in CASES:
        odom = _odometry(position, rpy)
        cases.append(
            {
                "name": name,
                "odom": {
                    "frame_id": odom.frame_id,
                    "child_frame_id": odom.child_frame_id,
                    "position": list(position),
                    "orientation": [
                        odom.orientation.x,
                        odom.orientation.y,
                        odom.orientation.z,
                        odom.orientation.w,
                    ],
                },
                "base_link": _dump(_resolve(odom, "base_link")),
                "camera_optical": _dump(_resolve(odom, "camera_optical")),
            }
        )
    return {
        "config": {
            "publish_hz": config.publish_hz,
            "camera_xyz": config.camera_xyz,
            "mid360_xyz": config.mid360_xyz,
            "mid360_mount_rpy_deg": config.mid360_mount_rpy_deg,
        },
        "static": [_dump(t) for t in _static_transforms()],
        "cases": cases,
    }


def _assert_same_rotation(got: Quaternion, want: list[float], what: str) -> None:
    """Compare by what the rotation does: q and -q are the same rotation."""
    other = Quaternion(*want)
    for axis in (Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)):
        a, b = got.rotate_vector(axis), other.rotate_vector(axis)
        for lhs, rhs, component in ((a.x, b.x, "x"), (a.y, b.y, "y"), (a.z, b.z, "z")):
            assert abs(lhs - rhs) < TOLERANCE, f"{what}: R*e {component} {lhs} != {rhs}"


def _assert_same(got: Transform, want: dict, what: str) -> None:
    assert (got.frame_id, got.child_frame_id) == (want["parent"], want["child"]), what
    for value, expected, component in (
        (got.translation.x, want["translation"][0], "x"),
        (got.translation.y, want["translation"][1], "y"),
        (got.translation.z, want["translation"][2], "z"),
    ):
        assert abs(value - expected) < TOLERANCE, f"{what}: {component} {value} != {expected}"
    _assert_same_rotation(got.rotation, want["rotation"], what)


def test_the_fixture_still_matches_the_python():
    """The rust crate asserts against this same file, so this pins both."""
    fixture = _fixture()
    if os.environ.get("DIMOS_WRITE_TF_PARITY"):
        FIXTURE.write_text(json.dumps(fixture, indent=2) + "\n")

    stored = json.loads(FIXTURE.read_text())
    assert stored["config"] == fixture["config"]
    for stored_edge, edge in zip(stored["static"], _static_transforms(), strict=True):
        _assert_same(edge, stored_edge, f"{edge.frame_id} -> {edge.child_frame_id}")
    for stored_case, (name, position, rpy) in zip(stored["cases"], CASES, strict=True):
        odom = _odometry(position, rpy)
        _assert_same(_resolve(odom, "base_link"), stored_case["base_link"], name)
        _assert_same(_resolve(odom, "camera_optical"), stored_case["camera_optical"], name)


def test_both_halves_publish_the_tree_at_the_same_rate():
    """The xyz/mount geometry is now shared (go2.constants); the rate still is not."""
    assert Go2TfConfig().publish_hz == GO2ZenohConfig().publish_hz


def test_the_tree_leaves_the_lidar_a_single_parent():
    """Point-LIO owns mid360_link; a second parent snaps the body at 35 Hz."""
    assert all(t.child_frame_id != "mid360_link" for t in _static_transforms())
