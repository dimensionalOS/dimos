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

"""Which lidar mount each exported stack ends up with.

The motion stacks re-declare GO2Zenoh on top of `go2_zenoh_basic`'s and rely on
`autoconnect` keeping the newest duplicate. That is deliberate (see
`_eliminate_duplicates`), but it is order-dependent, so the resolved value is
pinned here rather than left to the reader.
"""

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.unitree.go2.go2_mid360_static_transforms import MID360_MOUNT_PRESETS
from dimos.robot.unitree.go2.tf.go2_tf import Go2TfConfig
from dimos.robot.unitree.go2.zenoh import blueprints as bp
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2Zenoh, GO2ZenohConfig


def _mount(blueprint: Blueprint) -> tuple[float, float, float]:
    """The mount GO2Zenoh actually starts with, resolved the way the config is."""
    (atom,) = [a for a in blueprint.blueprints if a.module is GO2Zenoh]
    return tuple(GO2ZenohConfig(**atom.kwargs).mid360_mount)


@pytest.mark.parametrize(
    ("name", "preset"),
    [
        ("go2_zenoh_basic", "SF"),
        ("go2_zenoh_raycaster", "SF"),
        ("go2_zenoh_nav", "SF"),
        ("go2_zenoh_nav_remote", "SF"),
        ("go2_zenoh_nav_baked", "SF"),
        ("go2_zenoh_htc", "SF"),
        ("go2_zenoh_motion", "ATHENS"),
        ("go2_zenoh_motion_local", "ATHENS"),
    ],
)
def test_the_stack_runs_the_mount_it_declares(name: str, preset: str) -> None:
    assert _mount(getattr(bp, name)) == MID360_MOUNT_PRESETS[preset]


def test_the_motion_stacks_agree_with_the_baked_tf():
    """Both publish the same edges; disagree and base_link jumps between two mounts."""
    assert list(_mount(bp.go2_zenoh_motion)) == Go2TfConfig().mid360_mount_rpy_deg
