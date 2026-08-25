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

"""The Go2's tf tree, in rust, so it can be baked onto the robot.

This is the tf half of :class:`~dimos.robot.unitree.go2.zenoh.zenohconnection.GO2Zenoh`
and nothing else: the mount tree and the ``odom -> mid360_link`` edge. Baked
into the robot host beside the planner and the follower, the mount leg they
both wait on stops crossing wifi. Everything else GO2Zenoh does -- the bridge's
streams, the action verbs, the camera intrinsics -- stays on the laptop, so the
two coexist as long as only one of them publishes tf.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.constants import CAMERA_XYZ, MID360_MOUNT_PRESETS, MID360_XYZ


class Go2TfConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/go2_tf"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    # Every field below crosses to the rust struct verbatim, and every one of
    # them must: a native config has no rust-side defaults, so a field added
    # there and not here fails startup with `missing [...]`.
    #
    # The geometry is the rig GO2Zenoh measures, read from the same constants it
    # does so the two cannot drift apart. Lists, not Vector3/tuple: every field
    # here crosses to rust as JSON. The mount is the ATHENS preset -- a stack on
    # another rig must set BOTH halves to its own.
    publish_hz: float = 5.0
    camera_xyz: list[float] = Field(default_factory=lambda: [*CAMERA_XYZ])
    mid360_xyz: list[float] = Field(default_factory=lambda: [*MID360_XYZ])
    mid360_mount_rpy_deg: list[float] = Field(
        default_factory=lambda: [*MID360_MOUNT_PRESETS["ATHENS"]]
    )


class Go2Tf(NativeModule):
    """The Go2's mount tree on a timer, plus the odom edge per odometry message."""

    config: Go2TfConfig

    odometry: In[Odometry]

    tf: Out[TFMessage]


if TYPE_CHECKING:
    Go2Tf()
