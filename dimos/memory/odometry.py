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

"""Recorded odom streams, stored as ``PoseStamped`` in older recordings and ``Odometry`` in newer ones."""

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry

ODOM_STREAMS = frozenset({"odom", "go2_odom", "chassis_odom"})


def as_odometry(msg: PoseStamped | Odometry) -> Odometry:
    """A recorded odom payload as ``Odometry``, with a zero twist for a ``PoseStamped``."""
    return msg if isinstance(msg, Odometry) else Odometry.from_pose_stamped(msg)
