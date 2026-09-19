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

"""The R1 Pro with head stereo depth, placed by Point-LIO.

``r1pro-pointlio`` plus the head matched into a depth image and a point cloud
at camera rate, so the depth can be looked at in the viewer beside the lidar
before anything plans on it. The cloud is published as ``head_cloud`` in the
head's optical frame; a map that reads it resolves that frame through the tf
the connection publishes off the torso joints.

Usage, on the robot::

    dimos run r1pro-stereo --g.transport lcm

The matcher is a Rust native module built on first run (``cargo build
--release`` in the workspace). Its calibration comes from
``~/.dimos/r1pro/calibration.json`` (or ``DIMOS_R1_STEREO_CALIBRATION``) when
one exists, else the committed rig numbers; see
:mod:`dimos.robot.galaxea.r1pro.stereo_calibration`.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_visualization,
)
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_pointlio import r1pro_lidar_odometry
from dimos.robot.galaxea.r1pro.stereo import r1pro_stereo_cloud

r1pro_stereo = autoconnect(
    r1pro_visualization(),
    # The wrists are JPEG at ~28 Hz each and cost a decode whether or not
    # anything reads them; nothing here does.
    r1pro_control(publish_odom=False, enable_wrist_color=False),
    r1pro_lidar_odometry(),
    r1pro_stereo_cloud(),
).global_config(n_workers=5)
