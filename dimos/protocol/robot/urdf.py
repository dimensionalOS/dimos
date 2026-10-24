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

"""URDF <-> RobotModel, stdlib xml only.

What URDF can't say goes in one <dimos> block per joint or link, the way <gazebo
reference="..."> carries simulator data. URDF parsers skip unknown top-level elements,
so the file stays valid for MuJoCo, rerun, pinocchio and the rest:

    <dimos reference="FR_calf_joint">
      <limit acceleration="200.0" jerk="5000.0"/>
      <meta key="motor" value="M3"/>
    </dimos>

Robot-level data has no reference:

    <dimos odom_frame="odom">
      <base frame="base_link" height_clearance="0.45" width_clearance="0.4">
        <twist unit="m/s,rad/s" vx="1.5" vy="0.5" vz="0" wx="0" wy="0" wz="2.0"/>
        <accel unit="m/s^2,rad/s^2" vx="2.0" vy="1.0" wz="4.0"/>
        <footprint><box size="0.7 0.31 0.4"/></footprint>
      </base>
    </dimos>

SKETCH: signatures only.
"""

from __future__ import annotations

from dimos.protocol.robot.model import RobotModel

DIMOS_TAG = "dimos"


def parse_urdf(xml: str) -> RobotModel:
    """Mesh and texture uris are kept as written; each consumer resolves them itself."""
    raise NotImplementedError


def write_urdf(robot: RobotModel) -> str:
    """Canonical: same model, same text (fixed attribute order and float format), so the
    text's sha256 is the model's digest."""
    raise NotImplementedError
