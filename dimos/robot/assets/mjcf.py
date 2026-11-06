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


"""A MuJoCo MJCF robot as a URDF, so one MJCF is the source for both the simulator and RobotModel users."""

from __future__ import annotations

from mjcf_to_urdf import mjcf_spec_to_urdf
import mujoco

from dimos.robot.assets.model import RobotModel


def mjcf_to_urdf(spec: mujoco.MjSpec) -> RobotModel:
    """The robot in an MJCF as a RobotModel; mesh paths stay relative to the MJCF's folder, as MuJoCo resolves them."""
    return RobotModel.from_urdf_string(mjcf_spec_to_urdf(spec), asset_dir=spec.modelfiledir)
