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

"""Arm calibration for any arm described by a robot model and an ``ArmCalibrationSpec``.

``spec`` describes one arm, ``robots`` names the known Damiao robots,
``gravity_poses`` and ``gravity_fit`` identify its link masses, ``step_analysis``
and ``float_analysis`` judge its gains, and ``schema`` stores the result as one
versioned calibration file per arm.
"""
