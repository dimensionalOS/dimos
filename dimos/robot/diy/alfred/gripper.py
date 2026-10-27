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

"""The OpenArm v1.0 parallel gripper, the one on Alfred's arms, as GraspGenX describes a gripper.

GraspGenX predicts a pose for a gripper frame whose +Z is the approach and whose X is the axis
the jaws close along, and wants the volume the fingers sweep as an axis-aligned box in that
frame. Here that frame is ``openarm_<side>_ee_base_link`` (the wrist, J7's axis) turned half a
turn about x: Alfred's gripper hangs down the wrist's -z, and its fingers slide along the
wrist's x. Every number is measured off the robot description the URDF carries, in
``data/alfred_description``: the finger joints (``swap_openarm_v10_gripper.py``) and the baked
finger meshes (``meshes/openarm_v10/left_finger_*.stl``).
"""

from __future__ import annotations

from typing import Any

# link7 -> the rail plate the sliders run on, along the gripper axis (HAND_Z).
_PLATE_DEPTH_M = 0.1025
# The finger below the plate, down to the pad's tip (finger mesh, z extent below the slider).
_FINGER_BELOW_PLATE_M = 0.0924
# The last stretch of each finger angles in to the centreline and carries the pad: the part that
# touches an object. Above it the finger stands 9.5 mm off centre and is no grasp surface.
_PAD_LENGTH_M = 0.030
# The pad across the finger (the finger is 27 mm wide at the top of the pad, 16 mm at its tip).
_PAD_WIDTH_M = 0.026
# Each slider's travel (prismatic, 0 closed .. 0.044 open); the two pads overlap by 2.8 mm each
# at zero travel in the CAD, so the jaw opens 5.6 mm less than the 88 mm of travel.
_SLIDER_TRAVEL_M = 0.044
_PAD_OVERLAP_M = 0.0028
# The wrist -> grasp_frame distance the URDF fixes (HAND_Z + TCP_Z): the planning TCP, between
# the pads, 8.9 mm up from their tips.
_TCP_DEPTH_M = 0.186

OPENARM_V10_FINGERTIP_DEPTH_M = _PLATE_DEPTH_M + _FINGER_BELOW_PLATE_M
_PAD_CENTRE_DEPTH_M = OPENARM_V10_FINGERTIP_DEPTH_M - _PAD_LENGTH_M / 2


def _jaw(travel_per_finger: float) -> float:
    return 2 * (travel_per_finger - _PAD_OVERLAP_M)


# GraspGenX's sweep volume for the v1.0 gripper, in its convention (approach +Z, jaws closing
# along X, origin on the wrist). The box is the space between the pads, its far face at the
# fingertips; the sliders are prismatic, so closing moves nothing along the approach.
OPENARM_V10_SWEEP_VOLUME: dict[str, Any] = {
    "extents_open": (round(_jaw(_SLIDER_TRAVEL_M), 4), _PAD_WIDTH_M, _PAD_LENGTH_M),
    "offset_open": (0.0, 0.0, round(_PAD_CENTRE_DEPTH_M, 4)),
    "extents_half_open": (round(_jaw(_SLIDER_TRAVEL_M / 2), 4), _PAD_WIDTH_M, _PAD_LENGTH_M),
    "offset_half_open": (0.0, 0.0, round(_PAD_CENTRE_DEPTH_M, 4)),
    "fingertip_depth": round(OPENARM_V10_FINGERTIP_DEPTH_M, 4),
    "family": "parallel_2f",
}

# The GraspGenX gripper frame -> ``openarm_<side>_grasp_frame``, the planning tip: the TCP's
# depth along the approach, plus the quarter turn that takes GraspGenX's X closing axis onto
# grasp_frame's y (the URDF's grasp_frame has +z out of the gripper and +y across the jaw).
OPENARM_V10_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, _TCP_DEPTH_M),
    (0.0, 0.0, 0.0, 1.0),
)
