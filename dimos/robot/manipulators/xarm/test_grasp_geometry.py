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

import numpy as np

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.grasping.grasp_gen_x.module import (
    GraspGenXConfig,
    GraspGenXModule,
    GripperBox,
    SweepVolumeGripperConfig,
)
from dimos.robot.manipulators.xarm.blueprints.grasp import xarm_grasp_graspgenx
from dimos.robot.manipulators.xarm.grasp_geometry import (
    XARM_GRIPPER_BODY,
    XARM_GRIPPER_SWEEP_VOLUME,
)


def _bounds(extents: tuple[float, ...], offset: tuple[float, ...]) -> tuple[np.ndarray, np.ndarray]:
    half = np.asarray(extents) / 2.0
    return np.asarray(offset) - half, np.asarray(offset) + half


def test_the_fingers_flank_the_open_jaws_and_nothing_sits_between_them() -> None:
    boxes = [GripperBox.model_validate(box) for box in XARM_GRIPPER_BODY]
    jaws = SweepVolumeGripperConfig.model_validate(XARM_GRIPPER_SWEEP_VOLUME)
    jaw_low, jaw_high = _bounds(jaws.extents_open, jaws.offset_open)
    flanking = []
    for box in boxes:
        low, high = _bounds(box.extents, box.offset)
        # No part of the gripper reaches into the space the open jaws close on.
        overlap = np.minimum(high, jaw_high) - np.maximum(low, jaw_low)
        assert np.any(overlap <= 1e-3)
        # A finger lies against one side of that space for its whole height.
        touches_side = np.isclose(low[0], jaw_high[0], atol=1e-3) or np.isclose(
            high[0], jaw_low[0], atol=1e-3
        )
        if touches_side and low[2] <= jaw_low[2] and high[2] >= jaw_high[2]:
            flanking.append(box)
    assert len(flanking) == 2


def test_graspgenx_on_xarm_grasp_checks_grasps_with_the_xarm_gripper_body() -> None:
    parsed = BlueprintConfigParser(xarm_grasp_graspgenx).parse(environ={})
    config = GraspGenXConfig.model_validate(parsed.module_kwargs(GraspGenXModule.name))

    assert [box.model_dump() for box in config.gripper_body] == list(XARM_GRIPPER_BODY)
