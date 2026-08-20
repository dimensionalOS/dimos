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

"""A750 model configuration contracts."""

import xml.etree.ElementTree as ET

import numpy as np
import pinocchio
import pytest

from dimos.robot.manipulators.a750.config import make_a750_model_config


@pytest.mark.self_hosted
def test_gripper_zero_configuration_is_within_planning_model_limits() -> None:
    loaded = make_a750_model_config().model.load()
    root = ET.fromstring(loaded.xml)

    for name in ("finger", "finger_mimic"):
        limit = root.find(f"joint[@name='{name}']/limit")
        assert limit is not None
        assert limit.get("lower") == "0.0"
        assert limit.get("upper") == "0.06"

    model = pinocchio.buildModelFromXML(loaded.xml)
    neutral = pinocchio.neutral(model)

    assert np.all(neutral >= model.lowerPositionLimit)
    assert np.all(neutral <= model.upperPositionLimit)
