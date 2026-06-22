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

"""Unit tests for planning group identifier grammar."""

from __future__ import annotations

import pytest

from dimos.manipulation.planning.groups.identifiers import (
    local_joint_name_from_global,
    parse_global_joint_name,
)


def test_parse_global_joint_name_returns_robot_and_local_joint() -> None:
    assert parse_global_joint_name("left_arm/joint1") == ("left_arm", "joint1")


@pytest.mark.parametrize("name", ["joint1", "/joint1", "left_arm/", "left_arm/foo/bar"])
def test_parse_global_joint_name_rejects_invalid_names(name: str) -> None:
    with pytest.raises(ValueError, match="Invalid global joint name"):
        parse_global_joint_name(name)


def test_local_joint_name_from_global_requires_matching_robot() -> None:
    with pytest.raises(ValueError, match="does not belong to robot"):
        local_joint_name_from_global("right_arm", "left_arm/joint1")
