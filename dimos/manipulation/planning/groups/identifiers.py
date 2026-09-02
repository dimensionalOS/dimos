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

"""Canonical planning-group and joint-name validation."""

from collections.abc import Sequence

from dimos.manipulation.planning.spec.models import JointName, PlanningGroupID


def assert_valid_group_id(group_id: PlanningGroupID) -> None:
    """Validate a stable, unprefixed planning-group name."""
    if not group_id or "/" in group_id:
        raise ValueError(f"Invalid planning group ID: {group_id!r}")


def assert_valid_joint_name(joint_name: JointName) -> None:
    """Validate an exact canonical model joint name."""
    if not joint_name or joint_name.startswith("/") or joint_name.endswith("/"):
        raise ValueError(f"Invalid canonical joint name: {joint_name!r}")
    if any(not part for part in joint_name.split("/")):
        raise ValueError(f"Invalid canonical joint name: {joint_name!r}")


def assert_valid_joint_names(names: Sequence[JointName]) -> None:
    """Validate canonical joint names."""
    for name in names:
        assert_valid_joint_name(name)
