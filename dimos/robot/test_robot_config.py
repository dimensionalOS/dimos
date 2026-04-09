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

"""Unit tests for robot catalog functions and RobotConfig conversions."""

from __future__ import annotations

from dimos.robot.catalog.ufactory import xarm6, xarm7


class TestCatalogFunctions:
    """Verify catalog functions produce correct defaults."""

    def test_xarm7_defaults(self):
        cfg = xarm7()
        assert cfg.name == "arm"
        assert cfg.joint_names == [f"joint{i}" for i in range(1, 8)]
        assert cfg.end_effector_link == "link7"
        assert cfg.adapter_type == "mock"
        assert cfg.base_link == "link_base"

    def test_xarm6_defaults(self):
        cfg = xarm6()
        assert cfg.name == "arm"
        assert cfg.joint_names == [f"joint{i}" for i in range(1, 7)]
        # xarm6 defaults to add_gripper=True
        assert cfg.end_effector_link == "link_tcp"
        assert cfg.adapter_type == "mock"

    def test_xarm7_with_gripper(self):
        cfg = xarm7(add_gripper=True)
        assert cfg.end_effector_link == "link_tcp"
        assert cfg.gripper is not None
        assert len(cfg.collision_exclusion_pairs) > 0

    def test_xarm7_base_pose_offsets(self):
        cfg = xarm7(x_offset=0.1, y_offset=0.5, z_offset=0.2)
        assert cfg.base_pose[0] == 0.1
        assert cfg.base_pose[1] == 0.5
        assert cfg.base_pose[2] == 0.2


class TestRobotConfigConversions:
    """Verify RobotConfig → RobotModelConfig / HardwareComponent / TaskConfig."""

    def test_to_robot_model_config(self):
        cfg = xarm7(name="left")
        rmc = cfg.to_robot_model_config()
        assert rmc.name == "left"
        assert rmc.joint_names == [f"joint{i}" for i in range(1, 8)]
        assert rmc.coordinator_task_name == "traj_left"
        assert rmc.end_effector_link == "link7"
        assert rmc.base_link == "link_base"
        # joint_name_mapping should map prefixed → unprefixed
        assert rmc.joint_name_mapping == {f"left/joint{i}": f"joint{i}" for i in range(1, 8)}

    def test_to_hardware_component(self):
        cfg = xarm7(name="arm")
        hw = cfg.to_hardware_component()
        assert hw.hardware_id == "arm"
        assert hw.adapter_type == "mock"
        assert len(hw.joints) == 7
        assert all(j.startswith("arm/") for j in hw.joints)

    def test_to_task_config(self):
        cfg = xarm7(name="arm")
        tc = cfg.to_task_config()
        assert tc.name == "traj_arm"
        assert tc.type == "trajectory"
        assert len(tc.joint_names) == 7

    def test_joint_name_mapping(self):
        cfg = xarm7(name="left")
        mapping = cfg.joint_name_mapping
        assert mapping == {f"left/joint{i}": f"joint{i}" for i in range(1, 8)}

    def test_coordinator_joint_names(self):
        cfg = xarm7(name="arm")
        names = cfg.coordinator_joint_names
        assert names == [f"arm/joint{i}" for i in range(1, 8)]
