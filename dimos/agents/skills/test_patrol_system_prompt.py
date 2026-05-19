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

import pytest

from dimos.agents.skills.patrol_system_prompt import PATROL_SYSTEM_PROMPT


class TestPatrolSystemPromptContent:
    """Verify the patrol system prompt contains all required sections."""

    def test_prompt_is_nonempty_string(self):
        assert isinstance(PATROL_SYSTEM_PROMPT, str)
        assert len(PATROL_SYSTEM_PROMPT) > 100

    def test_prompt_mentions_patrol_keyword(self):
        lower = PATROL_SYSTEM_PROMPT.lower()
        assert "patrol" in lower or "巡检" in PATROL_SYSTEM_PROMPT

    def test_prompt_mentions_robot_identity(self):
        lower = PATROL_SYSTEM_PROMPT.lower()
        assert "daneel" in lower
        assert "go2" in lower or "unitree" in lower

    @pytest.mark.parametrize(
        "skill_name",
        [
            "create_patrol_route",
            "add_patrol_waypoint",
            "add_current_position_as_waypoint",
            "start_patrol",
            "stop_patrol",
            "get_patrol_status",
            "list_patrol_routes",
            "save_patrol_route",
            "save_current_map",
            "load_saved_map",
            "check_map_coverage",
            "list_available_maps",
            "check_localization_quality",
            "show_patrol_status",
            "tag_location",
            "navigate_with_text",
            "stop_navigation",
            "relative_move",
            "wait",
            "observe",
            "speak",
        ],
    )
    def test_prompt_mentions_skill(self, skill_name):
        assert skill_name in PATROL_SYSTEM_PROMPT, (
            f"Skill '{skill_name}' not found in PATROL_SYSTEM_PROMPT"
        )

    @pytest.mark.parametrize(
        "section_keyword",
        [
            "安全",           # safety
            "巡检工作流程",    # patrol workflow
            "异常处理",       # anomaly handling
        ],
    )
    def test_prompt_has_key_sections(self, section_keyword):
        assert section_keyword in PATROL_SYSTEM_PROMPT, (
            f"Section keyword '{section_keyword}' not found in PATROL_SYSTEM_PROMPT"
        )

    def test_prompt_describes_workflow_steps(self):
        assert "准备" in PATROL_SYSTEM_PROMPT       # preparation
        assert "路线规划" in PATROL_SYSTEM_PROMPT     # route planning
        assert "执行巡检" in PATROL_SYSTEM_PROMPT     # execute patrol
        assert "监控" in PATROL_SYSTEM_PROMPT         # monitoring
        assert "汇报" in PATROL_SYSTEM_PROMPT         # reporting

    def test_prompt_includes_obstacle_handling(self):
        assert "障碍" in PATROL_SYSTEM_PROMPT

    def test_prompt_includes_battery_monitoring(self):
        assert "电池" in PATROL_SYSTEM_PROMPT or "电量" in PATROL_SYSTEM_PROMPT

    def test_prompt_includes_slam_reference(self):
        lower = PATROL_SYSTEM_PROMPT.lower()
        assert "slam" in lower or "定位" in PATROL_SYSTEM_PROMPT
