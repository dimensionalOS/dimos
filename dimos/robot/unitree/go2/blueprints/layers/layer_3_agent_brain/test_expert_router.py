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

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.expert_router import (
    _Go2ExpertRouter,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


def test_route_task_recommends_navigation_with_context() -> None:
    router = _Go2ExpertRouter()
    try:
        result = router.route_task("go to the kitchen")

        assert result.success is True
        assert result.metadata["domain"] == "navigation"
        assert result.metadata["needs_context"] is True
        assert result.metadata["recommended_tools"][0] == "get_context"
        assert "navigate_with_text" in result.metadata["recommended_tools"]
    finally:
        _stop_modules(router)


def test_route_task_prioritizes_safety_without_context() -> None:
    router = _Go2ExpertRouter()
    try:
        result = router.route_task("stop following and cancel navigation")

        assert result.success is True
        assert result.metadata["domain"] == "safety"
        assert result.metadata["needs_context"] is False
        assert "stop_navigation" in result.metadata["recommended_tools"]
    finally:
        _stop_modules(router)


def test_route_task_supports_chinese_person_follow() -> None:
    router = _Go2ExpertRouter()
    try:
        result = router.route_task("\u8ddf\u968f\u7a7f\u84dd\u8272\u8863\u670d\u7684\u4eba")

        assert result.success is True
        assert result.metadata["domain"] == "person_follow"
        assert "follow_person" in result.metadata["recommended_tools"]
    finally:
        _stop_modules(router)


def test_route_task_requires_task() -> None:
    router = _Go2ExpertRouter()
    try:
        result = router.route_task(" ")

        assert result.success is False
        assert result.error_code == "INVALID_INPUT"
    finally:
        _stop_modules(router)
