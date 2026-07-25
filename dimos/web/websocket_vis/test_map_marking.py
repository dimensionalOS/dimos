"""Tests for non-motion semantic point marking in Command Center."""

from __future__ import annotations

import json

from dimos_lcm.std_msgs import String
import numpy as np

from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _module() -> WebsocketVisModule:
    module = WebsocketVisModule()
    module.costmap_encoder.last_full_grid = np.zeros((10, 12), dtype=np.int8)
    module.vis_state["costmap"] = {
        "grid": {"shape": [10, 12]},
        "origin": {"c": [-2.0, -3.0, 0.0]},
        "resolution": 0.5,
    }
    return module


def test_command_center_receives_map_marking_extension_once() -> None:
    module = _module()
    try:
        html = module._inject_map_marking_extension("<html><body>map</body></html>")
        assert html.count("/map-marking-extension.js") == 1
        assert module._inject_map_marking_extension(html) == html
    finally:
        module._close_module()


def test_armed_map_click_creates_candidate_without_navigation_goal() -> None:
    module = _module()
    candidates: list[String] = []
    goals = []
    unsubscribe_candidate = module.semantic_place_candidate.subscribe(
        candidates.append
    )
    unsubscribe_goal = module.goal_request.subscribe(goals.append)
    try:
        module._arm_map_marking(" 会场 正门 ")
        handled, candidate = module._consume_map_marking_click(1.0, 0.0)

        assert handled is True
        assert candidate is not None
        assert candidate["place"]["name"] == "会场 正门"
        assert candidate["place"]["pose"]["frame_id"] == "world"
        assert goals == []

        module.semantic_place_candidate.publish(
            String(json.dumps(candidate, ensure_ascii=False))
        )
        assert json.loads(candidates[-1].data)["place"]["name"] == "会场 正门"
        assert module._map_marking_state()["pending_name"] is None
        assert module._map_marking_state()["markers"][0]["status"] == "pending"
    finally:
        unsubscribe_candidate()
        unsubscribe_goal()
        module._close_module()


def test_obstacle_click_is_consumed_and_does_not_create_candidate() -> None:
    module = _module()
    module.costmap_encoder.last_full_grid[6, 6] = 100
    try:
        module._arm_map_marking("障碍物")
        handled, candidate = module._consume_map_marking_click(1.0, 0.0)

        assert handled is True
        assert candidate is None
        state = module._map_marking_state()
        assert state["pending_name"] == "障碍物"
        assert state["markers"] == []
        assert "可通行" in state["error"]
    finally:
        module._close_module()


def test_semantic_confirmation_updates_visible_marker_state() -> None:
    module = _module()
    try:
        module._arm_map_marking("门口")
        _, candidate = module._consume_map_marking_click(1.0, 0.0)
        assert candidate is not None

        module._on_semantic_place_confirmation(
            String(
                json.dumps(
                    {
                        "request_id": candidate["request_id"],
                        "accepted": True,
                    }
                )
            )
        )

        marker = module._map_marking_state()["markers"][0]
        assert marker["status"] == "confirmed"
        assert marker["reason"] is None
    finally:
        module._close_module()
