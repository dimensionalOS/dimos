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

from __future__ import annotations

import json
from typing import Any

from langchain_core.messages import HumanMessage

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.siteops import SiteOpsSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config

SITEOPS_SKILLS = {
    "scan_zone",
    "inspect_asset",
    "read_gauge",
    "check_clearance",
    "detect_blocked_aisle",
    "scan_receiving_manifest",
    "open_work_order",
    "verify_work_order",
    "nav_eval_report",
}


def _loads(text: str) -> dict[str, Any]:
    return json.loads(text)


def test_siteops_skills_return_stable_results() -> None:
    skill = SiteOpsSkillContainer()
    try:
        names = {info.func_name for info in skill.get_skills()}
        assert SITEOPS_SKILLS <= names

        first_scan = _loads(skill.scan_zone("COOLING_1"))
        second_scan = _loads(skill.scan_zone("cooling 1"))
        assert first_scan == second_scan
        assert first_scan["incidents"][0]["issue_type"] == "blocked_aisle"

        asset = _loads(skill.inspect_asset("COOLING_UNIT_1"))
        assert asset["status"] == "warning"
        assert {issue["issue_type"] for issue in asset["issues"]} == {
            "abnormal_gauge_reading",
            "asset_status_warning",
            "insufficient_clearance",
        }

        gauge = _loads(skill.read_gauge("COOLING_UNIT_1"))
        assert gauge["status"] == "abnormal"
        assert gauge["reading"] == 82.4

        clearance = _loads(skill.check_clearance("COOLING_UNIT_1"))
        assert clearance["status"] == "blocked"
        assert clearance["clear"] is False

        aisle = _loads(skill.detect_blocked_aisle("COOLING_1"))
        assert aisle["blocked"] is True
        assert aisle["blockers"][0]["id"] == "BOX-COOLING-7"

        manifest = _loads(skill.scan_receiving_manifest("INBOUND_DOCK"))
        assert manifest["status"] == "mismatch"
        assert manifest["missing_packages"] == ["PKG-1002"]
        assert manifest["unexpected_packages"] == ["PKG-1003"]

        order = _loads(skill.open_work_order("COOLING_1", "blocked_aisle"))
        duplicate_order = _loads(skill.open_work_order("COOLING_1", "blocked_aisle"))
        assert order["work_order_id"] == duplicate_order["work_order_id"]
        assert order["created"] is True
        assert duplicate_order["created"] is False

        verification = _loads(skill.verify_work_order(order["work_order_id"]))
        assert verification["status"] == "verification_failed"
        assert verification["issue_still_active"] is True

        report = _loads(skill.nav_eval_report("dogops-demo"))
        assert report["waypoints_attempted"] == 5
        assert report["waypoint_success_rate"] == 1.0
    finally:
        skill.stop()


def test_siteops_skills_are_exposed_and_callable_over_mcp(mcp_url: str, mcp_port: int) -> None:
    global_config.update(viewer="none", n_workers=1, mcp_port=mcp_port)
    blueprint = autoconnect(SiteOpsSkillContainer.blueprint(), McpServer.blueprint())
    coordinator = ModuleCoordinator.build(blueprint)
    adapter = McpAdapter(url=mcp_url)
    try:
        assert adapter.wait_for_ready()
        tools = adapter.list_tools()
        tool_names = {tool["name"] for tool in tools}
        assert SITEOPS_SKILLS <= tool_names

        scan = _loads(adapter.call_tool_text("scan_zone", {"zone_id": "COOLING_1"}))
        assert scan["zone_id"] == "COOLING_1"

        asset = _loads(adapter.call_tool_text("inspect_asset", {"asset_id": "COOLING_UNIT_1"}))
        assert asset["status"] == "warning"

        gauge = _loads(adapter.call_tool_text("read_gauge", {"asset_id": "COOLING_UNIT_1"}))
        assert gauge["status"] == "abnormal"

        clearance = _loads(
            adapter.call_tool_text("check_clearance", {"asset_id": "COOLING_UNIT_1"})
        )
        assert clearance["clear"] is False

        aisle = _loads(adapter.call_tool_text("detect_blocked_aisle", {"zone_id": "COOLING_1"}))
        assert aisle["blocked"] is True

        manifest = _loads(
            adapter.call_tool_text("scan_receiving_manifest", {"zone_id": "INBOUND_DOCK"})
        )
        assert manifest["missing_packages"] == ["PKG-1002"]

        order = _loads(
            adapter.call_tool_text(
                "open_work_order",
                {"entity_id": "COOLING_1", "issue_type": "blocked_aisle"},
            )
        )
        assert order["created"] is True

        verification = _loads(
            adapter.call_tool_text(
                "verify_work_order",
                {"work_order_id": order["work_order_id"]},
            )
        )
        assert verification["status"] == "verification_failed"

        report = _loads(adapter.call_tool_text("nav_eval_report", {"run_id": "dogops-demo"}))
        assert report["verification_revisit_success"] is True
    finally:
        coordinator.stop()


def test_siteops_agent_calls_read_gauge(agent_setup) -> None:
    history = agent_setup(
        blueprints=[SiteOpsSkillContainer.blueprint()],
        messages=[HumanMessage("Read the COOLING_UNIT_1 gauge using the SiteOps read_gauge tool.")],
        fixture="test_siteops_agent_calls_read_gauge.json",
    )

    tool_calls = [
        tool_call
        for message in history
        if hasattr(message, "tool_calls")
        for tool_call in message.tool_calls
    ]
    assert any(tool_call["name"] == "read_gauge" for tool_call in tool_calls)
    assert "82.4" in history[-1].content
