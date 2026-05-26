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

from copy import deepcopy
import json
import re
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module

_SITE_STATE: dict[str, Any] = {
    "zones": {
        "HOME": {
            "description": "Operations desk and report station",
            "detections": [{"type": "station", "id": "NOC_DESK", "status": "ready"}],
            "blocked": False,
            "blockers": [],
        },
        "INBOUND_DOCK": {
            "description": "Receiving area with staged packages",
            "detections": [
                {"type": "package", "id": "PKG-1001", "status": "expected"},
                {"type": "package", "id": "PKG-1003", "status": "unexpected"},
                {"type": "tag", "id": "APRILTAG-INBOUND", "status": "visible"},
            ],
            "blocked": False,
            "blockers": [],
        },
        "COOLING_1": {
            "description": "Cooling unit row one",
            "detections": [
                {"type": "asset", "id": "COOLING_UNIT_1", "status": "warning"},
                {"type": "gauge", "id": "COOLING_UNIT_1", "status": "high"},
                {"type": "tag", "id": "APRILTAG-COOLING-1", "status": "visible"},
            ],
            "blocked": True,
            "blockers": [{"id": "BOX-COOLING-7", "distance_m": 0.18}],
        },
        "QA_HOLD": {
            "description": "Quality hold shelf for manual remediation",
            "detections": [
                {"type": "package", "id": "PKG-1002", "status": "remediated"},
                {"type": "tag", "id": "APRILTAG-QA-HOLD", "status": "visible"},
            ],
            "blocked": False,
            "blockers": [],
        },
    },
    "assets": {
        "COOLING_UNIT_1": {
            "zone_id": "COOLING_1",
            "asset_type": "cooling_unit",
            "status": "warning",
            "clearance_m": 0.18,
            "min_clearance_m": 0.6,
            "gauge": {
                "reading": 82.4,
                "unit": "F",
                "normal_min": 55.0,
                "normal_max": 75.0,
            },
        },
        "NOC_DESK": {
            "zone_id": "HOME",
            "asset_type": "station",
            "status": "ready",
            "clearance_m": 1.2,
            "min_clearance_m": 0.6,
            "gauge": None,
        },
    },
    "manifest": {
        "INBOUND_DOCK": [
            {"id": "PKG-1001", "expected_zone": "INBOUND_DOCK"},
            {"id": "PKG-1002", "expected_zone": "INBOUND_DOCK"},
        ],
        "QA_HOLD": [{"id": "PKG-1002", "expected_zone": "QA_HOLD"}],
    },
    "runs": {
        "dogops-demo": {
            "route": ["HOME", "INBOUND_DOCK", "COOLING_1", "QA_HOLD", "HOME"],
            "waypoints_attempted": 5,
            "waypoints_reached": 5,
            "route_elapsed_s": 164.2,
            "navigation_retries": 1,
            "guided_interventions": 1,
            "tag_reacquisition_success": True,
            "tag_reacquisition_time_s": 2.7,
            "verification_revisit_success": True,
            "planned_distance_m": 18.6,
            "actual_distance_m": 20.1,
        }
    },
}


def _json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True)


def _normalize_id(value: str) -> str:
    return value.strip().upper().replace(" ", "_")


def _work_order_id(entity_id: str, issue_type: str) -> str:
    slug = re.sub(r"[^A-Z0-9]+", "-", f"{entity_id}-{issue_type}".upper()).strip("-")
    return f"WO-{slug}"


class SiteOpsSkillContainer(Module):
    """Deterministic DogOps/SiteOps skills for hackathon validation."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._site_state = deepcopy(_SITE_STATE)
        self._work_orders: dict[str, dict[str, Any]] = {}

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def scan_zone(self, zone_id: str) -> str:
        """Scan a SiteOps zone and return deterministic detections and incidents.

        Args:
            zone_id: Semantic zone id such as HOME, INBOUND_DOCK, COOLING_1, or QA_HOLD.
        """
        zone_key = _normalize_id(zone_id)
        zone = self._site_state["zones"].get(zone_key)
        if zone is None:
            return _json({"ok": False, "error": "unknown_zone", "zone_id": zone_id})

        incidents = []
        if zone["blocked"]:
            incidents.append(
                {
                    "entity_id": zone_key,
                    "issue_type": "blocked_aisle",
                    "severity": "high",
                }
            )

        return _json(
            {
                "ok": True,
                "zone_id": zone_key,
                "description": zone["description"],
                "detections": zone["detections"],
                "incidents": incidents,
            }
        )

    @skill
    def inspect_asset(self, asset_id: str) -> str:
        """Inspect a known SiteOps asset and return current status and related issues.

        Args:
            asset_id: Asset id such as COOLING_UNIT_1 or NOC_DESK.
        """
        asset_key = _normalize_id(asset_id)
        asset = self._site_state["assets"].get(asset_key)
        if asset is None:
            return _json({"ok": False, "error": "unknown_asset", "asset_id": asset_id})

        issues = []
        if asset["status"] != "ready":
            issues.append({"issue_type": "asset_status_warning", "severity": "medium"})
        if asset["clearance_m"] < asset["min_clearance_m"]:
            issues.append({"issue_type": "insufficient_clearance", "severity": "high"})

        gauge = asset["gauge"]
        if gauge is not None and not gauge["normal_min"] <= gauge["reading"] <= gauge["normal_max"]:
            issues.append({"issue_type": "abnormal_gauge_reading", "severity": "medium"})

        return _json(
            {
                "ok": True,
                "asset_id": asset_key,
                "zone_id": asset["zone_id"],
                "asset_type": asset["asset_type"],
                "status": asset["status"],
                "issues": issues,
            }
        )

    @skill
    def read_gauge(self, asset_id: str) -> str:
        """Read a deterministic gauge or status card for a SiteOps asset.

        Args:
            asset_id: Asset id with a gauge, for example COOLING_UNIT_1.
        """
        asset_key = _normalize_id(asset_id)
        asset = self._site_state["assets"].get(asset_key)
        if asset is None:
            return _json({"ok": False, "error": "unknown_asset", "asset_id": asset_id})

        gauge = asset["gauge"]
        if gauge is None:
            return _json({"ok": False, "error": "no_gauge", "asset_id": asset_key})

        in_range = gauge["normal_min"] <= gauge["reading"] <= gauge["normal_max"]
        return _json(
            {
                "ok": True,
                "asset_id": asset_key,
                "reading": gauge["reading"],
                "unit": gauge["unit"],
                "normal_min": gauge["normal_min"],
                "normal_max": gauge["normal_max"],
                "status": "normal" if in_range else "abnormal",
            }
        )

    @skill
    def check_clearance(self, asset_id: str) -> str:
        """Check whether an asset has enough physical clearance.

        Args:
            asset_id: Asset id such as COOLING_UNIT_1.
        """
        asset_key = _normalize_id(asset_id)
        asset = self._site_state["assets"].get(asset_key)
        if asset is None:
            return _json({"ok": False, "error": "unknown_asset", "asset_id": asset_id})

        clear = asset["clearance_m"] >= asset["min_clearance_m"]
        return _json(
            {
                "ok": True,
                "asset_id": asset_key,
                "clear": clear,
                "clearance_m": asset["clearance_m"],
                "min_clearance_m": asset["min_clearance_m"],
                "status": "clear" if clear else "blocked",
            }
        )

    @skill
    def detect_blocked_aisle(self, zone_id: str) -> str:
        """Detect whether a SiteOps zone has an aisle blockage.

        Args:
            zone_id: Semantic zone id such as COOLING_1.
        """
        zone_key = _normalize_id(zone_id)
        zone = self._site_state["zones"].get(zone_key)
        if zone is None:
            return _json({"ok": False, "error": "unknown_zone", "zone_id": zone_id})

        return _json(
            {
                "ok": True,
                "zone_id": zone_key,
                "blocked": zone["blocked"],
                "blockers": zone["blockers"],
            }
        )

    @skill
    def scan_receiving_manifest(self, zone_id: str) -> str:
        """Compare package detections against the receiving manifest for a zone.

        Args:
            zone_id: Zone id to reconcile against the manifest, for example INBOUND_DOCK.
        """
        zone_key = _normalize_id(zone_id)
        zone = self._site_state["zones"].get(zone_key)
        if zone is None:
            return _json({"ok": False, "error": "unknown_zone", "zone_id": zone_id})

        expected = [item["id"] for item in self._site_state["manifest"].get(zone_key, [])]
        detected = [
            detection["id"]
            for detection in zone["detections"]
            if detection.get("type") == "package"
        ]
        missing = sorted(set(expected) - set(detected))
        unexpected = sorted(set(detected) - set(expected))

        return _json(
            {
                "ok": True,
                "zone_id": zone_key,
                "expected_packages": expected,
                "detected_packages": detected,
                "missing_packages": missing,
                "unexpected_packages": unexpected,
                "status": "matched" if not missing and not unexpected else "mismatch",
            }
        )

    @skill
    def open_work_order(self, entity_id: str, issue_type: str) -> str:
        """Open or return an idempotent work order for a SiteOps issue.

        Args:
            entity_id: Asset, package, or zone id associated with the issue.
            issue_type: Issue type such as blocked_aisle, missing_package, or abnormal_gauge_reading.
        """
        entity_key = _normalize_id(entity_id)
        issue_key = issue_type.strip().lower().replace(" ", "_")
        work_order_id = _work_order_id(entity_key, issue_key)
        created = work_order_id not in self._work_orders

        self._work_orders.setdefault(
            work_order_id,
            {
                "work_order_id": work_order_id,
                "entity_id": entity_key,
                "issue_type": issue_key,
                "status": "open",
            },
        )

        payload = {"ok": True, "created": created, **self._work_orders[work_order_id]}
        return _json(payload)

    @skill
    def verify_work_order(self, work_order_id: str) -> str:
        """Revisit the work-order entity and verify whether the issue is closed.

        Args:
            work_order_id: Work order id returned by open_work_order.
        """
        order_key = _normalize_id(work_order_id).replace("_", "-")
        order = self._work_orders.get(order_key)
        if order is None:
            return _json(
                {"ok": False, "error": "unknown_work_order", "work_order_id": work_order_id}
            )

        still_active = self._issue_is_active(order["entity_id"], order["issue_type"])
        order["status"] = "verification_failed" if still_active else "verified_closed"
        return _json(
            {
                "ok": True,
                "work_order_id": order_key,
                "entity_id": order["entity_id"],
                "issue_type": order["issue_type"],
                "status": order["status"],
                "issue_still_active": still_active,
            }
        )

    @skill
    def nav_eval_report(self, run_id: str) -> str:
        """Return navigation and relocalization metrics for a SiteOps run.

        Args:
            run_id: Run id to summarize. Use dogops-demo for the deterministic fixture.
        """
        run_key = run_id.strip() or "dogops-demo"
        run = self._site_state["runs"].get(run_key, self._site_state["runs"]["dogops-demo"])
        reached = run["waypoints_reached"]
        attempted = run["waypoints_attempted"]
        return _json(
            {
                "ok": True,
                "run_id": run_key,
                "route": run["route"],
                "waypoints_attempted": attempted,
                "waypoints_reached": reached,
                "waypoint_success_rate": reached / attempted,
                "route_elapsed_s": run["route_elapsed_s"],
                "navigation_retries": run["navigation_retries"],
                "guided_interventions": run["guided_interventions"],
                "tag_reacquisition_success": run["tag_reacquisition_success"],
                "tag_reacquisition_time_s": run["tag_reacquisition_time_s"],
                "verification_revisit_success": run["verification_revisit_success"],
                "planned_distance_m": run["planned_distance_m"],
                "actual_distance_m": run["actual_distance_m"],
            }
        )

    def _issue_is_active(self, entity_id: str, issue_type: str) -> bool:
        if issue_type == "blocked_aisle":
            zone = self._site_state["zones"].get(entity_id)
            return bool(zone and zone["blocked"])

        if issue_type in {"insufficient_clearance", "blocked_clearance"}:
            asset = self._site_state["assets"].get(entity_id)
            return bool(asset and asset["clearance_m"] < asset["min_clearance_m"])

        if issue_type == "abnormal_gauge_reading":
            asset = self._site_state["assets"].get(entity_id)
            if not asset or asset["gauge"] is None:
                return False
            gauge = asset["gauge"]
            return not gauge["normal_min"] <= gauge["reading"] <= gauge["normal_max"]

        if issue_type == "missing_package":
            for zone_id, expected_items in self._site_state["manifest"].items():
                expected = {item["id"] for item in expected_items}
                detected = {
                    detection["id"]
                    for detection in self._site_state["zones"][zone_id]["detections"]
                    if detection.get("type") == "package"
                }
                if entity_id in expected:
                    return entity_id not in detected

        return False
