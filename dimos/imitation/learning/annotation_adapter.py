"""Frozen annotation selections for the existing synchronized dataprep primitives."""

from __future__ import annotations

from typing import Any

from dimos.imitation.learning.protocol import LearningError


def preflight(snapshots: list[dict[str, Any]]) -> None:
    if not snapshots:
        raise LearningError(422, "empty_selection", "At least one recording is required")
    schemas = []
    for snapshot in snapshots:
        schema = snapshot["capture"]["schema"]
        semantics = schema.get("metadata", {}).get("gripperSemantics", {})
        if semantics.get("gripperCommandRecorded") is False:
            raise LearningError(
                422,
                "gripper_command_missing",
                "Command-action dataset blocked: gripperCommandRecorded=false. Measured closure is not a command; obtain a correctly recorded source.",
            )
        if not schema.get("action") or not schema.get("observation"):
            raise LearningError(
                422, "schema_incompatible", "Observation and command action features required"
            )
        schemas.append(
            {
                key: schema.get(key)
                for key in ("observation", "action", "sync", "quality", "robot_type")
            }
        )
        selected = [ep for ep in snapshot["annotation"]["episodes"] if ep["included"]]
        if not selected or any(
            not ep.get("instruction") or not ep["instruction"].strip() for ep in selected
        ):
            raise LearningError(
                422, "empty_selection", "Included episodes with instructions required"
            )
    if any(schema != schemas[0] for schema in schemas[1:]):
        raise LearningError(
            422,
            "schema_incompatible",
            "All selected recordings must have identical feature and synchronization schemas",
        )
