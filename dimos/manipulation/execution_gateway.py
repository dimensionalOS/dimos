# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").
"""Coordinator RPC conversion seam for the manipulation execution runtime."""

from __future__ import annotations

from typing import Any

from dimos.manipulation.execution_models import Outcome
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


class ControlCoordinatorGateway:
    """Convert coordinator RPC requests and responses to normalized outcomes."""

    def __init__(self, client: Any) -> None:
        self._client = client

    def _invoke(self, task: str, method: str, kwargs: dict[str, Any]) -> Any:
        return self._client.task_invoke(task, method, kwargs)

    def execute(self, task_name: str, request: Any) -> Outcome:
        try:
            value = self._invoke(
                task_name,
                "execute",
                request if isinstance(request, dict) else {"trajectory": request},
            )
            return (
                Outcome.ACCEPTED
                if value is True
                else Outcome.REJECTED
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def cancel(self, task_name: str) -> Outcome:
        try:
            value = self._invoke(task_name, "cancel", {})
            return (
                Outcome.CANCELLED
                if value is True
                else Outcome.INACTIVE
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def status(self, task_name: str) -> Outcome:
        try:
            value = self._invoke(task_name, "get_state", {})
            value = getattr(value, "state", value)
            state = TrajectoryState(value)
        except (TypeError, ValueError, KeyError):
            return Outcome.UNKNOWN
        except Exception:
            return Outcome.UNKNOWN
        return {
            TrajectoryState.IDLE: Outcome.INACTIVE,
            TrajectoryState.EXECUTING: Outcome.RUNNING,
            TrajectoryState.COMPLETED: Outcome.COMPLETED,
            TrajectoryState.ABORTED: Outcome.CANCELLED,
            TrajectoryState.FAULT: Outcome.FAILED,
        }[state]

    def reset(self, task_name: str) -> Outcome:
        try:
            value = self._invoke(task_name, "reset", {})
            return (
                Outcome.INACTIVE
                if value is True
                else Outcome.RUNNING
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome:
        try:
            value = self._client.set_gripper_position(hardware_id, position)
            return (
                Outcome.ACCEPTED
                if value is True
                else Outcome.REJECTED
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def get_gripper_position(self, hardware_id: str) -> float | None:
        try:
            value = self._client.get_gripper_position(hardware_id)
            return float(value) if isinstance(value, (int, float)) else None
        except Exception:
            return None

    def stop(self) -> None:
        stop = getattr(self._client, "stop_rpc_client", None)
        if callable(stop):
            stop()
