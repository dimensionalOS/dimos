#!/usr/bin/env python3
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

"""Teleop visualization utilities for Rerun."""

from __future__ import annotations

import time

from dimos.msgs.geometry_msgs import Pose, PoseStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    import rerun as rr

    from dimos.dashboard.rerun_init import connect_rerun

    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False


def init_rerun_visualization() -> bool:
    """Initialize Rerun visualization connection."""
    if not RERUN_AVAILABLE:
        return False

    try:
        connect_rerun()
        logger.info("Connected to Rerun for teleop visualization")
        return True
    except Exception as e:
        logger.warning(f"Failed to connect to Rerun: {e}")
        return False


def visualize_controller_pose(controller_pose: Pose, controller_label: str) -> None:
    """Visualize controller absolute pose in Rerun.

    Args:
        controller_pose: The controller's current pose.
        controller_label: Label for the controller (e.g., "left_vr").
    """
    if not RERUN_AVAILABLE:
        return

    try:
        controller_pose_stamped = PoseStamped(
            ts=time.time(),
            frame_id=f"world/teleop/{controller_label}_controller",
            position=controller_pose.position,
            orientation=controller_pose.orientation,
        )
        rr.log(
            f"world/teleop/{controller_label}_controller",
            controller_pose_stamped.to_rerun(),  # type: ignore[no-untyped-call]
        )
        # Log 3D axes to visualize controller orientation (X=red, Y=green, Z=blue)
        rr.log(
            f"world/teleop/{controller_label}_controller/axes",
            rr.TransformAxes3D(0.15),  # type: ignore[attr-defined]
        )
    except Exception as e:
        logger.debug(f"Failed to log {controller_label} controller to Rerun: {e}")


def visualize_trigger_value(trigger_value: float, controller_label: str) -> None:
    """Visualize trigger value in Rerun as a scalar time series.

    Args:
        trigger_value: Trigger value (0.0-1.0).
        controller_label: Label for the controller (e.g., "left_vr").
    """
    if not RERUN_AVAILABLE:
        return

    try:
        rr.log(
            f"world/teleop/{controller_label}_controller/trigger",
            rr.Scalars(trigger_value),  # type: ignore[attr-defined]
        )
    except Exception as e:
        logger.debug(f"Failed to log {controller_label} trigger to Rerun: {e}")
