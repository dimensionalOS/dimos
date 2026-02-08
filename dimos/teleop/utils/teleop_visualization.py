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

from typing import TYPE_CHECKING

import rerun as rr

from dimos.dashboard.rerun_init import connect_rerun
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig
    from dimos.msgs.geometry_msgs import PoseStamped

logger = setup_logger()


def init_rerun_visualization(global_config: GlobalConfig | None = None) -> bool:
    """Initialize Rerun visualization connection."""
    if global_config is None:
        from dimos.core.global_config import GlobalConfig as GC

        global_config = GC()
    if not global_config.viewer_backend.startswith("rerun"):
        logger.debug(f"Skipping Rerun init: viewer_backend={global_config.viewer_backend}")
        return False

    try:
        connect_rerun(global_config=global_config)
        logger.info("Connected to Rerun for teleop visualization")
        return True
    except Exception as e:
        logger.warning(f"Failed to connect to Rerun: {e}")
        return False


def visualize_pose(pose_stamped: PoseStamped, controller_label: str) -> None:
    """Visualize controller absolute pose in Rerun."""
    try:
        rr.log(f"world/teleop/{controller_label}_controller", pose_stamped.to_rerun())  # type: ignore[no-untyped-call]
        rr.log(f"world/teleop/{controller_label}_controller/axes", rr.TransformAxes3D(0.10))  # type: ignore[attr-defined]
    except Exception as e:
        logger.debug(f"Failed to log {controller_label} controller to Rerun: {e}")


def visualize_buttons(
    controller_label: str,
    primary: bool = False,
    secondary: bool = False,
    grip: float = 0.0,
    trigger: float = 0.0,
) -> None:
    """Visualize button states in Rerun as scalar time series."""
    try:
        base_path = f"world/teleop/{controller_label}_controller"
        rr.log(f"{base_path}/primary", rr.Scalars(float(primary)))  # type: ignore[attr-defined]
        rr.log(f"{base_path}/secondary", rr.Scalars(float(secondary)))  # type: ignore[attr-defined]
        rr.log(f"{base_path}/grip", rr.Scalars(grip))  # type: ignore[attr-defined]
        rr.log(f"{base_path}/trigger", rr.Scalars(trigger))  # type: ignore[attr-defined]
    except Exception as e:
        logger.debug(f"Failed to log {controller_label} buttons to Rerun: {e}")


__all__ = ["init_rerun_visualization", "visualize_buttons", "visualize_pose"]
