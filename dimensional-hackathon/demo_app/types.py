from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class Waypoint:
    id: str
    pos: tuple[float, float, float]
    yaw_deg: float


@dataclass(frozen=True)
class Detection:
    bbox: tuple[int, int, int, int]
    class_name: str
    confidence: float
    frame: np.ndarray
    timestamp: float


@dataclass(frozen=True)
class AisleObservation:
    corridor_clear: bool
    obstruction_distance_m: float | None
    obstruction_direction: str
    obstruction_point_count: int
    occupied_cell_count: int
    obstruction_center_xy: tuple[float, float] | None


@dataclass(frozen=True)
class AlertEvent:
    bbox: tuple[int, int, int, int] | None
    class_name: str
    confidence: float
    frame: np.ndarray
    robot_pose: tuple[float, float]
    nearest_waypoint: str
    timestamp: float
    obstruction_distance_m: float | None = None
    obstruction_direction: str = ""
    obstruction_point_count: int = 0
    evidence_detections: list[Detection] | None = None
