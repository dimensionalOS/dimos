from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any


LIVE_TOPICS = {
    "global_costmap": "/global_costmap",
    "navigation_costmap": "/navigation_costmap",
    "odom": "/odom",
    "path": "/path",
    "target": "/target",
    "goal_request": "/goal_request",
    "clicked_point": "/clicked_point",
}
LIVE_TOPIC_MAX_AGE_S = 5.0


class DogOpsLiveMapAdapter:
    """Bridge DimOS navigation topics into the DogOps mission-map payload."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._started = False
        self._error = ""
        self._unsubscribers: list[Any] = []
        self._transports: list[Any] = []
        self._latest: dict[str, tuple[float, Any]] = {}

    def snapshot(self) -> dict[str, Any]:
        self.start()
        with self._lock:
            recorded = dict(self._latest)
            error = self._error
        now = time.time()
        latest = {
            name: item
            for name, item in recorded.items()
            if now - item[0] <= LIVE_TOPIC_MAX_AGE_S
        }
        topics = {
            name: {
                "topic": topic,
                "received": name in latest,
                "age_s": round(now - recorded[name][0], 3) if name in recorded else None,
                "stale": name in recorded and name not in latest,
            }
            for name, topic in LIVE_TOPICS.items()
        }
        costmap_msg = _latest_first(latest, "navigation_costmap", "global_costmap")
        path_msg = _latest_value(latest, "path")
        odom_msg = _latest_value(latest, "odom")
        target_msg = _latest_first(latest, "target", "goal_request", "clicked_point")
        path = _path_to_points(path_msg) if path_msg is not None else []
        ok = any(item["received"] for item in topics.values())
        return {
            "ok": ok,
            "source": "DimOS live LCM topics",
            "status": "receiving" if ok else "waiting_for_topics",
            "error": error,
            "topics": topics,
            "costmap": _grid_to_costmap(costmap_msg) if costmap_msg is not None else None,
            "path": path,
            "route": _path_to_route(path),
            "robot_pose": _pose_to_map_pose(odom_msg, source="odom") if odom_msg is not None else None,
            "target": _pose_to_map_pose(target_msg, source="target") if target_msg is not None else None,
        }

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._started = True
        try:
            LCMTransport, OccupancyGrid, Path, PoseStamped, PointStamped = _import_dimos_topic_types()
        except Exception as exc:
            with self._lock:
                self._error = (
                    "DimOS topic imports unavailable in this Python environment. "
                    f"Run from the full DimOS checkout/env or install its deps. {exc}"
                )
            return

        specs = {
            "global_costmap": (LIVE_TOPICS["global_costmap"], OccupancyGrid),
            "navigation_costmap": (LIVE_TOPICS["navigation_costmap"], OccupancyGrid),
            "odom": (LIVE_TOPICS["odom"], PoseStamped),
            "path": (LIVE_TOPICS["path"], Path),
            "target": (LIVE_TOPICS["target"], PoseStamped),
            "goal_request": (LIVE_TOPICS["goal_request"], PoseStamped),
            "clicked_point": (LIVE_TOPICS["clicked_point"], PointStamped),
        }
        for name, (topic, msg_type) in specs.items():
            try:
                transport = LCMTransport(topic, msg_type)
                unsubscribe = transport.subscribe(lambda msg, item=name: self._record(item, msg))
                self._unsubscribers.append(unsubscribe)
                self._transports.append(transport)
            except Exception as exc:
                with self._lock:
                    self._error = f"Failed subscribing to {topic}: {exc}"

    def stop(self) -> None:
        with self._lock:
            unsubscribers = list(self._unsubscribers)
            transports = list(self._transports)
            self._unsubscribers.clear()
            self._transports.clear()
            self._latest.clear()
            self._started = False
        for unsubscribe in unsubscribers:
            try:
                unsubscribe()
            except Exception:
                pass
        for transport in transports:
            try:
                transport.stop()
            except Exception:
                pass

    def _record(self, name: str, msg: Any) -> None:
        with self._lock:
            self._latest[name] = (time.time(), msg)


def _import_dimos_topic_types() -> tuple[Any, Any, Any, Any, Any]:
    try:
        from dimos.core.transport import LCMTransport
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
        from dimos.msgs.nav_msgs.Path import Path
    except ModuleNotFoundError:
        _extend_dimos_package_path()
        from dimos.core.transport import LCMTransport
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
        from dimos.msgs.nav_msgs.Path import Path
    return LCMTransport, OccupancyGrid, Path, PoseStamped, PointStamped


def _extend_dimos_package_path() -> None:
    root = os.environ.get("DIMOS_ROOT")
    if not root:
        return
    dimos_root = Path(root).expanduser()
    package_root = dimos_root / "dimos"
    if not package_root.exists():
        return
    if str(dimos_root) not in sys.path:
        sys.path.append(str(dimos_root))
    import dimos

    package_path = getattr(dimos, "__path__", None)
    if package_path is not None and str(package_root) not in package_path:
        package_path.append(str(package_root))


def _latest_value(latest: dict[str, tuple[float, Any]], name: str) -> Any | None:
    item = latest.get(name)
    return item[1] if item is not None else None


def _latest_first(latest: dict[str, tuple[float, Any]], *names: str) -> Any | None:
    available = [latest[name] for name in names if name in latest]
    if not available:
        return None
    return max(available, key=lambda item: item[0])[1]


def _grid_to_costmap(msg: Any, *, max_columns: int = 48, max_rows: int = 32) -> dict[str, Any]:
    width = int(getattr(msg, "width", 0) or getattr(getattr(msg, "info", None), "width", 0) or 0)
    height = int(getattr(msg, "height", 0) or getattr(getattr(msg, "info", None), "height", 0) or 0)
    resolution = float(
        getattr(msg, "resolution", 0.05)
        or getattr(getattr(msg, "info", None), "resolution", 0.05)
        or 0.05
    )
    origin = getattr(getattr(msg, "info", None), "origin", None) or getattr(msg, "origin", None)
    origin_x = float(getattr(getattr(origin, "position", None), "x", 0.0) or 0.0)
    origin_y = float(getattr(getattr(origin, "position", None), "y", 0.0) or 0.0)
    grid = getattr(msg, "grid", None)
    columns = min(max_columns, width) if width > 0 else 0
    rows = min(max_rows, height) if height > 0 else 0
    if grid is None or width <= 0 or height <= 0 or columns <= 0 or rows <= 0:
        return {"source": "DimOS live costmap", "columns": 0, "rows": 0, "cells": []}

    cells: list[dict[str, float]] = []
    for row in range(rows):
        y0 = math.floor(row * height / rows)
        y1 = math.floor((row + 1) * height / rows)
        for column in range(columns):
            x0 = math.floor(column * width / columns)
            x1 = math.floor((column + 1) * width / columns)
            cells.append(
                {
                    "x": origin_x + x0 * resolution,
                    "y": origin_y + y0 * resolution,
                    "width": (x1 - x0) * resolution,
                    "height": (y1 - y0) * resolution,
                    "cost": _block_cost(grid, x0, x1, y0, y1),
                }
            )
    return {
        "source": "DimOS live costmap",
        "columns": columns,
        "rows": rows,
        "resolution_m": resolution,
        "cells": cells,
    }


def _block_cost(grid: Any, x0: int, x1: int, y0: int, y1: int) -> float:
    best = 0.0
    for y in range(y0, y1):
        for x in range(x0, x1):
            try:
                value = float(grid[y][x])
            except Exception:
                value = -1.0
            if value < 0:
                continue
            best = max(best, min(1.0, value / 100.0))
    return best


def _path_to_points(msg: Any) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    for pose in getattr(msg, "poses", []) or []:
        point = _pose_to_map_pose(pose, source="path")
        if point is not None:
            points.append(point)
    return points


def _path_to_route(path: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "target_id": f"LIVE-PATH-{index + 1:03d}",
            "x": point["x"],
            "y": point["y"],
            "success": True,
            "guided": False,
            "retries": 0,
            "note": "DimOS planner path",
        }
        for index, point in enumerate(path)
    ]


def _pose_to_map_pose(msg: Any, *, source: str) -> dict[str, Any] | None:
    if msg is None:
        return None
    x = getattr(msg, "x", None)
    y = getattr(msg, "y", None)
    if x is None or y is None:
        position = getattr(msg, "position", None)
        x = getattr(position, "x", None)
        y = getattr(position, "y", None)
    if x is None or y is None:
        return None
    yaw = getattr(msg, "yaw", None)
    return {
        "x": float(x),
        "y": float(y),
        "theta_deg": math.degrees(float(yaw)) if yaw is not None else None,
        "source": source,
    }
