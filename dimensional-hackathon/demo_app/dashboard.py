from __future__ import annotations

import asyncio
import base64
import json
import time
from pathlib import Path
from typing import Any, Awaitable, Callable

import cv2
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

_DETECTION_TTL_SEC = 1.5


def create_app(
    stream_fps: int,
    command_handlers: dict[str, Callable[[], Awaitable[Any]]] | None = None,
) -> tuple[FastAPI, dict]:
    app = FastAPI(title="Go2 Demo Dashboard")
    state: dict[str, Any] = {
        "waypoints": [],
        "visited": [],
        "robot_pose": None,
        "pose_history": [],
        "corridor_clear": True,
        "obstruction_distance_m": None,
        "obstruction_direction": "center",
        "obstruction_point_count": 0,
        "anomalies": [],
        "mode": "IDLE",
        "latest_frame": None,
        "latest_detections": [],
        "event_log": [],
        "planned_path": [],
    }

    static_dir = Path(__file__).parent / "static"
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

    @app.get("/", response_class=HTMLResponse)
    async def root() -> str:
        return (static_dir / "index.html").read_text()

    @app.get("/api/map")
    async def get_map() -> dict:
        return {
            "waypoints": state["waypoints"],
            "visited": state["visited"],
            "robot_pose": state["robot_pose"],
            "pose_history": state["pose_history"],
            "corridor_clear": state["corridor_clear"],
            "obstruction_distance_m": state["obstruction_distance_m"],
            "obstruction_direction": state["obstruction_direction"],
            "obstruction_point_count": state["obstruction_point_count"],
            "anomalies": state["anomalies"],
            "mode": state["mode"],
            "planned_path": state["planned_path"],
        }

    @app.post("/api/command/{name}")
    async def post_command(name: str) -> dict:
        if not command_handlers or name not in command_handlers:
            raise HTTPException(status_code=404, detail=f"Unknown command: {name}")
        try:
            result = await command_handlers[name]()
            state.setdefault("event_log", []).append(
                {"type": "command", "payload": {"name": name, "result": result}}
            )
            return {"ok": True, "command": name, "result": result}
        except Exception as e:
            state.setdefault("event_log", []).append(
                {"type": "command_error", "payload": {"name": name, "error": str(e)}}
            )
            raise HTTPException(status_code=500, detail=str(e)) from e

    @app.websocket("/ws/stream")
    async def ws_stream(ws: WebSocket) -> None:
        await ws.accept()
        interval = 1.0 / max(stream_fps, 1)
        try:
            while True:
                frame = state.get("latest_frame")
                if frame is not None:
                    rendered = _overlay_detections(frame, state)
                    _, buf = cv2.imencode(".jpg", rendered, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    await ws.send_text(base64.b64encode(buf).decode())
                await asyncio.sleep(interval)
        except WebSocketDisconnect:
            return

    @app.websocket("/ws/events")
    async def ws_events(ws: WebSocket) -> None:
        await ws.accept()
        last_seen = 0
        try:
            while True:
                events = state.get("event_log", [])
                if len(events) > last_seen:
                    for event in events[last_seen:]:
                        await ws.send_text(json.dumps(event))
                    last_seen = len(events)
                await asyncio.sleep(0.2)
        except WebSocketDisconnect:
            return

    return app, state


def _overlay_detections(frame, state) -> Any:
    detections = state.get("latest_detections") or []
    if not detections:
        return frame
    now = time.time()
    out = frame.copy()
    for detection in detections:
        if now - detection.get("timestamp", 0) > _DETECTION_TTL_SEC:
            continue
        bbox = detection.get("bbox")
        if not bbox or len(bbox) != 4:
            continue
        x1, y1, x2, y2 = (int(v) for v in bbox)
        label = f"{detection.get('class_name', '?')} {detection.get('confidence', 0.0):.2f}"
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        ly = max(0, y1 - 6)
        cv2.rectangle(out, (x1, ly - th - 4), (x1 + tw + 6, ly + 2), (0, 255, 255), -1)
        cv2.putText(out, label, (x1 + 3, ly - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
    return out
