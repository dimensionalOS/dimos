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

"""
WebSocket Visualization Module for Dimos navigation and mapping.

This module provides a WebSocket data server for real-time visualization.
The frontend is served from a separate HTML file.
"""

import asyncio
from pathlib import Path as FilePath
import threading
import time
from typing import Any
import webbrowser

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable
import socketio  # type: ignore[import-untyped]
from starlette.applications import Starlette
from starlette.responses import FileResponse, HTMLResponse, JSONResponse, RedirectResponse, Response
from starlette.routing import Route
import uvicorn

from dimos.utils.data import get_data

# Path to the frontend HTML templates and command-center build
_TEMPLATES_DIR = FilePath(__file__).parent.parent / "templates"
_DASHBOARD_HTML = _TEMPLATES_DIR / "rerun_dashboard.html"
_COMMAND_CENTER_DIR = (
    FilePath(__file__).parent.parent / "command-center-extension" / "dist-standalone"
)
_WORLD_MODEL_PANEL_HTML = """<!DOCTYPE html>
<html>
<head>
    <title>World Model</title>
    <style>
        * { box-sizing: border-box; }
        html, body { margin: 0; width: 100%; height: 100%; overflow: hidden; }
        body {
            background: #0b0d10;
            color: #e7edf2;
            font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
            font-size: 12px;
        }
        .shell { height: 100%; display: grid; grid-template-rows: auto 1fr; }
        header {
            display: grid;
            grid-template-columns: auto 1fr auto auto;
            gap: 12px;
            align-items: center;
            padding: 10px 12px;
            border-bottom: 1px solid #242a31;
            background: #11161b;
        }
        h1 { margin: 0; font-size: 13px; font-weight: 700; letter-spacing: 0; }
        .meta { color: #92a0ac; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
        .pill {
            min-width: 56px;
            padding: 4px 8px;
            border: 1px solid #38424c;
            border-radius: 8px;
            text-align: center;
            font-weight: 700;
        }
        .risk-low { color: #6ee7a8; border-color: #24764a; }
        .risk-medium { color: #f0c867; border-color: #79601f; }
        .risk-high { color: #ff7c7c; border-color: #7a2929; }
        main {
            min-height: 0;
            display: grid;
            grid-template-columns: 0.8fr 1.2fr 1fr;
            gap: 0;
            overflow: hidden;
        }
        section {
            min-width: 0;
            min-height: 0;
            padding: 10px 12px;
            border-right: 1px solid #20262d;
            overflow: auto;
        }
        section:last-child { border-right: 0; }
        h2 { margin: 0 0 8px; color: #9fb2c3; font-size: 11px; font-weight: 700; letter-spacing: 0; }
        dl { display: grid; grid-template-columns: 92px 1fr; gap: 5px 8px; margin: 0; }
        dt { color: #778691; }
        dd { margin: 0; min-width: 0; overflow-wrap: anywhere; }
        ul { list-style: none; padding: 0; margin: 0; display: grid; gap: 6px; }
        li { padding-bottom: 6px; border-bottom: 1px solid #1b2026; overflow-wrap: anywhere; }
        li:last-child { border-bottom: 0; }
        code { color: #d8dee9; white-space: pre-wrap; overflow-wrap: anywhere; }
        .empty { color: #687683; }
    </style>
</head>
<body>
    <div class="shell">
        <header>
            <h1>World Model</h1>
            <div class="meta" id="wm-action">no action</div>
            <div class="pill" id="wm-risk">unknown</div>
            <div class="meta" id="wm-score">score --</div>
        </header>
        <main>
            <section>
                <h2>Prediction</h2>
                <dl>
                    <dt>success</dt><dd id="wm-success">--</dd>
                    <dt>confidence</dt><dd id="wm-confidence">--</dd>
                    <dt>samples</dt><dd id="wm-samples">0</dd>
                    <dt>event</dt><dd id="wm-event">--</dd>
                </dl>
            </section>
            <section>
                <h2>Future State Delta</h2>
                <code id="wm-delta" class="empty">waiting for state</code>
            </section>
            <section>
                <h2>Causal</h2>
                <ul id="wm-causal"><li class="empty">no causal evidence</li></ul>
            </section>
        </main>
    </div>
    <script>
        const riskEl = document.getElementById('wm-risk');
        const scoreEl = document.getElementById('wm-score');
        const actionEl = document.getElementById('wm-action');
        const successEl = document.getElementById('wm-success');
        const confidenceEl = document.getElementById('wm-confidence');
        const samplesEl = document.getElementById('wm-samples');
        const eventEl = document.getElementById('wm-event');
        const deltaEl = document.getElementById('wm-delta');
        const causalEl = document.getElementById('wm-causal');

        function text(value, fallback = '--') {
            if (value === null || value === undefined || value === '') return fallback;
            return String(value);
        }

        function renderList(items) {
            causalEl.replaceChildren();
            if (!items.length) {
                const empty = document.createElement('li');
                empty.className = 'empty';
                empty.textContent = 'no causal evidence';
                causalEl.appendChild(empty);
                return;
            }
            for (const item of items.slice(0, 6)) {
                const li = document.createElement('li');
                li.textContent = item;
                causalEl.appendChild(li);
            }
        }

        function render(state) {
            const prediction = state.prediction || {};
            const action = state.action || {};
            const modelState = state.model_state || {};
            const risk = text(prediction.risk, 'unknown').toLowerCase();
            riskEl.textContent = risk;
            riskEl.className = `pill risk-${risk}`;
            scoreEl.textContent = `score ${text(prediction.score, '--')}`;
            const actionName = text(action.skill_name || action.action, 'no action');
            const actionLabel = text(state.action_label, actionName === 'no action' ? '' : 'Action');
            actionEl.textContent = actionLabel ? `${actionLabel}: ${actionName}` : actionName;
            successEl.textContent = text(prediction.predicted_success);
            confidenceEl.textContent = text(prediction.confidence);
            samplesEl.textContent = text(modelState.sample_count, '0');
            eventEl.textContent = text(state.event);
            deltaEl.className = '';
            deltaEl.textContent = JSON.stringify(prediction.predicted_state_delta || {}, null, 2);

            const scm = prediction.structural_causal_model || {};
            const attribution = prediction.causal_attribution || {};
            const interventions = state.recent_interventions || [];
            const causalItems = [
                ...(prediction.reasons || []),
                ...((scm.counterfactuals || []).map((item) => item.intervention)),
                ...((attribution.risk_factors || []).map((item) => `${item.feature}: ${item.effect_on_success}`)),
                ...(interventions.map((item) => `${item.intervention_name} -> ${item.target_variable}`)),
            ];
            renderList(causalItems);
        }

        async function refresh() {
            try {
                const response = await fetch('/api/world-model-state', { cache: 'no-store' });
                render(await response.json());
            } catch (error) {
                renderList([String(error)]);
            }
        }

        refresh();
        setInterval(refresh, 700);
    </script>
</body>
</html>"""

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.models import LatLon
from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

from .optimized_costmap import OptimizedCostmapEncoder

logger = setup_logger()

_browser_open_lock = threading.Lock()
_browser_opened = False


class WebsocketConfig(ModuleConfig):
    port: int = 7779


class WebsocketVisModule(Module):
    """
    WebSocket-based visualization module for real-time navigation data.

    This module provides a web interface for visualizing:
    - Robot position and orientation
    - Navigation paths
    - Costmaps
    - Interactive goal setting via mouse clicks

    Inputs:
        - robot_pose: Current robot position
        - path: Navigation path
        - global_costmap: Global costmap for visualization

    Outputs:
        - click_goal: Goal position from user clicks
    """

    config: WebsocketConfig

    # LCM inputs
    odom: In[PoseStamped]
    gps_location: In[LatLon]
    path: In[Path]
    global_costmap: In[OccupancyGrid]

    # LCM outputs
    goal_request: Out[PoseStamped]
    gps_goal: Out[LatLon]
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]
    tele_cmd_vel: Out[Twist]
    movecmd_stamped: Out[TwistStamped]

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the WebSocket visualization module.

        Args:
            port: Port to run the web server on
            cfg: Optional global config for viewer settings
        """
        super().__init__(**kwargs)
        self._uvicorn_server_thread: threading.Thread | None = None
        self.sio: socketio.AsyncServer | None = None
        self.app = None
        self._broadcast_loop = None
        self._broadcast_thread = None
        self._uvicorn_server: uvicorn.Server | None = None

        self.vis_state = {}  # type: ignore[var-annotated]
        self.state_lock = threading.Lock()
        self.costmap_encoder = OptimizedCostmapEncoder(chunk_size=64)

        # Track GPS goal points for visualization
        self.gps_goal_points: list[dict[str, float]] = []
        logger.info(
            f"WebSocket visualization module initialized on port {self.config.port}, GPS goal tracking enabled"
        )

    def _start_broadcast_loop(self) -> None:
        def websocket_vis_loop() -> None:
            self._broadcast_loop = asyncio.new_event_loop()  # type: ignore[assignment]
            asyncio.set_event_loop(self._broadcast_loop)
            try:
                self._broadcast_loop.run_forever()  # type: ignore[attr-defined]
            except Exception as e:
                logger.error(f"Broadcast loop error: {e}")
            finally:
                self._broadcast_loop.close()  # type: ignore[attr-defined]

        self._broadcast_thread = threading.Thread(target=websocket_vis_loop, daemon=True)  # type: ignore[assignment]
        self._broadcast_thread.start()  # type: ignore[attr-defined]

    @rpc
    def start(self) -> None:
        super().start()

        self._create_server()

        self._start_broadcast_loop()

        self._uvicorn_server_thread = threading.Thread(target=self._run_uvicorn_server, daemon=True)
        self._uvicorn_server_thread.start()

        # Only auto-open when the user chose web-based viewing.
        if self.config.g.viewer == "rerun" and self.config.g.rerun_open in ("web", "both"):
            url = f"http://localhost:{self.config.port}/"
            logger.info(f"Dimensional Command Center: {url}")

            global _browser_opened
            with _browser_open_lock:
                if not _browser_opened:
                    try:
                        webbrowser.open_new_tab(url)
                        _browser_opened = True
                    except Exception as e:
                        logger.debug(f"Failed to open browser: {e}")

        try:
            unsub = self.odom.subscribe(self._on_robot_pose)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.gps_location.subscribe(self._on_gps_location)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.path.subscribe(self._on_path)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.global_costmap.subscribe(self._on_global_costmap)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

    @rpc
    def stop(self) -> None:
        if getattr(self, "_ws_stopped", False):
            return
        self._ws_stopped = True

        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True

        if self.sio and self._broadcast_loop and not self._broadcast_loop.is_closed():

            async def _disconnect_all() -> None:
                await self.sio.disconnect()

            asyncio.run_coroutine_threadsafe(_disconnect_all(), self._broadcast_loop)

        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            self._broadcast_loop.call_soon_threadsafe(self._broadcast_loop.stop)

        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        if self._uvicorn_server_thread and self._uvicorn_server_thread.is_alive():
            self._uvicorn_server_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()

    @rpc
    def set_gps_travel_goal_points(self, points: list[LatLon]) -> None:
        json_points = [{"lat": x.lat, "lon": x.lon} for x in points]
        self.vis_state["gps_travel_goal_points"] = json_points
        self._emit("gps_travel_goal_points", json_points)

    @rpc
    def set_world_model_state(self, state: dict[str, Any]) -> dict[str, Any]:
        """Publish a compact predictive world-model state to dashboard clients."""
        dashboard_state = _world_model_dashboard_state(state)
        with self.state_lock:
            self.vis_state["world_model_state"] = dashboard_state
        self._emit("world_model_state", dashboard_state)
        return _world_model_dashboard_summary(dashboard_state)

    @rpc
    def get_world_model_state(self) -> dict[str, Any]:
        """Return the latest predictive world-model state for polling dashboards."""
        with self.state_lock:
            state = self.vis_state.get("world_model_state")
        if isinstance(state, dict):
            return state
        return {"available": False, "prediction": {}, "recent_interventions": []}

    def _create_server(self) -> None:
        # Create SocketIO server
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

        async def serve_index(request):  # type: ignore[no-untyped-def]
            """Serve appropriate HTML based on viewer mode."""
            if not (
                self.config.g.viewer == "rerun" and self.config.g.rerun_open in ("web", "both")
            ):
                return RedirectResponse(url="/command-center")
            return FileResponse(_DASHBOARD_HTML, media_type="text/html")

        async def serve_command_center(request):  # type: ignore[no-untyped-def]
            """Serve the command center 2D visualization (built React app)."""
            index_file = get_data("command_center.html")
            if index_file.exists():
                return FileResponse(index_file, media_type="text/html")
            else:
                return Response(
                    content="Command center not built. Run: cd dimos/web/command-center-extension && npm install && npm run build:standalone",
                    status_code=503,
                    media_type="text/plain",
                )

        async def serve_world_model_panel(request):  # type: ignore[no-untyped-def]
            """Serve the polling world-model panel shown in the dashboard shell."""
            return HTMLResponse(_WORLD_MODEL_PANEL_HTML)

        async def serve_world_model_state(request):  # type: ignore[no-untyped-def]
            """Serve the latest world-model dashboard state for polling clients."""
            return JSONResponse(self.get_world_model_state())

        routes = [
            Route("/", serve_index),
            Route("/command-center", serve_command_center),
            Route("/world-model", serve_world_model_panel),
            Route("/api/world-model-state", serve_world_model_state),
        ]

        starlette_app = Starlette(routes=routes)

        self.app = socketio.ASGIApp(self.sio, starlette_app)

        # Register SocketIO event handlers
        @self.sio.event  # type: ignore[untyped-decorator]
        async def connect(sid, environ) -> None:  # type: ignore[no-untyped-def]
            with self.state_lock:
                current_state = dict(self.vis_state)

            # Include GPS goal points in the initial state
            if self.gps_goal_points:
                current_state["gps_travel_goal_points"] = self.gps_goal_points

            # Force full costmap update on new connection
            self.costmap_encoder.last_full_grid = None

            await self.sio.emit("full_state", current_state, room=sid)  # type: ignore[union-attr]
            logger.info(
                f"Client {sid} connected, sent state with {len(self.gps_goal_points)} GPS goal points"
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def click(sid, position) -> None:  # type: ignore[no-untyped-def]
            goal = PoseStamped(
                position=(position[0], position[1], 0),
                orientation=(0, 0, 0, 1),  # Default orientation
                frame_id="world",
            )
            self.goal_request.publish(goal)
            logger.info(
                "Click goal published", x=round(goal.position.x, 3), y=round(goal.position.y, 3)
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def gps_goal(sid: str, goal: dict[str, float]) -> None:
            logger.info(f"Received GPS goal: {goal}")

            # Publish the goal to LCM
            self.gps_goal.publish(LatLon(lat=goal["lat"], lon=goal["lon"]))

            # Add to goal points list for visualization
            self.gps_goal_points.append(goal)
            logger.info(f"Added GPS goal to list. Total goals: {len(self.gps_goal_points)}")

            # Emit updated goal points back to all connected clients
            if self.sio is not None:
                await self.sio.emit("gps_travel_goal_points", self.gps_goal_points)
            logger.debug(
                f"Emitted gps_travel_goal_points with {len(self.gps_goal_points)} points: {self.gps_goal_points}"
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def start_explore(sid: str) -> None:
            logger.info("Starting exploration")
            self.explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[untyped-decorator]
        async def stop_explore(sid) -> None:  # type: ignore[no-untyped-def]
            logger.info("Stopping exploration")
            self.stop_explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[untyped-decorator]
        async def clear_gps_goals(sid: str) -> None:
            logger.info("Clearing all GPS goal points")
            self.gps_goal_points.clear()
            if self.sio is not None:
                await self.sio.emit("gps_travel_goal_points", self.gps_goal_points)
            logger.info("GPS goal points cleared and updated clients")

        @self.sio.event  # type: ignore[untyped-decorator]
        async def move_command(sid: str, data: dict[str, Any]) -> None:
            # Publish Twist if transport is configured
            if self.tele_cmd_vel and self.tele_cmd_vel.transport:
                twist = Twist(
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.tele_cmd_vel.publish(twist)

            # Publish TwistStamped if transport is configured
            if self.movecmd_stamped and self.movecmd_stamped.transport:
                twist_stamped = TwistStamped(
                    ts=time.time(),
                    frame_id="base_link",
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.movecmd_stamped.publish(twist_stamped)

    def _run_uvicorn_server(self) -> None:
        config = uvicorn.Config(
            self.app,  # type: ignore[arg-type]
            host=global_config.listen_host,
            port=self.config.port,
            log_level="error",  # Reduce verbosity
        )
        self._uvicorn_server = uvicorn.Server(config)
        self._uvicorn_server.run()

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        pose_data = {"type": "vector", "c": [msg.position.x, msg.position.y, msg.position.z]}
        self.vis_state["robot_pose"] = pose_data
        self._emit("robot_pose", pose_data)

    def _on_gps_location(self, msg: LatLon) -> None:
        pose_data = {"lat": msg.lat, "lon": msg.lon}
        self.vis_state["gps_location"] = pose_data
        self._emit("gps_location", pose_data)

    def _on_path(self, msg: Path) -> None:
        points = [[pose.position.x, pose.position.y] for pose in msg.poses]
        path_data = {"type": "path", "points": points}
        self.vis_state["path"] = path_data
        self._emit("path", path_data)

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        costmap_data = self._process_costmap(msg)
        self.vis_state["costmap"] = costmap_data
        self._emit("costmap", costmap_data)

    def _process_costmap(self, costmap: OccupancyGrid) -> dict[str, Any]:
        """Convert OccupancyGrid to visualization format."""
        costmap = gradient(simple_inflate(costmap, 0.1), max_distance=1.0)
        grid_data = self.costmap_encoder.encode_costmap(costmap.grid)

        return {
            "type": "costmap",
            "grid": grid_data,
            "origin": {
                "type": "vector",
                "c": [costmap.origin.position.x, costmap.origin.position.y, 0],
            },
            "resolution": costmap.resolution,
            "origin_theta": 0,  # Assuming no rotation for now
        }

    def _emit(self, event: str, data: Any) -> None:
        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.sio.emit(event, data), self._broadcast_loop)


def _world_model_dashboard_state(state: dict[str, Any]) -> dict[str, Any]:
    dashboard_state = _bounded_jsonable(state)
    if not isinstance(dashboard_state, dict):
        dashboard_state = {}
    dashboard_state["available"] = True
    dashboard_state.setdefault("updated_at", round(time.time(), 3))
    return dashboard_state


def _world_model_dashboard_summary(state: dict[str, Any]) -> dict[str, Any]:
    prediction = state.get("prediction") if isinstance(state.get("prediction"), dict) else {}
    model_state = state.get("model_state") if isinstance(state.get("model_state"), dict) else {}
    recent_interventions = (
        state.get("recent_interventions")
        if isinstance(state.get("recent_interventions"), list)
        else []
    )
    return {
        "available": bool(state.get("available")),
        "risk": str(prediction.get("risk") or ""),
        "score": prediction.get("score"),
        "predicted_success": prediction.get("predicted_success"),
        "transition_count": len(state.get("recent_transitions") or []),
        "intervention_count": len(recent_interventions),
        "model_samples": int(model_state.get("sample_count") or 0),
    }


def _bounded_jsonable(value: Any, max_string_length: int = 500) -> Any:
    if value is None or isinstance(value, bool | int | float):
        return value
    if isinstance(value, str):
        return value[:max_string_length]
    if isinstance(value, dict):
        return {
            str(key): _bounded_jsonable(item, max_string_length)
            for key, item in value.items()
            if not str(key).startswith("_")
        }
    if isinstance(value, list | tuple):
        return [_bounded_jsonable(item, max_string_length) for item in value[:10]]
    return str(value)[:max_string_length]
