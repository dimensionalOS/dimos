# Copyright 2026 Dimensional Inc.
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

import asyncio
from queue import Queue
import threading
import time
from typing import Any

from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse

from dimos.agents.annotation import skill
from dimos.agents.skills.speak_skill_spec import SpeakSkillSpec
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.path_utils import get_project_root
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()


class BrowserNotificationConfig(ModuleConfig):
    server_port: int = 8450


AlertPayload = dict[str, Any]
ClientQueue = Queue[AlertPayload | None]


class BrowserNotificationSkill(Module):
    """Expose a browser notification page and a skill for robot-triggered alerts."""

    config: BrowserNotificationConfig
    _speaker: SpeakSkillSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._latest_alert: AlertPayload | None = None
        self._next_alert_id = 0
        self._client_queues: set[ClientQueue] = set()
        self._web_server = RobotWebInterface(port=self.config.server_port)
        self._web_server_thread: threading.Thread | None = None
        self._setup_routes()

    @rpc
    def start(self) -> None:
        super().start()
        self._start_server()

    @rpc
    def stop(self) -> None:
        self._stop_server()
        super().stop()

    @skill
    def notify_user(
        self,
        title: str,
        message: str,
        urgency: str = "normal",
        sound: bool = True,
        vibrate: bool = True,
        speak_text: str = "",
    ) -> str:
        """Send an alert to connected browser clients.

        Args:
            title: Short alert title.
            message: Alert body shown in the browser.
            urgency: One of "low", "normal", or "high". Unknown values become "normal".
            sound: Whether the browser should play an alert tone.
            vibrate: Whether the browser should request phone vibration.
            speak_text: Optional text for the robot to speak at the same time.
        """

        normalized_urgency = self._normalize_urgency(urgency)
        alert = self._make_alert(
            title=title,
            message=message,
            urgency=normalized_urgency,
            sound=sound,
            vibrate=vibrate,
        )
        client_count = self._publish_alert(alert)

        speech_status = ""
        if speak_text:
            speech_status = self._speak(speak_text)

        result = (
            f"Queued {normalized_urgency} browser notification '{title}' "
            f"for {client_count} connected client(s)."
        )
        if speech_status:
            result += f" {speech_status}"
        return result

    @staticmethod
    def _normalize_urgency(urgency: str) -> str:
        normalized = urgency.lower().strip()
        if normalized not in {"low", "normal", "high"}:
            return "normal"
        return normalized

    def _make_alert(
        self,
        title: str,
        message: str,
        urgency: str,
        sound: bool,
        vibrate: bool,
    ) -> AlertPayload:
        with self._lock:
            self._next_alert_id += 1
            alert_id = self._next_alert_id

        return {
            "id": alert_id,
            "title": title,
            "message": message,
            "urgency": urgency,
            "sound": bool(sound),
            "vibrate": bool(vibrate),
            "ts": time.time(),
        }

    def _publish_alert(self, alert: AlertPayload) -> int:
        with self._lock:
            self._latest_alert = alert
            queues = list(self._client_queues)

        for client_queue in queues:
            client_queue.put(alert)
        return len(queues)

    def _speak(self, text: str) -> str:
        if self._speaker is None:
            return "Speech skipped because SpeakSkill is not connected."
        try:
            return self._speaker.speak(text, blocking=False)
        except Exception:
            logger.exception("Failed to speak browser notification text")
            return "Speech failed."

    def _setup_routes(self) -> None:
        @self._web_server.app.get("/notify", response_class=HTMLResponse)
        async def notify_index() -> HTMLResponse:
            return HTMLResponse(content=_NOTIFY_HTML)

        @self._web_server.app.get("/notify/latest")
        async def latest_alert() -> JSONResponse:
            with self._lock:
                alert = self._latest_alert
            return JSONResponse(alert or {"id": 0})

        @self._web_server.app.websocket("/notify/ws")
        async def notify_ws(ws: WebSocket) -> None:
            await ws.accept()
            client_queue: ClientQueue = Queue()
            with self._lock:
                self._client_queues.add(client_queue)
                latest = self._latest_alert

            try:
                if latest is not None:
                    await ws.send_json(latest)
                while True:
                    alert = await asyncio.to_thread(client_queue.get)
                    if alert is None:
                        break
                    await ws.send_json(alert)
            except WebSocketDisconnect:
                logger.info("Browser notification client disconnected")
            finally:
                with self._lock:
                    self._client_queues.discard(client_queue)

    def _start_server(self) -> None:
        if self._web_server_thread is not None and self._web_server_thread.is_alive():
            logger.warning("Browser notification web server already running")
            return

        self._web_server_thread = threading.Thread(
            target=self._web_server.run,
            kwargs={
                "ssl": True,
                "ssl_certs_dir": get_project_root() / "assets" / "teleop_certs",
            },
            daemon=True,
            name="BrowserNotificationWebServer",
        )
        self._web_server_thread.start()
        logger.info(
            "Browser notification page started",
            url=f"https://0.0.0.0:{self.config.server_port}/notify",
        )

    def _stop_server(self) -> None:
        with self._lock:
            queues = list(self._client_queues)
            self._client_queues.clear()

        for client_queue in queues:
            client_queue.put(None)

        self._web_server.shutdown()
        if self._web_server_thread is not None:
            self._web_server_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._web_server_thread = None


_NOTIFY_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>DimOS Alerts</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101418;
      color: #f4f7f9;
    }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background: #101418;
    }
    main {
      width: min(92vw, 520px);
      display: grid;
      gap: 18px;
    }
    .status {
      color: #a7b3be;
      font-size: 14px;
    }
    .alert {
      min-height: 260px;
      border: 1px solid #2e3943;
      border-radius: 8px;
      padding: 28px;
      display: grid;
      align-content: center;
      gap: 16px;
      background: #172027;
      transition: background 160ms ease, border-color 160ms ease;
    }
    .alert.high {
      background: #451717;
      border-color: #ff5959;
    }
    .alert.normal {
      background: #183325;
      border-color: #4fd18b;
    }
    .alert.low {
      background: #1c2c3c;
      border-color: #64a7ff;
    }
    .title {
      font-size: clamp(40px, 16vw, 86px);
      font-weight: 800;
      line-height: 1;
    }
    .message {
      color: #dce5eb;
      font-size: 20px;
      line-height: 1.35;
    }
    button {
      height: 52px;
      border: 0;
      border-radius: 8px;
      color: #101418;
      background: #f4f7f9;
      font-size: 17px;
      font-weight: 700;
    }
    .flash {
      animation: flash 700ms ease-in-out 3;
    }
    @keyframes flash {
      50% { filter: brightness(1.55); }
    }
  </style>
</head>
<body>
  <main>
    <div class="status" id="status">Disconnected</div>
    <section class="alert" id="alert">
      <div class="title" id="title">DimOS</div>
      <div class="message" id="message">Tap enable, then wait for robot alerts.</div>
    </section>
    <button id="enable">Enable alerts</button>
  </main>
  <script>
    const statusEl = document.getElementById("status");
    const alertEl = document.getElementById("alert");
    const titleEl = document.getElementById("title");
    const messageEl = document.getElementById("message");
    const enableBtn = document.getElementById("enable");
    let enabled = false;
    let audioCtx = null;
    let lastAlertId = 0;

    function setStatus(text) {
      statusEl.textContent = text;
    }

    async function enableAlerts() {
      enabled = true;
      audioCtx = audioCtx || new AudioContext();
      if (audioCtx.state === "suspended") {
        await audioCtx.resume();
      }
      if ("Notification" in window && Notification.permission === "default") {
        await Notification.requestPermission();
      }
      if ("vibrate" in navigator) {
        navigator.vibrate([60, 40, 60]);
      }
      enableBtn.textContent = "Alerts enabled";
      setStatus("Connected");
    }

    function playTone(urgency) {
      if (!enabled || !audioCtx) return;
      const osc = audioCtx.createOscillator();
      const gain = audioCtx.createGain();
      const high = urgency === "high";
      osc.frequency.value = high ? 220 : 660;
      osc.type = high ? "square" : "sine";
      gain.gain.value = high ? 0.22 : 0.12;
      osc.connect(gain);
      gain.connect(audioCtx.destination);
      osc.start();
      osc.stop(audioCtx.currentTime + (high ? 0.8 : 0.35));
    }

    function vibrate(urgency) {
      if (!enabled || !("vibrate" in navigator)) return;
      const pattern = urgency === "high" ? [300, 80, 300, 80, 600] : [120, 60, 120];
      navigator.vibrate(pattern);
    }

    function showSystemNotification(alert) {
      if (!enabled || !("Notification" in window)) return;
      if (Notification.permission === "granted") {
        new Notification(alert.title, { body: alert.message });
      }
    }

    function renderAlert(alert) {
      if (!alert || !alert.id || alert.id <= lastAlertId) return;
      lastAlertId = alert.id;
      const urgency = alert.urgency || "normal";
      alertEl.className = `alert ${urgency} flash`;
      titleEl.textContent = alert.title || "Alert";
      messageEl.textContent = alert.message || "";
      setTimeout(() => alertEl.classList.remove("flash"), 2300);
      if (alert.sound) playTone(urgency);
      if (alert.vibrate) vibrate(urgency);
      showSystemNotification(alert);
    }

    function connect() {
      const protocol = location.protocol === "https:" ? "wss:" : "ws:";
      const ws = new WebSocket(`${protocol}//${location.host}/notify/ws`);
      ws.onopen = () => setStatus(enabled ? "Connected" : "Connected - enable alerts");
      ws.onmessage = (event) => renderAlert(JSON.parse(event.data));
      ws.onclose = () => {
        setStatus("Disconnected - retrying");
        setTimeout(connect, 1000);
      };
      ws.onerror = () => ws.close();
    }

    enableBtn.addEventListener("click", () => {
      enableAlerts().catch((err) => setStatus(`Enable failed: ${err}`));
    });
    connect();
  </script>
</body>
</html>
"""


__all__ = ["BrowserNotificationConfig", "BrowserNotificationSkill"]
