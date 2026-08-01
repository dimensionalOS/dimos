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

"""Loopback-only web presentation for the read-only code-policy observer."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Awaitable, Callable
from importlib import resources
from pathlib import Path
import socket
import sys
import threading
import time
from typing import Any, TextIO
import webbrowser

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
import uvicorn

from dimos.agents.code_policy import CodePolicyObserverDescriptor
from dimos.agents.code_policy_observer import ObservationEvent

DEFAULT_WEB_PORT = 8766
_SERVER_STARTUP_TIMEOUT_S = 5.0
_SERVER_STOP_TIMEOUT_S = 5.0
_KEEPALIVE_INTERVAL_S = 15.0
_ASSET_PACKAGE = "dimos.agents"
_ASSET_DIRECTORY = "code_policy_web_assets"
_CONTENT_SECURITY_POLICY = (
    "default-src 'self'; script-src 'self'; style-src 'self'; "
    "img-src 'self' data:; connect-src 'self'; object-src 'none'; "
    "base-uri 'none'; frame-ancestors 'none'"
)


class ObservationEventHub:
    """Thread-safe replay and fan-out for one observer recording."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._events: list[ObservationEvent] = []
        self._closed = False

    def publish(self, event: ObservationEvent) -> None:
        with self._condition:
            if self._closed:
                return
            self._events.append(event)
            self._condition.notify_all()

    def wait_after(
        self, sequence: int, timeout_s: float
    ) -> tuple[tuple[ObservationEvent, ...], bool]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._condition:
            while not self._closed and not any(event.sequence > sequence for event in self._events):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            events = tuple(event for event in self._events if event.sequence > sequence)
            return events, self._closed

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()


class WebSessionState:
    """Credential-free status projection exposed to the browser."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: dict[str, Any] = {
            "status": "starting",
            "recording_path": None,
            "code_policy_session_id": None,
            "jupyter_client_session_id": None,
            "kernel_generation": None,
        }

    def set_recording_path(self, path: Path) -> None:
        with self._lock:
            self._value["recording_path"] = str(path.resolve())

    def ready(self, descriptor: CodePolicyObserverDescriptor) -> None:
        with self._lock:
            self._value.update(
                {
                    "status": "ready",
                    "code_policy_session_id": descriptor.code_policy_session_id,
                    "jupyter_client_session_id": descriptor.jupyter_client_session_id,
                    "kernel_generation": descriptor.kernel_generation,
                }
            )

    def observe(self, event: ObservationEvent) -> None:
        with self._lock:
            self._value["code_policy_session_id"] = event.code_policy_session_id
            self._value["kernel_generation"] = event.kernel_generation
            if event.kind == "lifecycle":
                self._value["status"] = str(event.content.get("event", "lifecycle"))

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._value)


def create_web_app(hub: ObservationEventHub, state: WebSessionState) -> FastAPI:
    """Create the read-only observer application."""
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

    @app.middleware("http")
    async def security_headers(
        request: Request, call_next: Callable[[Request], Awaitable[Response]]
    ) -> Response:
        response: Response = await call_next(request)
        response.headers["Content-Security-Policy"] = _CONTENT_SECURITY_POLICY
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Referrer-Policy"] = "no-referrer"
        response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return _read_asset("index.html")

    @app.get("/assets/app.css")
    async def stylesheet() -> Response:
        return Response(_read_asset("app.css"), media_type="text/css")

    @app.get("/assets/app.js")
    async def javascript() -> Response:
        return Response(_read_asset("app.js"), media_type="text/javascript")

    @app.get("/api/session")
    async def session() -> JSONResponse:
        return JSONResponse(state.snapshot())

    @app.get("/api/events")
    async def events(request: Request) -> StreamingResponse:
        sequence = _last_event_id(request)

        async def event_stream() -> AsyncIterator[str]:
            nonlocal sequence
            while True:
                observed, closed = await asyncio.to_thread(
                    hub.wait_after, sequence, _KEEPALIVE_INTERVAL_S
                )
                for event in observed:
                    sequence = event.sequence
                    yield (
                        f"id: {event.sequence}\n"
                        "event: observation\n"
                        f"data: {event.model_dump_json()}\n\n"
                    )
                if closed:
                    yield "event: observer-close\ndata: {}\n\n"
                    return
                if not observed:
                    yield ": keepalive\n\n"

        return StreamingResponse(
            event_stream(),
            media_type="text/event-stream",
            headers={"Connection": "keep-alive"},
        )

    return app


class CodePolicyWebServer:
    """Own the loopback HTTP server and credential-free browser state."""

    def __init__(self, port: int = DEFAULT_WEB_PORT) -> None:
        if not 0 <= port <= 65535:
            raise ValueError("web port must be between 0 and 65535")
        self.hub = ObservationEventHub()
        self.state = WebSessionState()
        self._requested_port = port
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self.url: str | None = None

    def start(self, recording_path: Path) -> str:
        self.state.set_recording_path(recording_path)
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind(("127.0.0.1", self._requested_port))
            listener.listen()
        except OSError:
            listener.close()
            raise
        actual_port = int(listener.getsockname()[1])
        app = create_web_app(self.hub, self.state)
        config = uvicorn.Config(app, log_level="warning", access_log=False)
        server = uvicorn.Server(config)
        thread = threading.Thread(
            target=server.run,
            kwargs={"sockets": [listener]},
            name="code-policy-observer-web",
            daemon=True,
        )
        self._socket = listener
        self._server = server
        self._thread = thread
        thread.start()
        deadline = time.monotonic() + _SERVER_STARTUP_TIMEOUT_S
        while not server.started and thread.is_alive() and time.monotonic() < deadline:
            threading.Event().wait(0.01)
        if not server.started:
            self.stop()
            raise RuntimeError("code-policy observer web server failed to start")
        self.url = f"http://127.0.0.1:{actual_port}"
        return self.url

    def publish(self, event: ObservationEvent) -> None:
        self.state.observe(event)
        self.hub.publish(event)

    def connected(self, descriptor: CodePolicyObserverDescriptor) -> None:
        self.state.ready(descriptor)

    def stop(self) -> None:
        self.hub.close()
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=_SERVER_STOP_TIMEOUT_S)
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        self._server = None
        self._thread = None
        self._socket = None


class WebObserverView:
    """Notebook-like web view with status-only terminal output."""

    def __init__(
        self,
        *,
        port: int = DEFAULT_WEB_PORT,
        open_browser: bool = True,
        stream: TextIO | None = None,
        browser_opener: Callable[[str], Any] = webbrowser.open,
    ) -> None:
        self._server = CodePolicyWebServer(port)
        self._open_browser = open_browser
        self._stream = stream or sys.stdout
        self._browser_opener = browser_opener

    def start(self, recording_path: Path) -> None:
        try:
            url = self._server.start(recording_path)
        except OSError as exc:
            raise RuntimeError(f"could not bind code-policy observer web server: {exc}") from exc
        print(f"recording: {recording_path.resolve()}", file=self._stream, flush=True)
        print(
            "warning: recording and browser may contain code, environment, perception, "
            "map, and robot data",
            file=self._stream,
            flush=True,
        )
        print(f"web: {url}", file=self._stream, flush=True)

    def render(self, event: ObservationEvent) -> None:
        self._server.publish(event)
        if event.kind == "lifecycle":
            print(
                f"--- {event.content.get('event', 'lifecycle')} ---",
                file=self._stream,
                flush=True,
            )

    def connected(self, descriptor: CodePolicyObserverDescriptor, *, initial: bool) -> None:
        self._server.connected(descriptor)
        print(
            f"session={descriptor.code_policy_session_id} "
            f"client={descriptor.jupyter_client_session_id} "
            f"generation={descriptor.kernel_generation}",
            file=self._stream,
            flush=True,
        )
        if initial:
            print("ready", file=self._stream, flush=True)
        if initial and self._open_browser and self._server.url is not None:
            self._browser_opener(self._server.url)

    def close(self) -> None:
        self._server.stop()


def _read_asset(name: str) -> str:
    return (
        resources.files(_ASSET_PACKAGE).joinpath(_ASSET_DIRECTORY, name).read_text(encoding="utf-8")
    )


def _last_event_id(request: Request) -> int:
    raw = request.headers.get("last-event-id", "0")
    try:
        return max(0, int(raw))
    except ValueError:
        return 0
