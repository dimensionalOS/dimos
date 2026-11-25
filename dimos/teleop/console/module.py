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

"""A small local web page for the operator of an arm: stop, home and saved homes.

The page only forwards clicks: the coordinator stops the robot and
:class:`ArmHomeModule` moves it and keeps its homes. The stop is the
coordinator's software stop, which holds the arms where they are. It is not a
safety-rated emergency stop.
"""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any, Literal

from fastapi import HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.arm_home import ArmHomeModule
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

_INDEX = Path(__file__).parent / "web" / "static" / "index.html"
# The web server base class already serves its own page at "/".
CONSOLE_PATH = "/console"


class OperatorConsoleConfig(ModuleConfig):
    host: str = Field(default_factory=lambda m: m["g"].listen_host)
    port: int = 8450


class _Stop(BaseModel):
    estopped: bool


class _Arm(BaseModel):
    arm: str


class _Move(BaseModel):
    arm: str
    target: Literal["home", "rest"]


class OperatorConsoleModule(Module):
    """Serve the operator console and forward its clicks.

    Routes, all under the page's own origin:
        GET  /console          the page
        GET  /api/state        the stop this console last set, and each arm's home
        POST /api/estop        stop every task, or clear the stop
        POST /api/home/save    make where an arm is now its home
        POST /api/home/clear   go back to an arm's default home
        POST /api/move         move an arm to home or rest without the planner
        POST /api/cancel       stop a move where it is
    """

    config: OperatorConsoleConfig

    coordinator: ControlCoordinator
    arm_home: ArmHomeModule

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._estopped = False
        self._server: RobotWebInterface | None = None
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        server = RobotWebInterface(host=self.config.host, port=self.config.port)
        self._routes(server)
        self._server = server
        self._thread = threading.Thread(target=server.run, daemon=True, name="OperatorConsole")
        self._thread.start()
        for url in server.get_urls(CONSOLE_PATH):
            logger.info("Operator console", url=url)

    @rpc
    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _routes(self, server: RobotWebInterface) -> None:
        app = server.app

        @app.get(CONSOLE_PATH, response_class=HTMLResponse)
        def index() -> HTMLResponse:
            return HTMLResponse(content=_INDEX.read_text())

        @app.get("/api/state")
        def state() -> dict[str, Any]:
            with self._lock:
                estopped = self._estopped
            return {"estopped": estopped, "arms": self.arm_home.get_homes()}

        @app.post("/api/estop")
        def estop(body: _Stop) -> dict[str, Any]:
            # The stop goes out before anything else this handler does.
            self.coordinator.set_estop(body.estopped)
            if body.estopped:
                self.arm_home.cancel()
            with self._lock:
                self._estopped = body.estopped
            return {"estopped": body.estopped}

        @app.post("/api/home/save")
        def save_home(body: _Arm) -> dict[str, Any]:
            return {"home": _call(self.arm_home.save_home, body.arm)}

        @app.post("/api/home/clear")
        def clear_home(body: _Arm) -> dict[str, Any]:
            return {"home": _call(self.arm_home.clear_home, body.arm)}

        @app.post("/api/move")
        def move(body: _Move) -> dict[str, Any]:
            return {"result": _call(self.arm_home.manual_move, body.arm, body.target)}

        @app.post("/api/cancel")
        def cancel() -> dict[str, Any]:
            self.arm_home.cancel()
            return {"result": "cancelled"}


def _call(method: Any, *args: Any) -> Any:
    """Run a homing call, turning a refusal into an HTTP 400 the page can show."""
    try:
        return method(*args)
    except (ValueError, RuntimeError) as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
