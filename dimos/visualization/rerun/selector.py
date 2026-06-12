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

import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any, Protocol
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT, RERUN_WEB_VIEWER_PORT

logger = setup_logger()


class RerunTopicSelectionSpec(Spec, Protocol):
    def get_topic_catalog(self) -> list[dict[str, Any]]: ...
    def stage_topics(self, topics: list[str]) -> list[str]: ...
    def apply_staged_topics(self) -> list[str]: ...


class Config(ModuleConfig):
    host: str | None = None
    port: int | None = None
    api_port: int | None = None
    backend_port: int | None = None
    title: str = "DimOS Visual Console"
    rerun_web_url: str | None = None
    rerun_connect_url: str | None = None


def browser_connect_host(host: str) -> str:
    """Return a browser-usable host for locally bound services."""

    if host in {"0.0.0.0", "::"}:
        return "localhost"
    return host


def rerun_web_viewer_url(web_url: str, connect_url: str) -> str:
    """Build a Rerun web viewer URL that connects to the bridge source.

    Rerun's web viewer reads live sources from the ``url`` query parameter. The
    value must be URL-encoded so ``rerun+http://...`` is not parsed as
    ``rerun http://...``.
    """

    parsed = urlsplit(web_url)
    query = parse_qsl(parsed.query, keep_blank_values=True)
    if any(key == "url" for key, _ in query):
        return web_url
    query.append(("url", connect_url))
    return urlunsplit(
        (parsed.scheme, parsed.netloc, parsed.path or "/", urlencode(query), parsed.fragment)
    )


class RerunTopicSelectorModule(Module):
    """Reflex web console for staging/applying selector-managed Rerun topics."""

    config: Config
    _bridge: RerunTopicSelectionSpec
    dedicated_worker = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._api_thread: threading.Thread | None = None
        self._api_server: Any | None = None
        self._reflex_process: subprocess.Popen[bytes] | None = None

    @property
    def port(self) -> int:
        return self.config.port or (RERUN_WEB_VIEWER_PORT + 1)

    @property
    def api_port(self) -> int:
        return self.config.api_port or (self.port + 1)

    @property
    def backend_port(self) -> int:
        return self.config.backend_port or (self.port + 2)

    @property
    def rerun_url(self) -> str:
        web_url = (
            self.config.rerun_web_url
            or f"http://{browser_connect_host(self.config.g.listen_host)}:{RERUN_WEB_VIEWER_PORT}"
        )
        connect_url = (
            self.config.rerun_connect_url
            or f"rerun+http://{browser_connect_host(self.config.g.rerun_host or self.config.g.listen_host)}:{RERUN_GRPC_PORT}/proxy"
        )
        return rerun_web_viewer_url(web_url, connect_url)

    @rpc
    def start(self) -> None:
        super().start()
        self._start_selector_api()
        self._start_reflex()

    @rpc
    def stop(self) -> None:
        self._stop_reflex()
        self._stop_selector_api()
        super().stop()

    def _start_selector_api(self) -> None:
        try:
            from fastapi import FastAPI
            import uvicorn
        except ImportError as exc:
            raise RuntimeError(
                "Rerun topic selector requires Reflex/FastAPI/Uvicorn. Install the "
                "visualization extra or add `reflex` to the environment."
            ) from exc

        app = FastAPI(title="DimOS Rerun Topic Selector API")

        @app.get("/health")
        def health() -> dict[str, Any]:
            return {"ok": True, "title": self.config.title}

        @app.get("/catalog")
        def catalog() -> dict[str, Any]:
            return {
                "catalog": self._bridge.get_topic_catalog(),
                "title": self.config.title,
                "rerun_url": self.rerun_url,
            }

        @app.post("/stage")
        def stage(payload: dict[str, list[str]]) -> dict[str, Any]:
            return {"topics": self._bridge.stage_topics(payload.get("topics", []))}

        @app.post("/apply")
        def apply() -> dict[str, Any]:
            return {"topics": self._bridge.apply_staged_topics()}

        @app.post("/clear")
        def clear() -> dict[str, Any]:
            return {"topics": self._bridge.stage_topics([])}

        def run() -> None:
            host = self.config.host or self.config.g.listen_host
            logger.info("Starting Rerun topic selector API", host=host, port=self.api_port)
            try:
                config = uvicorn.Config(
                    app,
                    host=host,
                    port=self.api_port,
                    log_level="warning",
                )
                server = uvicorn.Server(config)
                self._api_server = server
                server.run()
            except Exception:
                logger.exception("Rerun topic selector API failed")

        self._api_thread = threading.Thread(target=run, daemon=True)
        self._api_thread.start()

    def _start_reflex(self) -> None:
        try:
            import reflex  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                "Rerun topic selector requires Reflex. Install the visualization extra "
                "or add `reflex` to the environment."
            ) from exc

        host = self.config.host or self.config.g.listen_host
        browser_host = browser_connect_host(host)
        repo_root = Path(__file__).resolve().parents[3]
        app_root = Path(__file__).resolve().with_name("reflex_selector_app")
        env = os.environ.copy()
        env["DIMOS_SELECTOR_API_URL"] = f"http://{browser_host}:{self.api_port}"
        env["DIMOS_SELECTOR_RERUN_URL"] = self.rerun_url
        env["DIMOS_SELECTOR_TITLE"] = self.config.title
        env["PYTHONPATH"] = (
            f"{repo_root}{os.pathsep}{env['PYTHONPATH']}"
            if env.get("PYTHONPATH")
            else str(repo_root)
        )
        cmd = [
            "reflex",
            "run",
            "--env",
            "dev",
            "--frontend-port",
            str(self.port),
            "--backend-port",
            str(self.backend_port),
            "--backend-host",
            host,
            "--loglevel",
            "info",
        ]
        logger.info(
            "Starting Rerun topic selector Reflex UI",
            host=host,
            port=self.port,
            backend_port=self.backend_port,
            api_url=env["DIMOS_SELECTOR_API_URL"],
        )
        self._reflex_process = subprocess.Popen(
            cmd,
            cwd=app_root,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def _stop_reflex(self) -> None:
        process = self._reflex_process
        self._reflex_process = None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        except Exception:
            process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            logger.warning("Rerun topic selector Reflex UI did not stop; killing")
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except Exception:
                process.kill()
            process.wait(timeout=2.0)

    def _stop_selector_api(self) -> None:
        if self._api_server is not None:
            self._api_server.should_exit = True
        if self._api_thread is not None:
            deadline = time.monotonic() + 5.0
            while self._api_thread.is_alive() and time.monotonic() < deadline:
                self._api_thread.join(timeout=0.1)
        self._api_thread = None
        self._api_server = None
