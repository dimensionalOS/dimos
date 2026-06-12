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

from collections.abc import Callable
from dataclasses import dataclass
import signal
import socket
import threading
import time
from typing import Any
import webbrowser

from dimos.core.run_registry import RunEntry, get_most_recent, list_runs
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT, RERUN_WEB_VIEWER_PORT
from dimos.visualization.rerun.selector import browser_connect_host, rerun_web_viewer_url

logger = setup_logger()

VISUALIZATION_EXTRA_HINT = (
    "Install visualization dependencies with `uv sync --extra visualization`."
)
_DEFAULT_FORBIDDEN_PORTS = {
    RERUN_GRPC_PORT,
    RERUN_WEB_VIEWER_PORT,
    RERUN_WEB_VIEWER_PORT + 1,
    RERUN_WEB_VIEWER_PORT + 2,
    RERUN_WEB_VIEWER_PORT + 3,
}


class TopicMonitorDependencyError(RuntimeError):
    """Raised when visualization dependencies required by the monitor are missing."""


class PortAllocationError(RuntimeError):
    """Raised when the requested topic monitor port set cannot be allocated."""


@dataclass(frozen=True)
class MonitorPorts:
    rerun_grpc: int
    rerun_web: int
    selector_frontend: int
    selector_api: int
    reflex_backend: int


@dataclass(frozen=True)
class RunContext:
    entry: RunEntry | None
    requested: str | None = None

    @property
    def label(self) -> str:
        if self.entry is None:
            return "LCM bus-only mode (no active DimOS run selected)"
        return f"{self.entry.run_id} ({self.entry.blueprint}, pid {self.entry.pid})"


@dataclass(frozen=True)
class TopicMonitorUrls:
    selector: str
    rerun_viewer: str
    selector_api: str
    rerun_connect: str


def _block_is_available(host: str, ports: list[int]) -> bool:
    sockets: list[socket.socket] = []
    try:
        for port in ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((host, port))
            sockets.append(sock)
        return True
    except OSError:
        return False
    finally:
        for sock in sockets:
            sock.close()


def allocate_monitor_ports(
    host: str = "127.0.0.1",
    *,
    port_base: int | None = None,
    start: int = 11000,
    end: int = 11999,
) -> MonitorPorts:
    """Allocate an isolated local port block for a topic monitor sidecar.

    Auto allocation deliberately avoids the historic Rerun/selector defaults so
    the sidecar cannot silently connect to an already-running embedded bridge.
    """

    count = 5
    bind_host = "127.0.0.1" if host in {"0.0.0.0", "::", "localhost"} else host

    def ports_from(base: int) -> list[int]:
        return [base + offset for offset in range(count)]

    if port_base is not None:
        ports = ports_from(port_base)
        if not _block_is_available(bind_host, ports):
            raise PortAllocationError(
                f"Requested topic monitor port block {port_base}-{port_base + 4} is not available"
            )
        return MonitorPorts(*ports)

    for base in range(start, end - count + 2):
        ports = ports_from(base)
        if any(port in _DEFAULT_FORBIDDEN_PORTS for port in ports):
            continue
        if _block_is_available(bind_host, ports):
            return MonitorPorts(*ports)

    raise PortAllocationError(f"No free topic monitor port block found in {start}-{end}")


def resolve_run_context(run: str | None = None) -> RunContext:
    """Resolve the run metadata used for monitor CLI context.

    Run metadata is informational only; the monitor observes the visible LCM bus.
    """

    if run is None or run == "latest":
        return RunContext(entry=get_most_recent(alive_only=True), requested=run)

    for entry in list_runs(alive_only=True):
        if entry.run_id == run:
            return RunContext(entry=entry, requested=run)
    raise ValueError(f"No active DimOS run found with run id {run!r}")


def _require_visualization_dependencies() -> None:
    missing: list[str] = []
    for module_name in ("rerun", "reflex", "fastapi", "uvicorn"):
        try:
            __import__(module_name)
        except ImportError:
            missing.append(module_name)
    if missing:
        joined = ", ".join(missing)
        raise TopicMonitorDependencyError(
            f"`dimos topic monitor` requires visualization dependencies; missing: {joined}. "
            f"{VISUALIZATION_EXTRA_HINT}"
        )


class TopicMonitorSidecar:
    """Foreground-owned topic monitor sidecar lifecycle."""

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        ports: MonitorPorts | None = None,
        title: str = "DimOS Topic Monitor",
    ) -> None:
        self.host = host
        self.browser_host = browser_connect_host(host)
        self.ports = ports or allocate_monitor_ports(host)
        self.title = title
        self._bridge: Any | None = None
        self._selector: Any | None = None

    @property
    def urls(self) -> TopicMonitorUrls:
        rerun_connect = f"rerun+http://{self.browser_host}:{self.ports.rerun_grpc}/proxy"
        rerun_web = rerun_web_viewer_url(
            f"http://{self.browser_host}:{self.ports.rerun_web}",
            rerun_connect,
        )
        return TopicMonitorUrls(
            selector=f"http://{self.browser_host}:{self.ports.selector_frontend}",
            rerun_viewer=rerun_web,
            selector_api=f"http://{self.browser_host}:{self.ports.selector_api}",
            rerun_connect=rerun_connect,
        )

    def start(self) -> None:
        _require_visualization_dependencies()
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM
        from dimos.protocol.service.lcmservice import autoconf
        from dimos.visualization.rerun.bridge import RerunBridgeModule
        from dimos.visualization.rerun.selector import RerunTopicSelectorModule

        autoconf(check_only=True)
        urls = self.urls
        bridge = RerunBridgeModule(
            pubsubs=[LCM()],
            selector_enabled=True,
            connect_url=urls.rerun_connect,
            rerun_open="none",
            rerun_web=True,
            web_port=self.ports.rerun_web,
            blueprint=None,
        )
        selector = RerunTopicSelectorModule(
            host=self.host,
            port=self.ports.selector_frontend,
            api_port=self.ports.selector_api,
            backend_port=self.ports.reflex_backend,
            title=self.title,
            rerun_web_url=f"http://{self.browser_host}:{self.ports.rerun_web}",
            rerun_connect_url=urls.rerun_connect,
        )
        selector._bridge = bridge

        bridge.start()
        try:
            selector.start()
        except Exception:
            bridge.stop()
            raise
        self._bridge = bridge
        self._selector = selector

    def stop(self) -> None:
        selector = self._selector
        bridge = self._bridge
        self._selector = None
        self._bridge = None
        if selector is not None:
            try:
                selector.stop()
            except Exception:
                logger.exception("Topic monitor selector shutdown failed")
        if bridge is not None:
            try:
                bridge.stop()
            except Exception:
                logger.exception("Topic monitor Rerun bridge shutdown failed")


def open_selector_url(url: str, *, opener: Callable[[str], bool] = webbrowser.open) -> bool:
    """Open the selector URL in a browser, returning whether it succeeded."""

    try:
        return bool(opener(url))
    except Exception:
        logger.warning("Could not open topic monitor browser", exc_info=True)
        return False


def run_topic_monitor(
    *,
    run: str | None = None,
    open_browser: bool = True,
    host: str = "127.0.0.1",
    port_base: int | None = None,
) -> None:
    """Run the topic monitor foreground sidecar until interrupted."""

    run_context = resolve_run_context(run)
    if run_context.entry is None:
        print("No active DimOS run selected; observing visible LCM bus traffic only.")
    else:
        print(f"Observing LCM bus with run context: {run_context.label}")

    sidecar = TopicMonitorSidecar(
        host=host, ports=allocate_monitor_ports(host, port_base=port_base)
    )
    sidecar.start()
    urls = sidecar.urls

    print("")
    print("DimOS Topic Monitor running")
    print(f"  Selector UI:  {urls.selector}")
    print(f"  Selector API: {urls.selector_api}")
    print(f"  Rerun viewer: {urls.rerun_viewer}")
    print(f"  Rerun source: {urls.rerun_connect}")
    print("  Stop: Ctrl-C")

    if open_browser and not open_selector_url(urls.selector):
        print(f"Could not open browser automatically. Open {urls.selector} manually.")

    stop_event = threading.Event()
    old_sigterm = signal.getsignal(signal.SIGTERM)
    old_sigint = signal.getsignal(signal.SIGINT)

    def _stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        while not stop_event.is_set():
            time.sleep(0.2)
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
        signal.signal(signal.SIGINT, old_sigint)
        sidecar.stop()
        print("\nTopic monitor stopped.")
