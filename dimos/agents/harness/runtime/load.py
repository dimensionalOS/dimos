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

"""Load the agent's relay bridge, HarnessRuntime and McpServer into the running system."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
import sys
from urllib.parse import urlsplit

from dimos.agents.harness.runtime.constants import JSON
from dimos.agents.harness.runtime.harness_runtime import HarnessRuntime
from dimos.agents.mcp.constants import mcp_url
from dimos.agents.mcp.mcp_server import McpServer, McpServerConfig
from dimos.agents.mcp.spec import McpServerSpec
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.porcelain.dimos import Dimos
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule


@dataclass(frozen=True)
class LoadResult:
    loaded: list[str]
    """Class names this call ran."""
    already: list[str]
    """Class names that were running."""

    def to_json(self) -> dict[str, JSON]:
        return {"loaded": list(self.loaded), "already": list(self.already)}


def _configured_blueprint(blueprint: Blueprint) -> Blueprint:
    (atom,) = blueprint.active_blueprints
    scoped = {
        name: value
        for name, value in os.environ.items()
        if name.lower().startswith(f"{atom.name}__")
    }
    parsed = BlueprintConfigParser(blueprint).parse(environ=scoped)
    return atom.module.blueprint(**{**atom.kwargs, **parsed.module_kwargs(atom.name)})


def mcp_server_blueprint() -> Blueprint:
    """McpServer with its MCPSERVER__* environment settings."""
    return _configured_blueprint(McpServer.blueprint())


def mcp_server_url() -> str:
    """The URL that clients on this host dial for the McpServer of mcp_server_blueprint()."""
    (atom,) = mcp_server_blueprint().active_blueprints
    config = McpServerConfig(**atom.kwargs)
    return mcp_url(config.host, config.port)


def _reaches(dialed: str, served: str) -> bool:
    """Whether a client that dials `dialed` reaches the server whose mcp_url() is `served`.

    A wildcard bind serves "localhost", which every address of this host reaches, and the
    loopback names stand for each other.
    """
    client, server = urlsplit(dialed), urlsplit(served)
    loopback = {"localhost", "127.0.0.1", "::1"}
    return client.port == server.port and (
        client.hostname == server.hostname
        or server.hostname == "localhost"
        or {client.hostname, server.hostname} <= loopback
    )


def load() -> LoadResult:
    """Ensure the relay bridge, HarnessRuntime and McpServer are running.

    Raises when the running system's McpServer serves an endpoint that `dimos agent` does
    not dial.
    """
    app = Dimos.connect()
    try:
        modules = app.list_modules()
        running = {info.class_name for info in modules}
        if McpServer.__name__ in running:
            served = app.find_module_by_spec(McpServerSpec).mcp_url()
            dialed = mcp_server_url()
            if not _reaches(dialed, served):
                raise RuntimeError(
                    f"The running McpServer serves {served}, but the agent dials {dialed}. "
                    "Give `dimos agent` the system's MCPSERVER__HOST and MCPSERVER__PORT."
                )
        loaded: list[str] = []
        already: list[str] = []
        bridge = next(
            (info for info in modules if any(rpc.name == "relay_endpoint" for rpc in info.rpcs)),
            None,
        )
        for cls in (RelayBridgeModule, HarnessRuntime, McpServer):
            if cls is RelayBridgeModule and bridge is not None:
                already.append(bridge.class_name or bridge.name)
                continue
            if cls.__name__ in running:
                already.append(cls.__name__)
                continue
            try:
                if cls is RelayBridgeModule:
                    app.run(
                        _configured_blueprint(
                            cls.blueprint(rtc=False, open_browser=False, web_build=False)
                        )
                    )
                else:
                    app.run(mcp_server_blueprint() if cls is McpServer else cls)
            except BaseException:
                # A module whose start() failed stays deployed; unloaded, the next load retries it.
                if cls.__name__ in {info.class_name for info in app.list_modules()}:
                    app.unload(cls)
                raise
            loaded.append(cls.__name__)
        return LoadResult(loaded, already)
    finally:
        app.stop()


def main() -> int:
    """Print the result as one JSON line on stdout. Everything else goes to stderr."""
    stdout = os.dup(1)
    os.dup2(2, 1)
    try:
        try:
            result = load()
        except Exception as error:
            print(f"dimos agent load: {type(error).__name__}: {error}", file=sys.stderr)
            return 1
        os.write(stdout, (json.dumps(result.to_json()) + "\n").encode())
        return 0
    finally:
        os.close(stdout)
