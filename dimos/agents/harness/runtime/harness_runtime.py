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

"""HarnessRuntime: the Python half of the on-robot agent, served by McpServer."""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
import threading
from typing import Any, Literal, TypeAlias, TypeVar
from uuid import uuid4

from pydantic import BaseModel, ConfigDict
from reactivex.disposable import Disposable

from dimos.agents.annotation import current_skill_context, skill
from dimos.agents.harness.runtime.agent_module_manager import (
    AgentModuleManager,
    AgentModuleOutcome,
    client_gone,
    deployed_revisions,
)
from dimos.agents.harness.runtime.constants import (
    AGENT_MODULE_PACKAGE,
    CONNECT_TIMEOUT_S,
    CONVERSATION_META,
    HARNESS_EVENT_METHOD,
    HARNESS_STATUS_METHOD,
    JSON,
)
from dimos.agents.harness.runtime.inventory import (
    Inventory,
    describe_module,
    describe_rpc,
    describe_runtime,
    environment,
    facts,
    module_text,
    relay_endpoint,
    rpc_text,
)
from dimos.agents.mcp.tool_stream import TOOL_STREAM_TOPIC
from dimos.agents.skill_result import SkillResult
from dimos.constants import HARNESS_STATE_DIR, RUNTIME_MODULES_DIR
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.agent_msgs.AgentNotice import AgentNotice
from dimos.msgs.agent_msgs.AgentStep import AgentStep
from dimos.porcelain.dimos import Dimos
from dimos.utils.logging_config import setup_logger

R = TypeVar("R")

StatusCause: TypeAlias = Literal["agent_module", "system_modules"]
"""An agent module operation or a change to the coordinator's module set."""


def _caller() -> int | None:
    context = current_skill_context()
    if context is None:
        return None
    conversation = context.get("meta", {}).get(CONVERSATION_META)
    if isinstance(conversation, int) and not isinstance(conversation, bool):
        return conversation
    return None


class TrajectoryStep(BaseModel):
    model_config = ConfigDict(strict=True, allow_inf_nan=False)

    conversation: int
    entry: str
    kind: str
    text: str
    ts: float


class HarnessRuntime(Module):
    """Discovers the system, runs agent modules, forwards notices and publishes trajectories."""

    dedicated_worker = True
    agent_notice: In[AgentNotice]
    agent_step: Out[AgentStep]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._logger = setup_logger()
        self._app_lock = threading.Lock()
        self._app: Dimos | None = None
        self._tool_streams_lock = threading.Lock()
        self._tool_streams: PubSubTransport[dict[str, JSON]] | None = None
        self._modules = AgentModuleManager(
            HARNESS_STATE_DIR / "modules",
            RUNTIME_MODULES_DIR / AGENT_MODULE_PACKAGE,
            connect=self._connect,
            reconnect=self._reconnect,
        )

    @rpc
    def start(self) -> None:
        super().start()
        transport: PubSubTransport[dict[str, JSON]] = make_transport(TOOL_STREAM_TOPIC)
        transport.start()
        with self._tool_streams_lock:
            self._tool_streams = transport
        self.register_disposable(Disposable(self.agent_notice.subscribe(self._on_notice)))

    @rpc
    def stop(self) -> None:
        with self._app_lock:
            app, self._app = self._app, None
        if app is not None:
            app.stop()
        with self._tool_streams_lock:
            transport, self._tool_streams = self._tool_streams, None
        if transport is not None:
            transport.stop()
        super().stop()

    @skill
    def publish_steps(self, steps: list[dict[str, Any]]) -> SkillResult:
        """Publish committed conversation entries on the ordinary agent_step stream.

        Args:
            steps: Entries with conversation (integer), entry (string), kind, text,
                and ts (Unix seconds). The whole batch is validated before publishing.

        Returns the number published. Recording follows the system's normal topic
        selection and retention; this call does not acknowledge durable storage.
        """
        parsed = [TrajectoryStep.model_validate(step) for step in steps]
        for step in parsed:
            self.agent_step.publish(
                AgentStep(step.conversation, step.entry, step.kind, step.text, step.ts)
            )
        return SkillResult(f"Published {len(parsed)} steps", metadata={"published": len(parsed)})

    @skill
    def inventory(
        self, module: str = "", rpc: str = "", compact: bool = False
    ) -> SkillResult | str:
        """Describe the running DimOS system, its modules, RPCs, streams, and saved agent
        modules. The whole-system result includes the environment and relay endpoint.
        Discovery is live on every call.

        An RPC is its module's Python API. A Dimos.connect() client calls it as
        app.get_module(MODULE).RPC(...); a module calls it through an attribute typed with
        the module's Spec. The harness's stream_get tool reads a relay-exposed stream once.
        A module reads native messages continuously through an In port of the same name
        and type.

        Args:
            module: An instance name or unique class name. Returns that module's
                documentation, its RPC signatures with the first paragraph of each doc,
                and its streams with message-type imports when available.
            rpc: Requires module. Returns one RPC with its full documentation.
            compact: Returns only the whole system's keyed one-line facts, which clients
                compare to report what changed. Ignored when module is set.
        """
        if rpc and not module:
            raise ValueError("rpc needs module")
        if module and rpc:
            rpc_detail = self._discover(lambda app: describe_rpc(app, module, rpc))
            return rpc_text(rpc_detail)
        if module:
            module_detail = self._discover(lambda app: describe_module(app, module))
            return module_text(module_detail)
        runtime = self._discover(describe_runtime)
        deployed = deployed_revisions([(entry.instance, entry.path) for entry in runtime.modules])
        agent_modules = self._modules.list(deployed)
        inventory = Inventory(environment(self._discover(relay_endpoint)), runtime, agent_modules)
        if compact:
            return SkillResult(metadata={"facts": [fact.to_json() for fact in facts(inventory)]})
        return SkillResult(metadata=inventory.to_json())

    @skill
    def agent_module(
        self,
        op: Literal["create", "restart", "unload", "delete", "list"],
        name: str = "",
        path: str = "",
    ) -> SkillResult:
        """Run agent-written DimOS modules for ongoing work such as watching streams,
        reacting to events, or commanding motion. Each uses its own worker by default.
        They stop when the system stops. Their source files remain, but the modules need
        restart to run again.

        A module is one Python file that defines one subclass of
        dimos.agents.harness.runtime.agent_module.AgentModule, which is a
        dimos.core.module.Module. It runs under its class name, which no running module may
        already use. An In[T] or Out[T] attribute with the name and type of a running stream
        joins that stream, and an attribute typed with a module's Spec calls that module's
        RPCs. AgentModule adds subscribe(stream, callback), whose callback errors go to the
        standard DimOS log, and notify(text, **data) -> bool for events that need the owning
        conversation's attention. notify() returns True after publishing and False when
        rate-limited by notify_interval_s, a class attribute that defaults to 10 seconds.
        Delivery requires a connected harness, and notices are not replayed after an outage.
        Every Image and PointCloud2 in data is saved to a file and replaced by its path.
        Other data must be JSON with finite numbers. Use dimos.utils.logging_config.setup_logger()
        for diagnostics and read them with dimos log or search the run's main.jsonl file.

        Keep the file's top level to imports and definitions, and keep setup short: the top
        level, __init__, and startup code run during loading. Slow setup can block other
        module lifecycle operations. An async main() must yield exactly once. Start loops
        and slow work in a background thread or task before the yield and stop them after it.

        Write the file with your write tool and create the module with path set to it.
        create copies it to the module's own source file, which every result names; from
        then on, edit that file and restart. Operation results report the resulting state
        and source path. Failures include an error and the module's traceback when available.

        Args:
            op: create starts a new module; restart runs the source file's current code,
                and leaves the old code running if the file does not compile or import.
                If the replacement fails during startup, the old code has already stopped;
                unload stops it; delete unloads it and removes it with its source file;
                list shows saved modules.
            name: Starts with a lowercase letter, contains only lowercase letters, digits
                and '_', and is at most 64 characters. Must not end in '_r' followed by
                digits. Not needed for list.
            path: With create, the absolute path of the file to start the module from.
                Without it, create uses the module's source file as it is.
        """
        if op == "list":
            return SkillResult(
                metadata={"agent_modules": [item.to_json() for item in self._modules.list()]}
            )
        if not name:
            raise ValueError(f"{op} needs name")
        if path and op != "create":
            raise ValueError("path is for create")
        try:
            outcome = self._change_module(op, name, path)
        finally:
            self._status("agent_module")
        return SkillResult(outcome.text(), metadata=outcome.to_json())

    @rpc
    def on_system_modules(self, _modules: list[RPCClient]) -> None:
        self._status("system_modules")

    def _change_module(
        self,
        op: Literal["create", "restart", "unload", "delete"],
        name: str,
        path: str,
    ) -> AgentModuleOutcome:
        match op:
            case "create":
                return self._modules.create(name, Path(path) if path else None, owner=_caller())
            case "restart":
                return self._modules.restart(name)
            case "unload":
                return self._modules.unload(name)
            case "delete":
                return self._modules.delete(name)

    def _status(self, cause: StatusCause) -> None:
        self._notify(HARNESS_STATUS_METHOD, {"cause": cause})

    def _notify(self, method: str, params: dict[str, JSON]) -> None:
        with self._tool_streams_lock:
            transport = self._tool_streams
        if transport is not None:
            transport.publish({"jsonrpc": "2.0", "method": method, "params": params})

    def _connect(self) -> Dimos:
        with self._app_lock:
            if self._app is None or not self._app.is_running:
                self._app = Dimos.connect(timeout=CONNECT_TIMEOUT_S)
            return self._app

    def _reconnect(self) -> None:
        with self._app_lock:
            app, self._app = self._app, None
        if app is not None:
            app.stop()

    def _discover(self, describe: Callable[[Dimos], R]) -> R:
        try:
            return describe(self._connect())
        except Exception as error:
            if not client_gone(error):
                raise
            self._reconnect()
            return describe(self._connect())

    def _on_notice(self, notice: AgentNotice) -> None:
        try:
            try:
                owner = self._modules.owner(notice.module)
            except (KeyError, ValueError):
                return
            self._notify(
                HARNESS_EVENT_METHOD,
                {
                    "id": str(uuid4()),
                    "module": notice.module,
                    "conversation": owner,
                    "at": notice.ts,
                    "text": notice.text,
                    "data": json.loads(notice.data),
                    "exports": json.loads(notice.exports),
                    "suppressed": notice.suppressed,
                },
            )
        except Exception:
            self._logger.exception("Agent notice not forwarded", module=notice.module)
