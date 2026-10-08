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

"""The running blueprint's skills (a module's `@skill` methods), over dimos's module RPC, as `dimos.porcelain`'s
RemoteModuleSource and SkillsProxy reach them: `Coordinator/list_modules` names the deployed modules, each one's
`<module>/get_skills` gives its skills (with the JSON schema McpServer gives an agent), and `<module>/<skill>` calls one.
Any blueprint with a skill has them; it needn't have an McpServer. A skill that holds a capability (`uses`) is called
through the run's McpServer instead when one answers, so the capability locks its agent goes by cover this call too."""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import threading
import time
from typing import Any, Protocol

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError

COORDINATOR = "Coordinator"
# the coordinator answers at once when a run is up
PING_TIMEOUT_S = 1.0
LIST_TIMEOUT_S = 3.0
# a skill runs until it returns (move_to waits up to 100 s for the robot to get there); a background one returns at once
CALL_TIMEOUT_S = 300.0
# a module restarted with new source can change its skills
CACHE_S = 30.0


class SkillError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


class Bus(Protocol):
    """dimos's module RPC, as a client: `name` is `<module>/<method>`; raises TimeoutError when nothing answers."""

    def call(self, name: str, args: list[Any], kwargs: dict[str, Any], timeout: float) -> Any: ...


class RpcBus:
    """The machine's dimos RPC bus (the backend `rpc_backend()` picks: LCM or zenoh), opened on first use and kept."""

    def __init__(self) -> None:
        self._rpc: Any = None
        self._lock = threading.Lock()

    def call(self, name: str, args: list[Any], kwargs: dict[str, Any], timeout: float) -> Any:
        with self._lock:
            if self._rpc is None:
                from dimos.core.transport_factory import rpc_backend

                rpc = rpc_backend()()
                rpc.start()
                self._rpc = rpc
        result, _unsub = self._rpc.call_sync(name, (args, kwargs), rpc_timeout=timeout)
        return result


bus: Bus = RpcBus()


@dataclass
class Run:
    """The live run the coordinator belongs to (none when it was started from python, not `dimos run`)."""

    run_id: str | None
    blueprint: str | None
    mcp_url: str | None


def _argv_port(argv: list[str]) -> str | None:
    """`--mcp-port N` / `--mcp-port=N` in a run's command line (the registry's overrides miss flags given after
    `run <blueprint>`)."""
    port = None
    for at, arg in enumerate(argv):
        if arg in ("--mcp-port", "--mcp_port") and at + 1 < len(argv):
            port = argv[at + 1]
        elif arg.startswith(("--mcp-port=", "--mcp_port=")):
            port = arg.split("=", 1)[1]
    return port


def mcp_url(entry: Any) -> str:
    """Where a run's McpServer would listen: its own `mcp_port` (override or command line), else GlobalConfig's."""
    from dimos.core.global_config import global_config

    overrides = getattr(entry, "config_overrides", None) or {}
    port = (
        overrides.get("mcp_port")
        or _argv_port(list(getattr(entry, "original_argv", None) or []))
        or global_config.mcp_port
    )
    return f"http://localhost:{int(port)}/mcp"


def live_run() -> Run:
    """The newest live run in the registry (one coordinator answers per bus: the gateway refuses a second launch)."""
    from dimos.core.run_registry import list_runs

    entries = sorted(list_runs(alive_only=True), key=lambda e: str(e.run_id), reverse=True)
    if not entries:
        return Run(None, None, None)
    return Run(str(entries[0].run_id), str(entries[0].blueprint), mcp_url(entries[0]))


def _modules(the_bus: Bus) -> list[Any] | None:
    """The coordinator's deployed modules (ModuleDescriptor), None when no coordinator answers."""
    try:
        return list(the_bus.call(f"{COORDINATOR}/list_modules", [], {}, PING_TIMEOUT_S))
    except TimeoutError:
        return None


def _skill(module: str, info: Any, run: Run) -> dict[str, Any]:
    # as McpServer's tools/list: the schema's description is the skill's, its title goes
    params = json.loads(info.args_schema)
    description = params.pop("description", "") or ""
    params.pop("title", None)
    params.setdefault("type", "object")
    params.setdefault("properties", {})
    return {
        "name": info.func_name,
        "module": module,
        "description": description,
        "params": params,
        "required": list(params.get("required") or []),
        "lifecycle": info.lifecycle,
        "uses": list(info.uses),
        "runId": run.run_id,
        "blueprint": run.blueprint,
    }


_cache: dict[str, Any] = {"key": None, "at": 0.0, "answer": None}
_cache_lock = threading.Lock()


def list_skills(the_bus: Bus | None = None, run: Run | None = None) -> dict[str, Any]:
    """Every skill of the running blueprint's modules; `errors` names a module that didn't say."""
    the_bus = the_bus or bus
    run = run or live_run()
    modules = _modules(the_bus)
    if modules is None:
        return {"skills": [], "run": None, "errors": []}
    names = sorted(d.rpc_name or d.class_name for d in modules if "get_skills" in d.rpc_names)
    key = (id(the_bus), run.run_id, tuple(names))
    with _cache_lock:
        if _cache["key"] == key and time.monotonic() - _cache["at"] < CACHE_S:
            return dict(_cache["answer"])

    def skills_of(name: str) -> tuple[str, list[Any] | Exception]:
        try:
            return name, list(the_bus.call(f"{name}/get_skills", [], {}, LIST_TIMEOUT_S))
        except Exception as error:
            return name, error

    skills: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    with ThreadPoolExecutor(max_workers=16) as pool:
        for name, found in pool.map(skills_of, names):
            if isinstance(found, Exception):
                errors.append({"module": name, "error": f"{type(found).__name__}: {found}"})
            else:
                skills += [_skill(name, info, run) for info in found]
    answer = {
        "skills": sorted(skills, key=lambda s: (s["name"], s["module"])),
        "run": {"runId": run.run_id, "blueprint": run.blueprint},
        "errors": errors,
    }
    with _cache_lock:
        _cache.update(key=key, at=time.monotonic(), answer=answer)
    return dict(answer)


def check_args(skill: dict[str, Any], args: dict[str, Any]) -> None:
    """A missing required argument or one the skill doesn't take is refused before anything reaches the robot."""
    known = skill["params"].get("properties") or {}
    missing = [name for name in skill["required"] if name not in args]
    unknown = [name for name in args if name not in known]
    if missing or unknown:
        takes = ", ".join(known) or "no arguments"
        raise SkillError(
            400,
            f"{skill['name']}: "
            + "; ".join(
                part
                for part in (
                    f"missing {', '.join(missing)}" if missing else "",
                    f"doesn't take {', '.join(unknown)}" if unknown else "",
                )
                if part
            )
            + f" (it takes {takes})",
        )


def _content(result: Any) -> list[dict[str, Any]]:
    """A skill's answer as MCP content blocks, as McpServer makes them."""
    if hasattr(result, "agent_encode"):
        return list(result.agent_encode())
    return [{"type": "text", "text": str(result)}]


def _through_mcp(
    url: str, skill: str, args: dict[str, Any], adapter_for: Callable[[str, int], McpAdapter]
) -> tuple[list[dict[str, Any]], bool] | None:
    """Calls it through the run's McpServer; None when none answers there."""
    adapter = adapter_for(url, int(LIST_TIMEOUT_S))
    try:
        if not any(tool.get("name") == skill for tool in adapter.list_tools()):
            return None
    except (McpError, OSError, ValueError):
        return None
    try:
        result = adapter_for(url, int(CALL_TIMEOUT_S)).call_tool(skill, args)
    except (McpError, OSError, ValueError) as error:
        raise SkillError(500, f"calling {skill} failed: {type(error).__name__}: {error}")
    # McpServer answers a busy capability as a plain text result, not an error
    return list(result.get("content") or []), not result.get("isError", False)


def call_skill(
    skill: str,
    args: dict[str, Any],
    module: str | None = None,
    run_id: str | None = None,
    the_bus: Bus | None = None,
    run: Run | None = None,
    adapter_for: Callable[[str, int], McpAdapter] = McpAdapter,
) -> dict[str, Any]:
    """Calls `skill` of the running blueprint (`module` picks one when two modules have it) and waits for its answer.
    Only ever on a person's or agent's explicit request: nothing here calls a skill by itself."""
    the_bus = the_bus or bus
    run = run or live_run()
    listed = list_skills(the_bus, run)
    if listed["run"] is None:
        raise SkillError(409, "no blueprint is running, so there are no skills to call")
    if run_id is not None and run_id != run.run_id:
        raise SkillError(404, f"run {run_id} isn't the running one ({run.run_id})")
    matches = [
        s
        for s in listed["skills"]
        if s["name"] == skill and (module is None or s["module"] == module)
    ]
    if not matches:
        known = sorted({s["name"] for s in listed["skills"]})
        where = f" of module {module}" if module else ""
        raise SkillError(
            404,
            f"the running blueprint has no skill {skill}{where}"
            + (f"; its skills: {', '.join(known)}" if known else "; it has no skills"),
        )
    if len(matches) > 1:
        raise SkillError(
            400,
            f"{len(matches)} modules have a skill {skill} ({', '.join(m['module'] for m in matches)}): say which, "
            "with module",
        )
    target = matches[0]
    check_args(target, args)
    through = None
    if target["uses"] and run.mcp_url:
        through = _through_mcp(run.mcp_url, skill, args, adapter_for)
    if through is not None:
        content, ok = through
        via = "mcp"
    else:
        via = "rpc"
        try:
            result = the_bus.call(f"{target['module']}/{skill}", [], args, CALL_TIMEOUT_S)
            content, ok = _content(result), True
        except TimeoutError:
            raise SkillError(504, f"{skill} didn't answer within {int(CALL_TIMEOUT_S)} s")
        except Exception as error:
            # the skill raised: the module RPC hands its exception back
            content = [
                {"type": "text", "text": f"Error running {skill}: {type(error).__name__}: {error}"}
            ]
            ok = False
    text = "\n".join(str(block.get("text", "")) for block in content if block.get("type") == "text")
    return {
        "skill": skill,
        "module": target["module"],
        "runId": run.run_id,
        "blueprint": run.blueprint,
        "via": via,
        "ok": ok,
        "text": text,
        "content": content,
    }
