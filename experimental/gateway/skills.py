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
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import threading
import time
from typing import Any, Protocol

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError

PING_TIMEOUT_S = 1.0
LIST_TIMEOUT_S = 3.0
CALL_TIMEOUT_S = 300.0
CACHE_S = 30.0


class SkillError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


class Bus(Protocol):
    def call(self, name: str, args: list[Any], kwargs: dict[str, Any], timeout: float) -> Any: ...


class CoordinatorBus:
    def __init__(self) -> None:
        self._coordinator: Any = None
        self._lock = threading.Lock()

    def call(self, name: str, args: list[Any], kwargs: dict[str, Any], timeout: float) -> Any:
        from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
        from dimos.core.transport_factory import rpc_backend

        with self._lock:
            if self._coordinator is None:
                rpc = rpc_backend()()
                rpc.start()
                self._coordinator = CoordinatorRPC(rpc)
        result, _ = self._coordinator.rpc.call_sync(name, (args, kwargs), rpc_timeout=timeout)
        return result


@dataclass
class Run:
    run_id: str | None
    blueprint: str | None
    mcp_url: str | None


def mcp_url(entry: Any) -> str:
    from dimos.core.global_config import global_config

    port = (entry.config_overrides or {}).get("mcp_port")
    argv = list(entry.original_argv or [])
    for at, arg in enumerate(argv):
        if arg in ("--mcp-port", "--mcp_port") and at + 1 < len(argv):
            port = argv[at + 1]
        elif arg.startswith(("--mcp-port=", "--mcp_port=")):
            port = arg.split("=", 1)[1]
    return f"http://localhost:{int(port or global_config.mcp_port)}/mcp"


def live_run() -> Run:
    from dimos.core.run_registry import list_runs

    entries = sorted(list_runs(alive_only=True), key=lambda e: str(e.run_id), reverse=True)
    if not entries:
        return Run(None, None, None)
    return Run(str(entries[0].run_id), str(entries[0].blueprint), mcp_url(entries[0]))


def _skill(module: str, info: Any, run: Run) -> dict[str, Any]:
    schema = json.loads(info.args_schema)
    description = schema.pop("description", "") or ""
    schema.pop("title", None)
    schema.setdefault("type", "object")
    schema.setdefault("properties", {})
    return {
        "name": info.func_name,
        "module": module,
        "description": description,
        "params": schema,
        "required": list(schema.get("required") or []),
        "lifecycle": info.lifecycle,
        "uses": list(info.uses),
        "runId": run.run_id,
        "blueprint": run.blueprint,
    }


class Skills:
    def __init__(
        self, bus: Bus | None = None, adapter: Callable[[str, int], McpAdapter] = McpAdapter
    ) -> None:
        self.bus = bus or CoordinatorBus()
        self.adapter = adapter
        self.cache: tuple[Any, float, dict[str, Any]] | None = None
        self.lock = threading.Lock()

    def listed(self, run: Run | None = None) -> dict[str, Any]:
        run = run or live_run()
        try:
            descriptors = list(self.bus.call("Coordinator/list_modules", [], {}, PING_TIMEOUT_S))
        except TimeoutError:
            return {"skills": [], "run": None, "errors": []}
        names = sorted(
            d.rpc_name or d.class_name for d in descriptors if "get_skills" in d.rpc_names
        )
        key = (run.run_id, tuple(names))
        with self.lock:
            if self.cache and self.cache[0] == key and time.monotonic() - self.cache[1] < CACHE_S:
                return dict(self.cache[2])

        def skills_of(name: str) -> tuple[str, list[Any] | Exception]:
            try:
                return name, list(self.bus.call(f"{name}/get_skills", [], {}, LIST_TIMEOUT_S))
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
        with self.lock:
            self.cache = (key, time.monotonic(), answer)
        return dict(answer)

    def _through_mcp(
        self, url: str, skill: str, args: dict[str, Any]
    ) -> tuple[list[Any], bool] | None:
        try:
            if not any(
                t.get("name") == skill for t in self.adapter(url, int(LIST_TIMEOUT_S)).list_tools()
            ):
                return None
        except (McpError, OSError, ValueError):
            return None
        try:
            result = self.adapter(url, int(CALL_TIMEOUT_S)).call_tool(skill, args)
        except (McpError, OSError, ValueError) as error:
            raise SkillError(500, f"calling {skill} failed: {type(error).__name__}: {error}")
        return list(result.get("content") or []), not result.get("isError", False)

    def call(
        self, skill: str, args: dict[str, Any], module: str | None = None, run_id: str | None = None
    ) -> dict[str, Any]:
        run = live_run()
        listed = self.listed(run)
        if listed["run"] is None:
            raise SkillError(409, "no blueprint is running, so there are no skills to call")
        if run_id is not None and run_id != run.run_id:
            raise SkillError(404, f"run {run_id} isn't the running one ({run.run_id})")
        matches = [
            s for s in listed["skills"] if s["name"] == skill and module in (None, s["module"])
        ]
        if not matches:
            known = ", ".join(sorted({s["name"] for s in listed["skills"]})) or "none"
            raise SkillError(
                404, f"the running blueprint has no skill {skill} (its skills: {known})"
            )
        if len(matches) > 1:
            which = ", ".join(m["module"] for m in matches)
            raise SkillError(
                400, f"{len(matches)} modules have a skill {skill} ({which}): pass module"
            )
        target = matches[0]
        known_args = target["params"].get("properties") or {}
        missing = [name for name in target["required"] if name not in args]
        unknown = [name for name in args if name not in known_args]
        if missing or unknown:
            problems = []
            if missing:
                problems.append(f"missing {', '.join(missing)}")
            if unknown:
                problems.append(f"doesn't take {', '.join(unknown)}")
            takes = ", ".join(known_args) or "no arguments"
            raise SkillError(400, f"{skill}: {'; '.join(problems)} (it takes {takes})")
        through = (
            self._through_mcp(run.mcp_url, skill, args) if target["uses"] and run.mcp_url else None
        )
        if through is not None:
            (content, ok), via = through, "mcp"
        else:
            via = "rpc"
            try:
                result = self.bus.call(f"{target['module']}/{skill}", [], args, CALL_TIMEOUT_S)
                encoded = result.agent_encode() if hasattr(result, "agent_encode") else None
                content, ok = list(encoded or [{"type": "text", "text": str(result)}]), True
            except TimeoutError:
                raise SkillError(500, f"{skill} didn't answer within {int(CALL_TIMEOUT_S)} s")
            except Exception as error:
                text = f"Error running {skill}: {type(error).__name__}: {error}"
                content, ok = [{"type": "text", "text": text}], False
        return {
            "skill": skill,
            "module": target["module"],
            "runId": run.run_id,
            "blueprint": run.blueprint,
            "via": via,
            "ok": ok,
            "text": "\n".join(str(b.get("text", "")) for b in content if b.get("type") == "text"),
            "content": content,
        }


MCP_PROTOCOL = "2025-06-18"
MCP_TOOLS: list[dict[str, Any]] = [
    {
        "name": "list_skills",
        "description": "The skills of the robot's running dimos blueprint: name, module, description and JSON-schema params. Call this before call_skill.",
        "inputSchema": {"type": "object", "properties": {"query": {"type": "string"}}},
    },
    {
        "name": "call_skill",
        "description": "Call one skill of the running blueprint and wait for its answer. This acts on the robot.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "skill": {"type": "string"},
                "args": {"type": "object"},
                "module": {"type": "string"},
            },
            "required": ["skill"],
        },
    },
]


def mcp_tool(skills: Skills, name: str, arguments: dict[str, Any]) -> tuple[str, bool]:
    if name == "list_skills":
        answer = skills.listed()
        if answer["run"] is None:
            return "No blueprint is running, so there are no skills. Launch one first.", False
        words = str(arguments.get("query") or "").lower().split()
        keys = ("name", "module", "description", "params", "lifecycle", "blueprint")
        found = [
            {key: s[key] for key in keys}
            for s in answer["skills"]
            if all(w in f"{s['name']} {s['module']} {s['description']}".lower() for w in words)
        ]
        problems = [f"{e['module']}: {e['error']}" for e in answer["errors"]]
        return json.dumps({"skills": found, "problems": problems}), False
    if name == "call_skill":
        skill, args = arguments.get("skill"), arguments.get("args") or {}
        if not isinstance(skill, str) or not isinstance(args, dict):
            return "call_skill needs skill (a string) and args (an object)", True
        try:
            result = skills.call(skill, args, arguments.get("module"))
        except SkillError as error:
            return str(error), True
        return result["text"] or "(no answer text)", not result["ok"]
    return f"no tool {name}", True
