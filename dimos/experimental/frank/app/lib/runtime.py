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

"""Frank's operation registry and one reconnecting MCP notification listener."""

import asyncio
from collections import deque
import json
import os
from pathlib import Path
import sys
import time
from typing import Any
import uuid

import httpx

from dimos.core.global_config import global_config

ROBOT = Path(__file__).resolve().parents[2] / "tools" / "robot.py"
LONG_RUNNING = {"move_to", "navigate_with_text", "wait"}
STOP_TOOLS = {
    "begin_exploration": "end_exploration",
    "start_patrol": "stop_patrol",
    "follow_person": "stop_following",
    "look_out_for": "stop_looking_out",
    "move_to": "stop_navigation",
    "navigate_with_text": "stop_navigation",
}
BACKGROUND = {"begin_exploration", "start_patrol", "follow_person", "look_out_for"}


class Runtime:
    def __init__(self, bus: Any) -> None:
        self.bus = bus
        self.generation = uuid.uuid4().hex
        self.operations: dict[str, dict[str, Any]] = {}
        self.completed: dict[str, dict[str, Any]] = {}
        self.command_tasks: dict[str, asyncio.Task] = {}
        self.watch_responses: dict[str, str] = {}
        self.speech_tasks: set[asyncio.Task] = set()
        self.audit: deque[dict[str, Any]] = deque(maxlen=200)
        self.connected = False
        self.last_frame: float | None = None
        self.error: str | None = None
        self.listener: asyncio.Task | None = None

    def record(self, event: str, **fields: Any) -> None:
        self.audit.append({"ts": time.time(), "event": event, **fields})

    def begin(self, tool: str) -> str:
        if not self.connected:
            raise RuntimeError("MCP callback listener is disconnected; retry when connected")
        token = f"frank:{self.generation}:{uuid.uuid4().hex}"
        self.operations[token] = {"tool": tool, "status": "running", "started_ts": time.time()}
        self.record("operation_started", tool=tool, token=token)
        return token

    def complete(self, token: str, status: str, result: Any) -> None:
        op = self.operations.pop(token, None)
        if op is None:
            return
        op.update(status=status, result=result, finished_ts=time.time())
        self.completed[token] = op
        while len(self.completed) > 100:
            self.completed.pop(next(iter(self.completed)))
        self.record("operation_result", tool=op["tool"], operation_id=token, status=status)
        if op["tool"] in LONG_RUNNING or op["tool"] in BACKGROUND:
            self.bus.publish(
                {
                    "type": "tool",
                    "tool": op["tool"],
                    "operation_id": token,
                    "status": status,
                    "result": result,
                    "text": f"{op['tool']}: {status}",
                    "generation": self.generation,
                }
            )

    def finish(self, token: str, failed: bool, result: str) -> None:
        op = self.operations.get(token)
        if op is None:
            return
        try:
            payload = json.loads(result)
        except ValueError:
            payload = {"text": result}
        if not failed and op["tool"] in BACKGROUND:
            op.update(status="running", last_message=result[-1500:])
            self.record("operation_result", tool=op["tool"], status="running")
        else:
            status = "failed" if failed else "succeeded"
            if not failed and op["tool"] == "execute_sport_command":
                status = "accepted"  # Request acknowledgment is not physical completion.
            self.complete(token, status, payload)

    def status(self, token: str) -> dict[str, Any]:
        op = self.operations.get(token) or self.completed.get(token)
        if op is None:
            raise KeyError(token)
        return {"operation_id": token, **op}

    async def command(self, name: str, arguments: dict[str, Any], token: str) -> dict[str, Any]:
        proc = await asyncio.create_subprocess_exec(
            sys.executable,
            str(ROBOT),
            "skill",
            name,
            json.dumps(arguments),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env={**os.environ, "FRANK_OPERATION_TOKEN": token},
        )
        try:
            out, err = await asyncio.wait_for(proc.communicate(), 175)
            try:
                result = json.loads(out.decode().strip().splitlines()[-1])
            except (ValueError, IndexError):
                result = {
                    "isError": True,
                    "content": [
                        {"type": "text", "text": err.decode()[-2000:] or "Invalid robot output"}
                    ],
                }
            if proc.returncode:
                result["isError"] = True
            return result
        finally:
            if proc.returncode is None:
                proc.kill()
                await proc.wait()

    async def run_operation(
        self, token: str, name: str, arguments: dict[str, Any]
    ) -> dict[str, Any]:
        try:
            result = await self.command(name, arguments, token)
            failed = bool(result.get("isError"))
            if failed and name in STOP_TOOLS:
                await self.command(STOP_TOOLS[name], {}, token)
            self.finish(token, failed, json.dumps(result))
            return result
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if name in STOP_TOOLS:
                try:
                    await self.command(STOP_TOOLS[name], {}, token)
                except Exception as stop_error:
                    self.record("operation_cancel_failed", tool=name, message=str(stop_error))
            result = {"isError": True, "content": [{"type": "text", "text": str(exc)}]}
            self.finish(token, True, json.dumps(result))
            return result
        finally:
            self.command_tasks.pop(token, None)

    async def call(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        # Stop tools must work even when the notification listener is disconnected.
        if name in STOP_TOOLS.values():
            owners = [
                token for token, op in self.operations.items() if STOP_TOOLS.get(op["tool"]) == name
            ]
            for token in owners:
                await self.cancel_operation(token)
            if owners:
                return {"content": [{"type": "text", "text": "Cancelled matching operations"}]}
            return await self.command(name, arguments, "stop")
        if name in {*STOP_TOOLS, "execute_sport_command"} and any(
            op["tool"] in STOP_TOOLS for op in self.operations.values()
        ):
            raise RuntimeError(
                "A movement or monitoring operation is active. Cancel it before starting another."
            )
        token = self.begin(name)
        task = asyncio.create_task(self.run_operation(token, name, arguments))
        self.command_tasks[token] = task
        if name in LONG_RUNNING or name in BACKGROUND:
            return {
                "content": [
                    {
                        "type": "text",
                        "text": json.dumps(
                            {
                                "operation_id": token,
                                "status": "running",
                                "message": "Accepted. Completion will arrive as a tool event; this is not success.",
                            }
                        ),
                    }
                ]
            }
        return await asyncio.shield(task)

    async def cancel_operation(self, token: str) -> dict[str, Any]:
        op = self.operations.get(token)
        if op is None:
            return self.status(token)
        op["cancelling"] = True
        task = self.command_tasks.pop(token, None)
        if task is not None:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        stop = STOP_TOOLS.get(op["tool"])
        if stop:
            result = await self.command(stop, {}, token)
            if result.get("isError"):
                self.record("operation_cancel_failed", tool=op["tool"], result=result)
                op.pop("cancelling", None)
                raise RuntimeError(f"Could not stop {op['tool']}: {result}")
        self.complete(token, "cancelled", {"message": "Cancelled by request"})
        return self.status(token)

    def notification(self, frame: dict[str, Any]) -> None:
        self.last_frame = time.time()
        params = frame.get("params") or {}
        token = str(params.get("progressToken", ""))
        if not token.startswith(f"frank:{self.generation}:"):
            return
        op = self.operations.get(token)
        if op is None:
            return
        if op.get("cancelling"):
            return
        method = frame.get("method")
        if method == "dimos/tool_stopped":
            self.complete(
                token,
                "stopped",
                {"message": "Background operation stopped", "last_message": op.get("last_message")},
            )
            return
        elif method == "notifications/progress":
            text = params.get("message") or ""
        else:
            return
        if text:
            op["last_message"] = text
            self.record("tool_callback", tool=op["tool"], message=text)
            self.bus.publish(
                {
                    "type": "tool",
                    "tool": op["tool"],
                    "operation_id": token,
                    "status": "running",
                    "text": text,
                    "generation": self.generation,
                }
            )

    async def listen(self) -> None:
        url = f"http://{global_config.listen_host}:{global_config.mcp_port}/mcp"
        while True:
            try:
                async with httpx.AsyncClient(timeout=httpx.Timeout(45, connect=5)) as client:
                    async with client.stream(
                        "GET", url, headers={"Accept": "text/event-stream"}
                    ) as response:
                        response.raise_for_status()
                        self.connected = True
                        self.error = None
                        self.record("listener_connected")
                        data: list[str] = []
                        async for line in response.aiter_lines():
                            if line.startswith("data:"):
                                data.append(line[5:].lstrip())
                            elif not line and data:
                                self.notification(json.loads("\n".join(data)))
                                data.clear()
            except (httpx.HTTPError, ValueError) as exc:
                self.error = str(exc)
                self.record("listener_error", message=self.error)
            finally:
                self.connected = False
            await asyncio.sleep(2)

    async def start(self) -> None:
        if self.listener is None:
            self.listener = asyncio.create_task(self.listen())

    async def stop(self) -> None:
        errors: list[str] = []
        for token in list(self.operations):
            try:
                await self.cancel_operation(token)
            except Exception as exc:
                errors.append(str(exc))
                self.record("operation_cancel_failed", operation_id=token, message=str(exc))
        for task in list(self.speech_tasks):
            task.cancel()
        if self.speech_tasks:
            await asyncio.gather(*self.speech_tasks, return_exceptions=True)
        self.speech_tasks.clear()
        if self.listener is not None:
            self.listener.cancel()
            try:
                await self.listener
            except asyncio.CancelledError:
                pass
            self.listener = None
            self.connected = False
            self.record("listener_stopped")
        if errors:
            raise RuntimeError("; ".join(errors))

    async def reset(self) -> None:
        await self.stop()
        self.generation = uuid.uuid4().hex
        self.operations.clear()
        self.completed.clear()
        self.watch_responses.clear()
        self.record("runtime_reset", generation=self.generation)
        await self.start()

    def snapshot(self) -> dict[str, Any]:
        return {
            "generation": self.generation,
            "operations": [{"id": token, **op} for token, op in self.operations.items()],
            "listener": {
                "running": self.listener is not None and not self.listener.done(),
                "connected": self.connected,
                "last_frame": self.last_frame,
                "error": self.error,
            },
            "instant_responses": len(self.speech_tasks),
            "audit": list(self.audit),
        }
