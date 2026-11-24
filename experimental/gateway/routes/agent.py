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
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager, suppress
import json
from pathlib import Path
from typing import Annotated, Any, Literal

from fastapi import APIRouter, Path as PathParam
from typing_extensions import NotRequired, TypedDict

from dimos.utils.logging_config import setup_logger
from experimental.gateway import constants, responses
from experimental.gateway.errors import ApiError
from experimental.gateway.events import Bus
from experimental.gateway.state import GatewayState

logger = setup_logger()
router = APIRouter(prefix="/agent")
Id = Annotated[int, PathParam(gt=0)]
Index = Annotated[int, PathParam(ge=0)]


class RequestId(TypedDict):
    request: str


class Create(RequestId):
    cwd: NotRequired[str | None]


class Image(TypedDict):
    type: Literal["image"]
    mimeType: str
    data: str


class Message(RequestId):
    text: str
    images: list[Image]
    whenBusy: Literal["steer", "followUp"]


class Notify(TypedDict):
    kind: Literal["notify"]
    text: str


class Skill(TypedDict):
    kind: Literal["skill"]
    skill: str
    arguments: dict[str, Any]


class Schedule(TypedDict):
    cron: str
    timeZone: str | None
    conversation: int
    action: Notify | Skill


class NewSchedule(Schedule):
    name: str


@asynccontextmanager
async def sdk_process(
    checkout: Path, operation: str, *args: str
) -> AsyncIterator[asyncio.subprocess.Process]:
    script = checkout / "dimos/agents/dist/cli.js"
    if not script.is_file():
        raise ApiError(503, "The DimOS Agent client is missing. Build it with dimos agent version")
    try:
        proc = await asyncio.create_subprocess_exec(
            "node",
            str(script.resolve()),
            "sdk",
            operation,
            *args,
            cwd=checkout,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            limit=constants.AGENT_EVENT_BYTES,
        )
    except OSError as exc:
        raise ApiError(503, f"Could not start the DimOS Agent client: {exc}") from exc
    try:
        yield proc
    finally:
        if proc.returncode is None:
            with suppress(ProcessLookupError):
                proc.kill()
        await proc.communicate()


async def call_sdk(
    checkout: Path, operation: str, args: Any = None, conversation: int | None = None
) -> Any:
    async with sdk_process(
        checkout, operation, *([] if conversation is None else [str(conversation)])
    ) as proc:
        try:
            output, _ = await asyncio.wait_for(
                proc.communicate(json.dumps(args or {}).encode()), constants.AGENT_CALL_TIMEOUT_S
            )
        except asyncio.TimeoutError as exc:
            raise ApiError(504, "DimOS Agent timed out; check the outcome before retrying") from exc
    if proc.returncode or not output:
        raise ApiError(503, "The DimOS Agent client failed; check the gateway log")
    reply = json.loads(output)
    if "error" in reply:
        statuses = {"invalid": 400, "viewing": 403, "not_found": 404, "running": 409, "stale": 409}
        raise ApiError(reply.get("status", statuses.get(reply.get("refusal"), 503)), reply["error"])
    return reply["value"]


async def watch(checkout: Path, bus: Bus) -> None:
    """Relay the agent's own live state, including turns started by a scheduler or another client."""
    while True:
        try:
            async with sdk_process(checkout, "watch") as proc:
                assert proc.stdout is not None
                async for line in proc.stdout:
                    event = json.loads(line)
                    suffix = f"/{event['conversation']}" if event["type"] == "agent-chat" else ""
                    bus.send(event, suffix=suffix)
        except (ApiError, ValueError) as exc:
            logger.debug("DimOS Agent watcher unavailable", error=str(exc))
        await asyncio.sleep(constants.AGENT_RECONNECT_S)


@router.get("/conversations", response_model=list[responses.AgentConversationRow])
async def conversations(state: GatewayState) -> Any:
    return await call_sdk(state.dimos_dir, "conversations")


@router.post("/conversations", response_model=responses.AgentConversationId)
async def create_conversation(state: GatewayState, body: Create) -> Any:
    return await call_sdk(state.dimos_dir, "createConversation", {"cwd": None, **body})


@router.get("/conversations/{id}", response_model=responses.AgentConversation)
async def conversation(state: GatewayState, id: Id) -> Any:
    return await call_sdk(state.dimos_dir, "conversation", conversation=id)


@router.post("/conversations/{id}/submissions", response_model=responses.AgentSubmissionId)
async def send(state: GatewayState, id: Id, body: Message) -> Any:
    return await call_sdk(state.dimos_dir, "send", body, id)


@router.get(
    "/conversations/{id}/submissions/{submission}", response_model=responses.AgentSubmission
)
async def submission_status(state: GatewayState, id: Id, submission: Id) -> Any:
    return await call_sdk(state.dimos_dir, "submission", {"id": submission}, id)


@router.post("/conversations/{id}/abort", response_model=list[responses.AgentQueuedText])
async def abort(state: GatewayState, id: Id, body: RequestId) -> Any:
    return await call_sdk(state.dimos_dir, "abort", body, id)


@router.get("/conversations/{id}/images/{entry}/{message}/{block}", response_model=Image)
async def image(state: GatewayState, id: Id, entry: Id, message: Index, block: Index) -> Any:
    address = {"entry": entry, "message": message, "block": block}
    return await call_sdk(state.dimos_dir, "image", {"at": address}, id)


@router.get("/jobs", response_model=list[responses.AgentJob])
async def jobs(state: GatewayState) -> Any:
    return await call_sdk(state.dimos_dir, "jobs")


@router.post("/jobs", response_model=responses.AgentJob)
async def create_job(state: GatewayState, body: NewSchedule) -> Any:
    return await call_sdk(state.dimos_dir, "createJob", body)


@router.put("/jobs/{name}", response_model=responses.AgentJob)
async def update_job(state: GatewayState, name: str, body: Schedule) -> Any:
    return await call_sdk(state.dimos_dir, "updateJob", {**body, "name": name})


@router.delete("/jobs/{name}")
async def delete_job(state: GatewayState, name: str) -> None:
    await call_sdk(state.dimos_dir, "deleteJob", {"name": name})


@router.post("/jobs/{name}/runs", response_model=responses.AgentJobOutcome)
async def run_job(state: GatewayState, name: str, body: RequestId) -> Any:
    return await call_sdk(state.dimos_dir, "runJob", {**body, "name": name})
