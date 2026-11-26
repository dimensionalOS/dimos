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

from typing import Annotated, Any, Literal

from fastapi import APIRouter, Path as PathParam
from typing_extensions import NotRequired, TypedDict

from experimental.gateway import responses
from experimental.gateway.state import GatewayState, State

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


async def call_sdk(
    state: State, operation: str, args: Any = None, conversation: int | None = None
) -> Any:
    return await state.agent.call(state.dimos_dir, operation, args, conversation)


@router.get("/conversations", response_model=list[responses.AgentConversationRow])
async def conversations(state: GatewayState) -> Any:
    return await call_sdk(state, "conversations")


@router.post("/conversations", response_model=responses.AgentConversationId)
async def create_conversation(state: GatewayState, body: Create) -> Any:
    return await call_sdk(state, "createConversation", {"cwd": None, **body})


@router.get("/conversations/{id}", response_model=responses.AgentConversation)
async def conversation(state: GatewayState, id: Id) -> Any:
    return await call_sdk(state, "conversation", conversation=id)


@router.post("/conversations/{id}/submissions", response_model=responses.AgentSubmissionId)
async def send(state: GatewayState, id: Id, body: Message) -> Any:
    return await call_sdk(state, "send", body, id)


@router.post("/conversations/{id}/abort", response_model=list[responses.AgentQueuedText])
async def abort(state: GatewayState, id: Id, body: RequestId) -> Any:
    return await call_sdk(state, "abort", body, id)


@router.get("/conversations/{id}/images/{entry}/{message}/{block}", response_model=Image)
async def image(state: GatewayState, id: Id, entry: Id, message: Index, block: Index) -> Any:
    address = {"entry": entry, "message": message, "block": block}
    return await call_sdk(state, "image", {"at": address}, id)


@router.get("/jobs", response_model=list[responses.AgentJob])
async def jobs(state: GatewayState) -> Any:
    return await call_sdk(state, "jobs")


@router.post("/jobs", response_model=responses.AgentJob)
async def create_job(state: GatewayState, body: NewSchedule) -> Any:
    return await call_sdk(state, "createJob", body)


@router.put("/jobs/{name}", response_model=responses.AgentJob)
async def update_job(state: GatewayState, name: str, body: Schedule) -> Any:
    return await call_sdk(state, "updateJob", {**body, "name": name})


@router.delete("/jobs/{name}")
async def delete_job(state: GatewayState, name: str) -> None:
    await call_sdk(state, "deleteJob", {"name": name})


@router.post("/jobs/{name}/runs", response_model=responses.AgentJobOutcome)
async def run_job(state: GatewayState, name: str, body: RequestId) -> Any:
    return await call_sdk(state, "runJob", {**body, "name": name})
