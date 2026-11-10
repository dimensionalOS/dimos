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

from typing import Annotated, Any

from fastapi import APIRouter, Query
from pydantic import BaseModel

from experimental.gateway import responses
from experimental.gateway.state import ApiError, FreshQuery, GatewayState

router = APIRouter()


class UploadRequest(BaseModel):
    path: str | None = None
    robotId: str | None = None
    kind: str | None = None
    retry: str | None = None


@router.get("/cloud/account", response_model=responses.Account)
async def account(
    state: GatewayState,
    fresh: FreshQuery = False,
    login_not: Annotated[str | None, Query(alias="loginNot")] = None,
) -> dict[str, Any]:
    """`?loginNot=` answers only once the login's state isn't that one (the login page waits on it instead of polling)."""
    login = await state.uploads.next_login(login_not) if login_not else state.uploads.login_state()
    return {**await state.uploads.account(fresh), "login": login}


@router.post("/cloud/login", response_model=responses.Login)
async def start_login(state: GatewayState) -> dict[str, Any]:
    return await state.uploads.start_login()


@router.delete("/cloud/login", response_model=responses.Account)
async def cancel_login_or_logout(state: GatewayState) -> dict[str, Any]:
    if state.uploads.login_state()["state"] in ("starting", "pending"):
        state.uploads.cancel_login()
        answer = await state.uploads.account(False)
    else:
        answer = await state.uploads.logout()
    return {**answer, "login": state.uploads.login_state()}


@router.get("/cloud/uploads", response_model=responses.Uploads)
async def upload_list(state: GatewayState) -> dict[str, Any]:
    return {**state.uploads.listing(), **state.uploads.uploaded()}


@router.post("/cloud/uploads", response_model=responses.Upload)
async def enqueue(state: GatewayState, request: UploadRequest) -> dict[str, Any]:
    if (request.path is None) == (request.retry is None):
        raise ApiError(400, "give either path or retry")
    if request.path is None:
        assert request.retry is not None
        return state.uploads.retry(request.retry)
    return state.uploads.enqueue(request.path, request.robotId, request.kind)


@router.delete("/cloud/uploads", response_model=responses.Uploads)
async def remove(state: GatewayState, id: str | None = None) -> dict[str, Any]:
    if id is None:
        state.uploads.clear_finished()
    else:
        state.uploads.cancel(id)
    return {**state.uploads.listing(), **state.uploads.uploaded()}
