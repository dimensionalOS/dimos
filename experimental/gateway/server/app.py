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
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import (
    JSONResponse,
)
from starlette.exceptions import HTTPException as StarletteHTTPException

from experimental.gateway.server.endpoints import register_endpoints
from experimental.gateway.server.openapi import document, operation_id
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import (
    events,
)
from experimental.gateway.utils.blueprint_watch import BlueprintWatch
from experimental.gateway.utils.discovery import Discovery
from experimental.gateway.utils.http import ApiError
from experimental.gateway.utils.jobs import Jobs


class GatewayApp(FastAPI):
    def openapi(self) -> dict[str, Any]:
        if self.openapi_schema is None:
            self.openapi_schema = document(self)
        return self.openapi_schema


def create_app(state: ServerState, background: bool = True) -> FastAPI:
    discovered = state.discovery = state.discovery or Discovery(
        state.dimos_dir, lambda event: state.bus.send(event)
    )
    state.jobs = state.jobs or Jobs(
        lambda event: state.bus.send(event), lambda key, payload: state.bus.publish(key, payload)
    )

    async def list_in_child() -> list[dict[str, Any]]:
        answer = await discovered.child_answer("list")
        if "error" in answer:
            raise RuntimeError(answer["error"])
        return list(answer["blueprints"])

    def blueprints_changed(_: list[dict[str, Any]], added: list[str], removed: list[str]) -> None:
        state.cache.forget("list")
        state.bus.send({"type": "blueprints", "added": added, "removed": removed})

    watch = state.watch = state.watch or BlueprintWatch(
        state.dimos_dir,
        list_in_child,
        blueprints_changed,
        lambda: discovered.refresh("files changed"),
    )

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        if background:
            state.background += [
                asyncio.create_task(events.watch_launch(state.bus)),
                asyncio.create_task(state.uploads.work()),
                asyncio.create_task(discovered.run()),
                asyncio.create_task(watch.run()),
            ]
        yield
        state.uploads.shutdown()
        for task in state.background:
            task.cancel()

    app = GatewayApp(
        title="dimos gateway",
        lifespan=lifespan,
        openapi_url=None,
        docs_url=None,
        redoc_url=None,
        generate_unique_id_function=operation_id,
        separate_input_output_schemas=False,
    )
    app.state.server = state

    @app.exception_handler(ApiError)
    async def api_error(_: Request, error: ApiError) -> JSONResponse:
        return JSONResponse({"error": str(error)}, status_code=error.status)

    @app.exception_handler(StarletteHTTPException)
    async def http_error(request: Request, error: StarletteHTTPException) -> JSONResponse:
        message = (
            f"no such route: {request.url.path}" if error.status_code == 404 else str(error.detail)
        )
        return JSONResponse({"error": message}, status_code=error.status_code)

    @app.exception_handler(RequestValidationError)
    async def bad_request(_: Request, error: RequestValidationError) -> JSONResponse:
        problems = "; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in error.errors())
        return JSONResponse({"error": f"bad request: {problems}"}, status_code=400)

    @app.exception_handler(Exception)
    async def failed(_: Request, error: Exception) -> JSONResponse:
        return JSONResponse({"error": str(error) or type(error).__name__}, status_code=500)

    register_endpoints(app, state)
    return app
