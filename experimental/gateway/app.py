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

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from experimental.gateway import events
from experimental.gateway.routes import blueprints, cloud, discovery, runs, server, skills
from experimental.gateway.state import API_VERSION, ApiError, State, site_dirs


def create_app(state: State, background: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        if background:
            watch = events.watch_blueprints(
                state.dimos_dir, site_dirs(state.dimos_dir), state.relist, state.blueprints_changed
            )
            state.background += [
                asyncio.create_task(events.watch_launch(state.bus)),
                asyncio.create_task(state.uploads.work()),
                asyncio.create_task(watch),
            ]
        yield
        state.uploads.shutdown()
        for task in state.background:
            task.cancel()

    app = FastAPI(
        title="dimos gateway",
        version=API_VERSION,
        lifespan=lifespan,
        openapi_url="/dimos/openapi.json",
        docs_url=None,
        redoc_url=None,
    )
    app.state.gateway = state

    @app.exception_handler(ApiError)
    async def api_error(_: Request, error: ApiError) -> JSONResponse:
        return JSONResponse({"error": str(error)}, status_code=error.status)

    @app.exception_handler(StarletteHTTPException)
    async def http_error(request: Request, error: StarletteHTTPException) -> JSONResponse:
        missing = error.status_code == 404
        message = f"no such route: {request.url.path}" if missing else str(error.detail)
        return JSONResponse({"error": message}, status_code=error.status_code)

    @app.exception_handler(RequestValidationError)
    async def bad_request(_: Request, error: RequestValidationError) -> JSONResponse:
        problems = "; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in error.errors())
        return JSONResponse({"error": f"bad request: {problems}"}, status_code=400)

    @app.exception_handler(Exception)
    async def failed(_: Request, error: Exception) -> JSONResponse:
        return JSONResponse({"error": str(error) or type(error).__name__}, status_code=500)

    for module in (server, blueprints, runs, cloud, skills, discovery):
        app.include_router(module.router)
    return app
