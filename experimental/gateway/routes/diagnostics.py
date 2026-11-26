# Copyright 2025-2026 Dimensional Inc.
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

"""Session control and evidence under the existing gateway access boundary."""

from __future__ import annotations

import os
from pathlib import Path
from threading import BoundedSemaphore
from typing import Any, Literal

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field
from starlette.concurrency import run_in_threadpool

from dimos.constants import STATE_DIR
from dimos.diagnostics.analysis import analyze, load_bundle
from dimos.diagnostics.capabilities import capabilities as describe_capabilities
from dimos.diagnostics.ingest import WIRE_LIMIT, decode_batch, ingest
from dimos.diagnostics.probe import clock_reply
from dimos.diagnostics.session import Requirement, SessionSettings, SessionStore
from dimos.diagnostics.session_network import SessionNetwork
from dimos.diagnostics.store import DiagnosticsStore
from dimos.diagnostics.topology import topology as build_topology

router = APIRouter(prefix="/diagnostics", tags=["diagnostics"])


def store() -> DiagnosticsStore:
    return DiagnosticsStore(
        Path(os.environ.get("DIMOS_DIAGNOSTICS_DIR") or STATE_DIR / "diagnostics")
    )


@router.get("/capabilities")
def capabilities() -> dict[str, Any]:
    return describe_capabilities()


@router.get("/clock")
def clock() -> dict[str, Any]:
    return clock_reply()


class CaptureControl(BaseModel):
    action: Literal["mark", "flush", "stop"]
    reason: str = Field(default="manual", max_length=512)


class ReadinessRequest(BaseModel):
    requirements: list[Requirement] = Field(min_length=1, max_length=128)


_upload_slots = BoundedSemaphore(4)
network_observer = SessionNetwork()


@router.get("/session")
def active_session() -> dict[str, Any]:
    return {
        "schema": "dimos.diagnostics.session/v1",
        "session": SessionStore(store().root).active(),
    }


@router.put("/session")
def start_session(settings: SessionSettings) -> dict[str, Any]:
    try:
        result = SessionStore(store().root).start(settings)
        network_observer.start(store().root, settings)
        return result
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


@router.post("/{run_id}/control")
def control(run_id: str, request: CaptureControl) -> dict[str, Any]:
    try:
        count = SessionStore(store().root).control(run_id, request.action, request.reason)
        if request.action == "stop":
            network_observer.stop()
        return {"run_id": run_id, "requested": request.action, "collectors": count}
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


@router.post("/{run_id}/readiness")
def readiness(run_id: str, request: ReadinessRequest) -> dict[str, Any]:
    try:
        return SessionStore(store().root).readiness(run_id, request.requirements)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error


@router.post("/browser")
async def browser(request: Request) -> dict[str, Any]:
    if not _upload_slots.acquire(blocking=False):
        raise HTTPException(status_code=429, detail="diagnostics busy; retry this batch")
    try:
        body = bytearray()
        async for chunk in request.stream():
            body.extend(chunk)
            if len(body) > WIRE_LIMIT:
                raise HTTPException(status_code=413, detail="batch too large")
        batch = await run_in_threadpool(
            decode_batch, bytes(body), request.headers.get("content-encoding")
        )
        return await run_in_threadpool(ingest, store().root, batch)
    except (ValueError, KeyError, TypeError, AttributeError) as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    except OSError as error:
        raise HTTPException(status_code=503, detail="diagnostic storage unavailable") from error
    finally:
        _upload_slots.release()


@router.get("/{run_id}/status")
def status(run_id: str) -> list[dict[str, Any]]:
    try:
        return store().statuses(run_id)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error


@router.get("/{run_id}/topology")
def topology(run_id: str) -> dict[str, Any]:
    reports = status(run_id)
    return {
        **build_topology(reports, [e for r in reports for e in r.get("topology", [])]),
        "coverage": "observed and declared transport boundaries; not physical AP topology",
        "processes": [
            {"process": r.get("process"), "at_ns": r.get("at_ns"), "edges": r.get("topology", [])}
            for r in reports
        ],
    }


@router.get("/{run_id}/metrics")
def metrics(run_id: str) -> dict[str, Any]:
    reports = status(run_id)
    return {
        "schema": "dimos.diagnostics.metrics/v1",
        "processes": [
            {
                "process": r.get("process"),
                "at_ns": r.get("at_ns"),
                "freshness": r.get("freshness"),
                "state": r.get("state"),
                "stages": r.get("stages", {}),
                "counts": r.get("counts", {}),
                "observer_drops": r.get("observer_drops", 0),
                "io_errors": r.get("io_errors", 0),
                "process_metrics": r.get("process_metrics", {}),
            }
            for r in reports
        ],
    }


@router.get("/{run_id}/incidents")
def incidents(run_id: str) -> list[dict[str, Any]]:
    try:
        return store().incidents(run_id)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error


def incident_path(run_id: str, process: str, incident: str) -> Path:
    try:
        return store().incident(run_id, process, incident)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    except FileNotFoundError as error:
        raise HTTPException(status_code=404, detail="incident not found") from error


@router.get("/{run_id}/incidents/{process}/{incident}")
def download(run_id: str, process: str, incident: str) -> FileResponse:
    return FileResponse(incident_path(run_id, process, incident), media_type="application/x-ndjson")


@router.get("/{run_id}/incidents/{process}/{incident}/analysis")
def analysis(run_id: str, process: str, incident: str) -> dict[str, Any]:
    manifests, events = load_bundle(incident_path(run_id, process, incident))
    return analyze(manifests, events)
