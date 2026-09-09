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

"""FRANK chat server: the phone talks to /api, the agent talks to /agent.

uv run python dimos/experimental/frank/app/server.py     # http://127.0.0.1:7790
"""

import asyncio
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
import os
from pathlib import Path
import sys
import time
from typing import Any

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))

from dotenv import load_dotenv
from fastapi import Depends, FastAPI, File, HTTPException, Request, UploadFile
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from lib.events import EventBus, Waiters
from lib.ops import router as ops_router, stop_loop, stop_motion
from lib.runtime import Runtime
from lib.scheduler import Scheduler
from lib.store import Store, selfie_path
from lib.world import SEEN_GAP_S, World, snapshot
from pydantic import BaseModel, Field
import requests
import uvicorn

SKILL_DIR = APP_DIR.parent
load_dotenv(SKILL_DIR / ".env")  # same key file speak.py uses
load_dotenv()
STT_URL = "https://api.elevenlabs.io/v1/speech-to-text"
STT_MODEL = "scribe_v1"

LOOPBACK = {"127.0.0.1", "::1", "localhost"}
MAX_PHONE_WAIT_S = 30.0
MAX_AGENT_WAIT_S = 120.0

store = Store()
bus = EventBus(store)
runtime = Runtime(bus)
waiters = Waiters()
scheduler = Scheduler(store, bus)
world = World(on_sighting=store.save_last_sighting)
world.load(store.last_sightings())  # positions survive a restart; `in_view` starts false
# A room full of people coming and going is not the agent working; only a `found` (the person it
# asked to be told about) counts as activity and holds the idle scheduler off.
bus.on_delivery = lambda event: None if event.get("type") == "seen" else scheduler.touch()


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    await runtime.start()
    task = asyncio.create_task(scheduler.run())
    try:
        yield
    finally:
        task.cancel()
        await runtime.stop()


app = FastAPI(title="FRANK", lifespan=lifespan)


# --- request bodies -----------------------------------------------------


class NewPerson(BaseModel):
    name: str = Field(min_length=1, max_length=60)
    selfie: str | None = None


class NewMessage(BaseModel):
    text: str = Field(min_length=1, max_length=2000)


class AgentMessage(BaseModel):
    person_id: str
    text: str = Field(min_length=1, max_length=2000)


class Sighting(BaseModel):
    person_id: str
    confidence: float
    pose: dict[str, float] | None = None
    # everything below is what the rolling watcher adds; all optional so the old shape still works
    x: float | None = None
    y: float | None = None
    bearing_deg: float | None = None
    range_m: float | None = None
    face_px: int | None = None
    in_view: bool = True


class Watch(BaseModel):
    person_id: str
    say: str | None = Field(default=None, max_length=300)


class TaskDone(BaseModel):
    outcome: str
    note: str | None = None


# --- helpers ------------------------------------------------------------


def agent_only(request: Request) -> None:
    """Loopback is trusted; anything else needs FRANK_AGENT_TOKEN as a bearer token.

    A tunnel proxy on this machine (Tailscale Funnel, ngrok) also connects from loopback, but it
    adds X-Forwarded-For, so a forwarded request is never treated as local."""
    host = request.client.host if request.client else ""
    if host in LOOPBACK and "x-forwarded-for" not in request.headers:
        return
    token = os.environ.get("FRANK_AGENT_TOKEN")
    header = request.headers.get("authorization", "")
    if token and header == f"Bearer {token}":
        return
    raise HTTPException(403, "agent endpoints are localhost only")


def need_person(person_id: str) -> Any:
    person = store.get_person(person_id)
    if person is None:
        raise HTTPException(404, "no such person")
    return person


def clamp(value: float, high: float) -> float:
    return max(0.0, min(value, high))


# --- phone side ---------------------------------------------------------


@app.get("/")
def index() -> FileResponse:
    return FileResponse(APP_DIR / "static" / "index.html")


@app.post("/api/people")
def create_person(body: NewPerson) -> dict[str, Any]:
    person = store.add_person(body.name.strip(), body.selfie)
    scheduler.touch()
    bus.publish({"type": "enrolled", "person_id": person["person_id"], "name": person["name"]})
    return {"person_id": person["person_id"], "name": person["name"]}


@app.get("/api/people/{person_id}")
def read_person(person_id: str) -> dict[str, Any]:
    p = need_person(person_id)
    return {"person_id": p["person_id"], "name": p["name"], "created_at": p["created_at"]}


@app.get("/api/people/{person_id}/history")
def read_history(person_id: str, limit: int = 50) -> dict[str, Any]:
    """The last few messages, so a returning phone can redraw the chat it left."""
    need_person(person_id)
    return {"messages": store.last_messages(person_id, max(1, min(limit, 200)))}


@app.post("/api/people/{person_id}/messages")
def post_message(person_id: str, body: NewMessage) -> dict[str, Any]:
    person = need_person(person_id)
    msg = store.add_message(person_id, "person", body.text.strip())
    scheduler.touch()
    waiters.notify(person_id)
    bus.publish(
        {
            "type": "chat",
            "person_id": person_id,
            "name": person["name"],
            "text": msg["text"],
        }
    )
    return msg


@app.get("/api/people/{person_id}/messages")
async def get_messages(
    request: Request, person_id: str, after: int = 0, wait: float = 0
) -> dict[str, Any]:
    need_person(person_id)
    deadline = time.monotonic() + clamp(wait, MAX_PHONE_WAIT_S)
    while True:
        messages = store.messages_after(person_id, after)
        if messages:
            return {"messages": messages, "generation": runtime.generation}
        if await request.is_disconnected():
            return {"messages": [], "generation": runtime.generation}
        left = deadline - time.monotonic()
        if left <= 0:
            return {"messages": [], "generation": runtime.generation}
        await waiters.wait(person_id, min(left, 0.5))


@app.post("/api/people/{person_id}/transcribe")
async def transcribe(person_id: str, audio: UploadFile = File(...)) -> dict[str, str]:
    """Fallback speech-to-text for browsers without the Web Speech API. The phone records with
    MediaRecorder and posts the blob; ElevenLabs turns it into the text the person meant to type."""
    need_person(person_id)
    key = os.environ.get("ELEVENLABS_API_KEY")
    if not key:
        raise HTTPException(503, "no speech-to-text configured")
    data = await audio.read()
    if not data:
        raise HTTPException(400, "empty recording")
    try:
        r = requests.post(
            STT_URL,
            headers={"xi-api-key": key},
            files={
                "file": (audio.filename or "audio.webm", data, audio.content_type or "audio/webm")
            },
            data={"model_id": STT_MODEL},
            timeout=60,
        )
        r.raise_for_status()
    except requests.RequestException as e:
        raise HTTPException(502, f"speech-to-text failed: {e}")
    return {"text": (r.json().get("text") or "").strip()}


def forget_everywhere(person_id: str) -> None:
    runtime.watch_responses.pop(person_id, None)
    """Erase a person from storage and from the live world state."""
    store.forget(person_id)
    world.forget(person_id)


@app.post("/api/people/{person_id}/forget")
def forget_person(person_id: str) -> dict[str, bool]:
    need_person(person_id)
    forget_everywhere(person_id)
    return {"ok": True}


# --- agent side ---------------------------------------------------------


@app.post("/agent/session/start", dependencies=[Depends(agent_only)])
async def agent_session_start(body: dict[str, Any]) -> dict[str, Any]:
    """Every loop launch, including CLI launches, starts a clean conversation runtime."""
    killed = await asyncio.to_thread(stop_loop, exclude_pid=int(body["pid"]))
    stopped = await asyncio.to_thread(stop_motion)
    if not stopped["ok"]:
        raise HTTPException(503, stopped["output"])
    await reset_runtime(clear_history=True)
    runtime.record(
        "session_started", session=str(body["session"]), pid=int(body["pid"]), killed=killed
    )
    return {"ok": True, "generation": runtime.generation}


@app.post("/agent/operations", dependencies=[Depends(agent_only)])
def agent_operation(body: dict[str, Any]) -> dict[str, Any]:
    if body.get("action") == "start":
        try:
            return {"token": runtime.begin(str(body["tool"]))}
        except RuntimeError as exc:
            raise HTTPException(503, str(exc)) from exc
    runtime.finish(str(body["token"]), bool(body.get("failed")), str(body.get("result", "")))
    return {"ok": True}


class ToolCall(BaseModel):
    name: str
    arguments: dict[str, Any] = Field(default_factory=dict)


@app.post("/agent/tools/call", dependencies=[Depends(agent_only)])
async def agent_tool_call(body: ToolCall) -> dict[str, Any]:
    try:
        return await runtime.call(body.name, body.arguments)
    except RuntimeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.get("/agent/operations/{token}", dependencies=[Depends(agent_only)])
def agent_operation_status(token: str) -> dict[str, Any]:
    try:
        return runtime.status(token)
    except KeyError as exc:
        raise HTTPException(404, "Unknown operation in this session") from exc


@app.post("/agent/operations/{token}/cancel", dependencies=[Depends(agent_only)])
async def agent_operation_cancel(token: str) -> dict[str, Any]:
    try:
        return await runtime.cancel_operation(token)
    except KeyError as exc:
        raise HTTPException(404, "Unknown operation in this session") from exc
    except RuntimeError as exc:
        raise HTTPException(503, str(exc)) from exc


@app.get("/agent/events", dependencies=[Depends(agent_only)])
async def agent_events(request: Request, wait: float = 0) -> dict[str, Any]:
    return await bus.poll(clamp(wait, MAX_AGENT_WAIT_S), request.is_disconnected)


@app.post("/agent/send", dependencies=[Depends(agent_only)])
def agent_send(body: AgentMessage) -> dict[str, Any]:
    need_person(body.person_id)
    msg = store.add_message(body.person_id, "frank", body.text)
    scheduler.touch()
    waiters.notify(body.person_id)
    return msg


@app.get("/agent/people", dependencies=[Depends(agent_only)])
def agent_people() -> list[dict[str, Any]]:
    return store.people_overview()


@app.get("/agent/people/{person_id}/history", dependencies=[Depends(agent_only)])
def agent_history(person_id: str, since_minutes: float = 60) -> dict[str, Any]:
    need_person(person_id)
    return {"messages": store.history_since(person_id, since_minutes)}


@app.get("/agent/people/{person_id}/selfie.jpg", dependencies=[Depends(agent_only)])
def agent_selfie(person_id: str) -> FileResponse:
    need_person(person_id)
    path = selfie_path(person_id)
    if not path.exists():
        raise HTTPException(404, "no selfie on file")
    return FileResponse(path, media_type="image/jpeg")


async def say_on_match(person_id: str, text: str) -> None:
    """Run the exact preselected response without invoking an LLM or a shell."""
    runtime.record("instant_response_started", person_id=person_id, message=text)
    store.add_message(person_id, "frank", text)
    waiters.notify(person_id)
    proc = None
    try:
        proc = await asyncio.create_subprocess_exec(
            sys.executable,
            str(SKILL_DIR / "speak.py"),
            text,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.PIPE,
        )
        _, error = await asyncio.wait_for(proc.communicate(), timeout=60)
        if proc.returncode:
            raise RuntimeError(error.decode(errors="replace")[-500:])
        runtime.record("instant_response_spoken", person_id=person_id, message=text)
    except asyncio.CancelledError:
        if proc is not None and proc.returncode is None:
            proc.kill()
            await proc.wait()
        raise
    except Exception as exc:
        if proc is not None and proc.returncode is None:
            proc.kill()
            await proc.wait()
        runtime.record("instant_response_failed", person_id=person_id, message=str(exc))
        bus.publish(
            {
                "type": "tool",
                "tool": "instant_response",
                "text": f"Automatic greeting failed: {exc}",
            }
        )


@app.post("/agent/sightings", dependencies=[Depends(agent_only)])
async def agent_sighting(body: Sighting) -> dict[str, Any]:
    person = need_person(body.person_id)
    fields = body.model_dump(exclude={"person_id", "pose"})
    kind = None
    if body.in_view:
        # decided before the sighting lands, so "how long since we last saw them" is still true
        gap = float(scheduler.config.values.get("seen_event_gap_s") or SEEN_GAP_S)
        kind = world.event_for(body.person_id, seen_gap_s=gap)
    world.record(body.person_id, fields)
    if body.in_view:
        store.record_sighting(body.person_id, body.pose)
    immediate = runtime.watch_responses.pop(body.person_id, None) if kind == "found" else None
    if immediate:
        task = asyncio.create_task(say_on_match(body.person_id, immediate))
        runtime.speech_tasks.add(task)
        task.add_done_callback(runtime.speech_tasks.discard)
    if kind:
        bus.publish(
            {
                "type": kind,
                "person_id": body.person_id,
                "name": person["name"],
                "x": body.x,
                "y": body.y,
                "bearing_deg": body.bearing_deg,
                "range_m": body.range_m,
                "automatic_response": immediate,
            }
        )
    return {"ok": True, "event": kind}


@app.get("/agent/world", dependencies=[Depends(agent_only)])
def agent_world() -> dict[str, Any]:
    return snapshot(world, store.people_overview())


@app.post("/agent/watch", dependencies=[Depends(agent_only)])
def agent_watch(body: Watch) -> dict[str, Any]:
    need_person(body.person_id)
    if body.say:
        runtime.watch_responses[body.person_id] = body.say
    else:
        runtime.watch_responses.pop(body.person_id, None)
    runtime.record("watch_armed", person_id=body.person_id, message=body.say or "Notify only")
    return {
        "ok": True,
        "person_id": body.person_id,
        "expires_at": world.watch(body.person_id),
        "say": body.say,
    }


@app.delete("/agent/watch/{person_id}", dependencies=[Depends(agent_only)])
def agent_unwatch(person_id: str) -> dict[str, Any]:
    runtime.watch_responses.pop(person_id, None)
    return {"ok": world.unwatch(person_id), "person_id": person_id}


@app.get("/agent/watch", dependencies=[Depends(agent_only)])
def agent_watches() -> dict[str, Any]:
    now = time.time()
    return {
        "watches": [
            {"person_id": pid, "expires_in_s": round(exp - now, 1)}
            for pid, exp in sorted(world.watches(now).items())
        ]
    }


@app.post("/agent/tasks/{task_id}/done", dependencies=[Depends(agent_only)])
def agent_task_done(task_id: str, body: TaskDone) -> JSONResponse:
    ok = scheduler.close_task(task_id, body.outcome, body.note)
    scheduler.touch()
    if not ok:
        return JSONResponse({"ok": False, "error": "no such task"}, status_code=404)
    return JSONResponse({"ok": True})


async def reset_runtime(clear_history: bool = False) -> None:
    await runtime.stop()
    store.write("DELETE FROM events")
    store.write("DELETE FROM tasks")
    if clear_history:
        store.write("DELETE FROM messages")
        store.write("UPDATE people SET last_seen_pose = NULL")
        world.load({})
    for person_id in world.watches():
        world.unwatch(person_id)
    scheduler.touch()
    await runtime.reset()


# the operator dashboard, laptop only (agent_only refuses anything forwarded by the funnel)
app.include_router(
    ops_router(
        store,
        lambda: snapshot(world, store.people_overview()),
        [Depends(agent_only)],
        forget_everywhere,
        runtime,
        reset_runtime,
        world,
    )
)
app.mount("/static", StaticFiles(directory=APP_DIR / "static"), name="static")


def main() -> None:
    port = int(os.environ.get("FRANK_PORT", "7790"))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")


if __name__ == "__main__":
    main()
