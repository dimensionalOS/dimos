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
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
from tempfile import TemporaryDirectory
from uuid import uuid4

import httpx
import pytest
import uvicorn
import zenoh

from dimos.agents.mcp.mcp_server import app as mcp_app
from experimental.gateway.app import create_app
from experimental.gateway.errors import ApiError
from experimental.gateway.events import Bus
from experimental.gateway.routes import agent

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "dimos/agents"
ANSWER = "🤖 This answer comes from the pi-durable agent."
# Only the model is scripted. The agent owns a real store, scheduler, and Chord socket.
DAEMON = """
import { once } from 'node:events';
import { fauxProvider, fauxAssistantMessage } from '@earendil-works/pi-ai';
import { agentPaths } from '#dimos/agents/gateway/config/paths.ts';
import { parseDimosInstall } from '#dimos/agents/gateway/config/environment.ts';
import { ConfigStore } from '#dimos/agents/gateway/config/store.ts';
import { runGateway } from '#dimos/agents/gateway/main.ts';
const paths = agentPaths(process.env);
const faux = fauxProvider({models:[{id:'test', input:['text','image']}],
    tokensPerSecond: 15, tokenSize:{min:1,max:1}});
const {provider, id:modelId} = faux.getModel();
await (await ConfigStore.open(paths.configFile)).update({model:{provider,modelId}});
faux.setResponses([
    fauxAssistantMessage(process.argv[1]),
    async (_context, {signal}) => {
        // Keep the scheduled turn active while the test queues both inputs and aborts it.
        if (!signal.aborted) await once(signal, 'abort');
        return fauxAssistantMessage('');
    },
]);
await runGateway({paths, install:parseDimosInstall(process.env), bundledSkillsDir:'harness/skills',
    env:process.env, providers:[faux.provider]});
"""


@pytest.fixture(scope="module")
def agent_bundle():
    if not shutil.which("node") or not (PACKAGE / "node_modules/typebox").exists():
        pytest.skip("requires the installed on-robot Node agent")
    subprocess.run(["node", "build.mjs"], cwd=PACKAGE, check=True)


@pytest.fixture
async def agent_events():
    loop = asyncio.get_running_loop()
    events = asyncio.Queue()
    cfg = zenoh.Config.from_json5(
        '{mode:"peer",listen:{endpoints:["tcp/127.0.0.1:0"]},scouting:{multicast:{enabled:false}}}'
    )
    with zenoh.open(cfg) as session:

        def receive(sample):
            frame = str(sample.key_expr), json.loads(sample.payload.to_bytes())
            loop.call_soon_threadsafe(events.put_nowait, frame)

        with session.declare_subscriber("test/dimos/events/agent/*", receive):
            yield Bus("test", session.put), events


@pytest.fixture
async def skill_server(monkeypatch, unused_tcp_port):
    monkeypatch.setattr(
        mcp_app.state, "rpc_calls", {"read_temperature": lambda unit, **_: f"21 {unit}"}
    )
    server = uvicorn.Server(
        uvicorn.Config(mcp_app, host="127.0.0.1", port=unused_tcp_port, log_level="error")
    )
    serving = asyncio.create_task(server.serve())
    try:
        yield f"http://127.0.0.1:{unused_tcp_port}/mcp"
    finally:
        server.should_exit = True
        await serving


@pytest.fixture
async def live_agent(state, monkeypatch, skill_server, agent_bundle, agent_events):
    with TemporaryDirectory(prefix="dimos-") as home:
        monkeypatch.setenv("XDG_RUNTIME_DIR", home)
        monkeypatch.setattr(state, "dimos_dir", ROOT)
        bus, events = agent_events
        env = dict(
            PATH=os.environ["PATH"],
            HOME=home,
            XDG_RUNTIME_DIR=home,
            DIMOS_ROOT=str(ROOT),
            DIMOS_PYTHON=sys.executable,
            DIMOS_CLI=str(ROOT / ".venv/bin/dimos"),
            DIMOS_MCP_URL=skill_server,
            DIMOS_RUNS_DIR=home,
            DIMOS_STATE_DIR=home,
        )
        daemon = await asyncio.create_subprocess_exec(
            "node",
            "--import=./test/web-sdk.mjs",
            "--input-type=module",
            "-e",
            DAEMON,
            ANSWER,
            cwd=PACKAGE,
            env=env,
        )
        watcher = asyncio.create_task(agent.watch(ROOT, bus))
        try:
            with socket.socket(socket.AF_UNIX) as listener:
                path = str(Path(home) / "http.sock")
                listener.bind(path)
                listener.listen()
                server = uvicorn.Server(
                    uvicorn.Config(create_app(state, background=False), log_level="error")
                )
                serving = asyncio.create_task(server.serve(sockets=[listener]))
                try:
                    async with httpx.AsyncClient(
                        transport=httpx.AsyncHTTPTransport(uds=path), base_url="http://dimos"
                    ) as http:
                        yield http, events
                finally:
                    server.should_exit = True
                    await serving
        finally:
            watcher.cancel()
            await asyncio.gather(watcher, return_exceptions=True)
            if daemon.returncode is None:
                daemon.terminate()
                try:
                    await asyncio.wait_for(daemon.wait(), 5)
                except asyncio.TimeoutError:
                    daemon.kill()
                    await daemon.wait()


def request_id():
    return {"request": str(uuid4())}


async def test_agent_chat_and_jobs_share_the_daemons_state(live_agent):
    http, events = live_agent
    chunks = []

    async def event(kind, predicate=lambda event: True):
        while True:
            key, frame = await asyncio.wait_for(events.get(), 15)
            if key == "test/dimos/events/agent/chat":
                assert "text" not in (frame.get("turn") or {})
                if "delta" in frame:
                    chunks.append(frame)
            if key == f"test/dimos/events/{kind}" and predicate(frame):
                assert frame["type"] == kind
                return frame

    assert await event("agent/conversations") == {
        "type": "agent/conversations",
        "change": "snapshot",
        "rows": [],
    }
    created = (await http.post("/agent/conversations", json=request_id())).raise_for_status().json()
    cid = created["conversation"]
    room = f"/agent/conversations/{cid}"
    [conversation] = (await http.get("/agent/conversations")).raise_for_status().json()
    assert await event("agent/conversations") == {
        "type": "agent/conversations",
        "change": "added",
        "id": cid,
        "row": conversation,
    }
    draft = {
        "name": "demo",
        "conversation": cid,
        "cron": "0 0 * * *",
        "timeZone": "UTC",
        "action": {"kind": "notify", "text": "scheduled input"},
    }
    assert (await http.post("/agent/jobs", json=draft)).raise_for_status().json()["runs"] == []
    job = await event("agent/jobs", lambda e: e["change"] == "added")
    assert job["name"] == job["row"]["name"] == draft["name"]
    assert job["row"]["conversation"] == cid
    assert job["row"]["action"] == draft["action"]
    assert len((await http.get("/agent/jobs")).json()[0]["upcoming"]) == 3
    assert (await http.put("/agent/jobs/demo", json={**draft, "cron": "5 0 * * *"})).json()[
        "cron"
    ] == "5 0 * * *"
    updated = await event("agent/jobs", lambda e: "cron" in e.get("fields", []))
    assert updated["change"] == "updated"
    assert updated["name"] == draft["name"]
    assert updated["row"]["cron"] == "5 0 * * *"
    image = {
        "type": "image",
        "mimeType": "image/png",
        "data": "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aS2kAAAAASUVORK5CYII=",
    }
    msg = {**request_id(), "text": "describe", "images": [image], "whenBusy": "followUp"}
    sent = (await http.post(f"{room}/submissions", json=msg)).raise_for_status().json()
    assert (await http.post(f"{room}/submissions", json=msg)).json() == sent
    titled = await event("agent/conversations", lambda e: "title" in e.get("fields", []))
    assert titled["change"] == "updated"
    assert titled["id"] == cid
    assert titled["row"]["title"] == msg["text"]
    partial = await event("agent/chat", lambda e: "delta" in e)
    assert partial["conversation"] == cid
    detail = (await http.get(room)).raise_for_status().json()
    assert detail["turn"]["run"]["inputs"] == [sent["submission"]]
    assert detail["turn"]["messageTimestamp"] == partial["messageTimestamp"]
    assert ANSWER.startswith(detail["turn"]["text"])
    receipt = f"{room}/submissions/{sent['submission']}"
    pending = (await http.get(receipt)).raise_for_status().json()
    assert pending["record"]["status"] == "placed"
    assert pending["answer"] is None
    at = next(item for item in detail["transcript"]["items"] if item["kind"] == "input")["images"][
        0
    ]["at"]
    assert (
        await http.get(f"{room}/images/{at['entry']}/{at['message']}/{at['block']}")
    ).json() == image
    committed = await event(
        "agent/chat",
        lambda e: any(item["kind"] == "assistant" for item in e.get("entries", [])),
    )
    assert committed["conversation"] == cid
    finished = await event("agent/chat", lambda e: "turn" in e and e["turn"] is None)
    assert finished["conversation"] == cid
    record = (await http.get(receipt)).raise_for_status().json()
    assert record["record"]["status"] == "done"
    assert record["answer"]["message"]["content"][0]["text"] == ANSWER
    assert record["answer"] in committed["entries"]
    assert len(chunks) > 1
    recovered = detail["turn"]["text"].encode("utf-16-le")
    offset = 0
    for chunk in chunks:
        assert chunk["conversation"] == cid
        assert chunk["messageTimestamp"] == record["answer"]["message"]["timestamp"]
        assert chunk["offset"] == offset
        assert chunk["delta"] and "reset" not in chunk
        data = chunk["delta"].encode("utf-16-le")
        recovered += data[max(0, len(recovered) - offset * 2) :]
        offset += len(data) // 2
    assert "".join(chunk["delta"] for chunk in chunks) == ANSWER
    assert recovered.decode("utf-16-le") == ANSWER
    run = (await http.post("/agent/jobs/demo/runs", json=request_id())).raise_for_status().json()
    assert run == {"kind": "submitted"}
    await event("agent/chat", lambda e: e["conversation"] == cid and e.get("turn"))
    follow_up = {**msg, **request_id(), "images": [], "text": "later"}
    assert (await http.post(f"{room}/submissions", json=follow_up)).status_code == 200
    steer = {**follow_up, **request_id(), "whenBusy": "steer", "text": "use metric units"}
    assert (await http.post(f"{room}/submissions", json=steer)).status_code == 200
    stop = request_id()
    withdrawn = [
        {"mode": "followUp", "text": "later"},
        {"mode": "steer", "text": "use metric units"},
    ]
    assert (await http.post(f"{room}/abort", json=stop)).json() == withdrawn
    assert (await http.post(f"{room}/abort", json=stop)).json() == withdrawn
    listed = (await http.get("/agent/jobs")).raise_for_status().json()
    assert listed[0]["runs"][-1]["outcome"] == run
    skill = {"kind": "skill", "skill": "read_temperature", "arguments": {"unit": "celsius"}}
    edited = await http.put("/agent/jobs/demo", json={**draft, "action": skill})
    assert edited.raise_for_status().json()["action"] == skill
    result = (await http.post("/agent/jobs/demo/runs", json=request_id())).raise_for_status().json()
    assert result == {"kind": "ok", "text": "21 celsius"}
    ran = await event(
        "agent/jobs",
        lambda e: "runs" in e.get("fields", []) and e["row"]["runs"][-1]["outcome"] == result,
    )
    assert ran["name"] == draft["name"]
    assert (await http.get("/agent/jobs")).json()[0]["runs"][-1]["outcome"] == result
    assert (await http.delete("/agent/jobs/demo")).status_code == 200
    assert await event("agent/jobs", lambda e: e["change"] == "removed") == {
        "type": "agent/jobs",
        "change": "removed",
        "name": draft["name"],
    }
    assert (await http.get("/agent/jobs")).json() == []
    other = (await http.post("/agent/conversations", json=request_id())).json()["conversation"]
    assert (
        await http.get(f"/agent/conversations/{other}/submissions/{sent['submission']}")
    ).status_code == 404
    assert (
        await http.get(f"/agent/conversations/{other}/images/{at['entry']}/0/{at['block']}")
    ).status_code == 404
    assert (await http.get("/agent/conversations/999999")).status_code == 404


def test_checkout_without_agent_client(client):
    response = client.get("/agent/conversations")
    assert response.status_code == 503
    assert "dimos agent version" in response.json()["error"]


async def test_offline_agent_and_gateway_shutdown(agent_bundle, monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    with pytest.raises(ApiError, match="offline") as error:
        await agent.call_sdk(ROOT, "jobs")
    assert error.value.status == 503
    async with agent.sdk_process(ROOT, "watch") as watcher:
        assert watcher.stdin is not None
        watcher.stdin.close()
        assert await asyncio.wait_for(watcher.wait(), 5) == 0
