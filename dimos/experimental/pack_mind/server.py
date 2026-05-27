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

"""PACK MIND — web demo backend (S2).

Streams two fog-of-war exploration sims (independent + shared) over a WebSocket to
the Three.js frontend, and accepts kill/reset controls.

    uv run python -m dimos.experimental.pack_mind.server
    # then open http://localhost:8000
"""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from dimos.experimental.pack_mind.explore_sim import build_explore, build_explore_building

# PACK_MIND_MAP=building swaps the synthetic maze for a real DimOS SLAM floor plan.
_BUILD = build_explore_building if os.environ.get("PACK_MIND_MAP") == "building" else build_explore

_STATIC = Path(__file__).parent / "static"
_PORT = int(os.environ.get("PACK_MIND_PORT", "8000"))
_STEPS_PER_FRAME = 5
_FPS = 18
_HOLD_FRAMES = 70  # hold the finished maze before auto-replaying

app = FastAPI(title="PACK MIND — Fog of War")
app.mount("/static", StaticFiles(directory=str(_STATIC)), name="static")


@app.get("/")
def index() -> FileResponse:
    return FileResponse(str(_STATIC / "explore.html"))


def _new_sims(seed: int, n_dogs: int) -> dict:
    return {
        "independent": _BUILD(False, seed, n_dogs),
        "shared": _BUILD(True, seed, n_dogs),
    }


@app.websocket("/ws")
async def ws(websocket: WebSocket) -> None:
    await websocket.accept()
    cfg = {"seed": 0, "n_dogs": 3}
    sims = _new_sims(cfg["seed"], cfg["n_dogs"])
    controls: asyncio.Queue = asyncio.Queue()

    async def receiver() -> None:
        try:
            while True:
                controls.put_nowait(json.loads(await websocket.receive_text()))
        except Exception:
            pass

    rt = asyncio.create_task(receiver())
    done_frames = 0
    try:
        while True:
            while not controls.empty():
                c = controls.get_nowait()
                if c.get("cmd") == "reset":
                    cfg["seed"] = int(c.get("seed", cfg["seed"]))
                    sims = _new_sims(cfg["seed"], cfg["n_dogs"])
                    done_frames = 0
                elif c.get("cmd") == "kill":
                    dog = int(c.get("dog", 0))
                    for s in sims.values():
                        if dog < len(s.dogs):
                            s.set_online(dog, False)

            if all(s.all_done() for s in sims.values()):
                done_frames += 1
                if done_frames > _HOLD_FRAMES:
                    sims = _new_sims(cfg["seed"], cfg["n_dogs"])
                    done_frames = 0
            else:
                for s in sims.values():
                    for _ in range(_STEPS_PER_FRAME):
                        if not s.all_done():
                            s.step()

            await websocket.send_text(json.dumps({k: v.state() for k, v in sims.items()}))
            await asyncio.sleep(1 / _FPS)
    except WebSocketDisconnect:
        pass
    finally:
        rt.cancel()


if __name__ == "__main__":
    import uvicorn

    print(f"PACK MIND web demo → http://localhost:{_PORT}")
    uvicorn.run(app, host="0.0.0.0", port=_PORT)
