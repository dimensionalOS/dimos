"""Bridge robot audio to a web client.

  - WebSocket /audio_out  -> sends int16 PCM frames from the Go2 mic
  - GET       /audio_info -> reports the current sample_rate + channels (so
                              consumers know how to decode /audio_out frames)
  - POST      /play       -> uploads a WAV body, plays it on the Go2 speaker

Compose with `unitree-go2-basic`:

    dimos run unitree-go2-basic audio-ws-module

Then:
    ws://127.0.0.1:7781/audio_out                      (mic, binary frames)
    curl http://127.0.0.1:7781/audio_info              (sample_rate, channels)
    curl --data-binary @clip.wav http://127.0.0.1:7781/play
"""

import asyncio
import threading
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.robot.unitree.connection import AudioMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class AudioWsConfig(ModuleConfig):
    port: int = 7781


class AudioWsModule(Module):
    config: AudioWsConfig
    audio: In[AudioMessage]   # robot mic (from GO2Connection.audio)
    audio_in: Out[bytes]      # WAV bytes for robot speaker (-> GO2Connection.audio_in)

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._app = FastAPI()
        self._app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["GET", "POST"],
            allow_headers=["*"],
        )
        self._clients: set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        self._last_format: dict[str, int] | None = None
        self._setup_routes()

    def _setup_routes(self) -> None:
        @self._app.get("/audio_info")
        def audio_info() -> dict[str, Any]:
            if self._last_format is None:
                return {"status": "no frame yet"}
            return {"status": "ok", **self._last_format}

        @self._app.websocket("/audio_out")
        async def audio_out(ws: WebSocket) -> None:
            await ws.accept()
            if self._last_format is not None:
                await ws.send_json({"event": "format", **self._last_format})
            self._clients.add(ws)
            try:
                while True:
                    # Treat any incoming message as keep-alive; ignore content.
                    await ws.receive()
            except WebSocketDisconnect:
                pass
            finally:
                self._clients.discard(ws)

        @self._app.post("/play")
        async def play(req: Request) -> dict[str, str]:
            wav = await req.body()
            if not wav:
                return {"status": "empty"}
            self.audio_in.publish(wav)
            return {"status": "queued", "bytes": str(len(wav))}

    @rpc
    def start(self) -> None:
        super().start()

        self.audio.subscribe(self._on_audio)

        def run() -> None:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            config = uvicorn.Config(
                self._app, host="127.0.0.1", port=self.config.port, log_level="warning"
            )
            self._server = uvicorn.Server(config)
            self._loop.run_until_complete(self._server.serve())

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        logger.info(
            f"audio-ws-module: ws://127.0.0.1:{self.config.port}/audio_out  "
            f"| GET http://127.0.0.1:{self.config.port}/audio_info  "
            f"| POST http://127.0.0.1:{self.config.port}/play"
        )

    def _on_audio(self, msg: AudioMessage) -> None:
        self._last_format = {"sample_rate": msg.sample_rate, "channels": msg.channels}
        if not self._clients or self._loop is None:
            return

        chunk = msg.data

        async def broadcast() -> None:
            stale: list[WebSocket] = []
            for ws in list(self._clients):
                try:
                    await ws.send_bytes(chunk)
                except Exception:
                    stale.append(ws)
            for ws in stale:
                self._clients.discard(ws)

        asyncio.run_coroutine_threadsafe(broadcast(), self._loop)

    @rpc
    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        super().stop()
