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
import queue
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
        # Cross-thread handoff: subscriber thread enqueues, broadcast task drains.
        self._frame_queue: queue.Queue[bytes] = queue.Queue(maxsize=200)
        self._clients: set[WebSocket] = set()
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        self._last_format: dict[str, int] | None = None
        self._stop_event = threading.Event()
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
                    # Server -> client only; sink any inbound messages.
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

        @self._app.on_event("startup")
        async def _spawn_broadcaster() -> None:
            asyncio.create_task(self._broadcast_loop())

    async def _broadcast_loop(self) -> None:
        """Drain the cross-thread queue and fan out to WebSocket clients."""
        loop = asyncio.get_running_loop()
        while not self._stop_event.is_set():
            try:
                chunk = await loop.run_in_executor(None, self._frame_queue.get, True, 0.5)
            except queue.Empty:
                continue
            if not self._clients:
                continue
            stale: list[WebSocket] = []
            for ws in list(self._clients):
                try:
                    await ws.send_bytes(chunk)
                except Exception:
                    stale.append(ws)
            for ws in stale:
                self._clients.discard(ws)

    @rpc
    def start(self) -> None:
        super().start()

        self.audio.subscribe(self._on_audio)

        def run() -> None:
            config = uvicorn.Config(
                self._app,
                host="127.0.0.1",
                port=self.config.port,
                log_level="warning",
                lifespan="on",
            )
            self._server = uvicorn.Server(config)
            try:
                # Server.run() owns its own asyncio loop and cleanup; this
                # avoids the run_until_complete/run_coroutine_threadsafe race
                # that produced "Event loop stopped before Future completed".
                self._server.run()
            except OSError as e:
                logger.error(
                    f"audio-ws failed to bind :{self.config.port} ({e}); "
                    f"is another instance running? `lsof -ti :{self.config.port} | xargs kill -9`"
                )
            except Exception:
                logger.exception("audio-ws server crashed")

        self._thread = threading.Thread(target=run, daemon=True, name="audio-ws-uvicorn")
        self._thread.start()
        logger.info(
            f"audio-ws-module: ws://127.0.0.1:{self.config.port}/audio_out  "
            f"| GET http://127.0.0.1:{self.config.port}/audio_info  "
            f"| POST http://127.0.0.1:{self.config.port}/play"
        )

    def _on_audio(self, msg: AudioMessage) -> None:
        self._last_format = {"sample_rate": msg.sample_rate, "channels": msg.channels}
        try:
            self._frame_queue.put_nowait(msg.data)
        except queue.Full:
            # Drop oldest to bound latency.
            try:
                self._frame_queue.get_nowait()
                self._frame_queue.put_nowait(msg.data)
            except queue.Empty:
                pass

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._server is not None:
            self._server.should_exit = True
        super().stop()
