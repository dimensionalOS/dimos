#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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


# Working FastAPI/Uvicorn Impl.

# Notes: Do not use simultaneously with Flask, this includes imports.
# Workers are not yet setup, as this requires a much more intricate
# reorganization. There appears to be possible signalling issues when
# opening up streams on multiple windows/reloading which will need to
# be fixed. Also note, Chrome only supports 6 simultaneous web streams,
# and its advised to test threading/worker performance with another
# browser like Safari.

# Fast Api & Uvicorn
import asyncio

# For audio processing
import io
from pathlib import Path
from queue import Empty, Queue
from threading import Lock
import time

import cv2
from fastapi import FastAPI, File, Form, HTTPException, Request, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.templating import Jinja2Templates
import ffmpeg  # type: ignore[import-untyped]
import numpy as np
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import SingleAssignmentDisposable
import soundfile as sf  # type: ignore[import-untyped]
from sse_starlette.sse import EventSourceResponse
import uvicorn

from dimos.stream.audio.base import AudioEvent
from dimos.web.edge_io import EdgeIO

# TODO: Resolve threading, start/stop stream functionality.


class FastAPIServer(EdgeIO):
    def __init__(  # type: ignore[no-untyped-def]
        self,
        dev_name: str = "FastAPI Server",
        edge_type: str = "Bidirectional",
        host: str = "0.0.0.0",
        port: int = 5555,
        text_streams=None,
        audio_subject=None,
        **streams,
    ) -> None:
        print("Starting FastAPIServer initialization...")  # Debug print
        super().__init__(dev_name, edge_type)
        self.app = FastAPI()
        self._server: uvicorn.Server | None = None

        # Add CORS middleware with more permissive settings for development
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # More permissive for development
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
            expose_headers=["*"],
        )

        self.port = port
        self.host = host
        BASE_DIR = Path(__file__).resolve().parent
        self.templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))
        self.streams = streams
        self.active_streams = {}
        self.stream_locks = {key: Lock() for key in self.streams}
        self.stream_queues = {}  # type: ignore[var-annotated]
        self.stream_disposables = {}  # type: ignore[var-annotated]

        # Initialize text streams
        self.text_streams = text_streams or {}
        self.text_queues = {}  # type: ignore[var-annotated]
        self.text_disposables = {}
        self.text_clients = set()  # type: ignore[var-annotated]

        # Create a Subject for text queries
        self.query_subject = rx.subject.Subject()  # type: ignore[var-annotated]
        self.query_stream = self.query_subject.pipe(ops.share())
        self.audio_subject = audio_subject

        for key in self.streams:
            if self.streams[key] is not None:
                self.active_streams[key] = self.streams[key].pipe(
                    ops.map(self.process_frame_fastapi), ops.share()
                )

        # Set up text stream subscriptions
        for key, stream in self.text_streams.items():
            if stream is not None:
                self.text_queues[key] = Queue(maxsize=100)
                disposable = stream.subscribe(
                    lambda text, k=key: self.text_queues[k].put(text) if text is not None else None,
                    lambda e, k=key: self.text_queues[k].put(None),
                    lambda k=key: self.text_queues[k].put(None),
                )
                self.text_disposables[key] = disposable
                self.disposables.add(disposable)

        print("Setting up routes...")  # Debug print
        self.setup_routes()
        print("FastAPIServer initialization complete")  # Debug print

    def process_frame_fastapi(self, frame):  # type: ignore[no-untyped-def]
        """Convert frame to JPEG format for streaming."""
        _, buffer = cv2.imencode(".jpg", frame)
        return buffer.tobytes()

    def stream_generator(self, key):  # type: ignore[no-untyped-def]
        """Generate frames for a given video stream."""

        def generate():  # type: ignore[no-untyped-def]
            if key not in self.stream_queues:
                self.stream_queues[key] = Queue(maxsize=10)

            frame_queue = self.stream_queues[key]

            # Clear any existing disposable for this stream
            if key in self.stream_disposables:
                self.stream_disposables[key].dispose()

            disposable = SingleAssignmentDisposable()
            self.stream_disposables[key] = disposable
            self.disposables.add(disposable)

            if key in self.active_streams:
                with self.stream_locks[key]:
                    # Clear the queue before starting new subscription
                    while not frame_queue.empty():
                        try:
                            frame_queue.get_nowait()
                        except Empty:
                            break

                    disposable.disposable = self.active_streams[key].subscribe(
                        lambda frame: frame_queue.put(frame) if frame is not None else None,
                        lambda e: frame_queue.put(None),
                        lambda: frame_queue.put(None),
                    )

            try:
                while True:
                    try:
                        frame = frame_queue.get(timeout=1)
                        if frame is None:
                            break
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                    except Empty:
                        # Instead of breaking, continue waiting for new frames
                        continue
            finally:
                if key in self.stream_disposables:
                    self.stream_disposables[key].dispose()

        return generate

    def create_video_feed_route(self, key):  # type: ignore[no-untyped-def]
        """Create a video feed route for a specific stream."""

        async def video_feed():  # type: ignore[no-untyped-def]
            return StreamingResponse(
                self.stream_generator(key)(),  # type: ignore[no-untyped-call]
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        return video_feed

    async def text_stream_generator(self, key):  # type: ignore[no-untyped-def]
        """Generate SSE events for text stream."""
        client_id = id(object())
        self.text_clients.add(client_id)

        try:
            while True:
                if key not in self.text_queues:
                    yield {"event": "ping", "data": ""}
                    await asyncio.sleep(0.1)
                    continue

                try:
                    text = self.text_queues[key].get_nowait()
                    if text is not None:
                        yield {"event": "message", "id": key, "data": text}
                    else:
                        break
                except Empty:
                    yield {"event": "ping", "data": ""}
                    await asyncio.sleep(0.1)
        finally:
            self.text_clients.remove(client_id)

    @staticmethod
    def _decode_audio(raw: bytes) -> tuple[np.ndarray, int]:  # type: ignore[type-arg]
        """Convert the webm/opus blob sent by the browser into mono 16-kHz PCM."""
        try:
            # Use ffmpeg to convert to 16-kHz mono 16-bit PCM WAV in memory
            out, _ = (
                ffmpeg.input("pipe:0")
                .output(
                    "pipe:1",
                    format="wav",
                    acodec="pcm_s16le",
                    ac=1,
                    ar="16000",
                    loglevel="quiet",
                )
                .run(input=raw, capture_stdout=True, capture_stderr=True)
            )
            # Load with soundfile (returns float32 by default)
            audio, sr = sf.read(io.BytesIO(out), dtype="float32")
            # Ensure 1-D array (mono)
            if audio.ndim > 1:
                audio = audio[:, 0]
            return np.array(audio), sr
        except Exception as exc:
            print(f"ffmpeg decoding failed: {exc}")
            return None, None  # type: ignore[return-value]

    def setup_routes(self) -> None:
        """Set up FastAPI routes."""

        @self.app.get("/streams")
        async def get_streams():  # type: ignore[no-untyped-def]
            """Get list of available video streams"""
            return {"streams": list(self.streams.keys())}

        @self.app.get("/text_streams")
        async def get_text_streams():  # type: ignore[no-untyped-def]
            """Get list of available text streams"""
            return {"streams": list(self.text_streams.keys())}

        @self.app.get("/", response_class=HTMLResponse)
        async def index(request: Request):  # type: ignore[no-untyped-def]
            stream_keys = list(self.streams.keys())
            text_stream_keys = list(self.text_streams.keys())
            return self.templates.TemplateResponse(
                "index_fastapi.html",
                {
                    "request": request,
                    "stream_keys": stream_keys,
                    "text_stream_keys": text_stream_keys,
                    "has_voice": self.audio_subject is not None,
                },
            )

        @self.app.post("/submit_query")
        async def submit_query(query: str = Form(...)):  # type: ignore[no-untyped-def]
            # Using Form directly as a dependency ensures proper form handling
            try:
                if query:
                    # Emit the query through our Subject
                    self.query_subject.on_next(query)
                    return JSONResponse({"success": True, "message": "Query received"})
                return JSONResponse({"success": False, "message": "No query provided"})
            except Exception as e:
                # Ensure we always return valid JSON even on error
                return JSONResponse(
                    status_code=500,
                    content={"success": False, "message": f"Server error: {e!s}"},
                )

        @self.app.post("/upload_audio")
        async def upload_audio(file: UploadFile = File(...)):  # type: ignore[no-untyped-def]
            """Handle audio upload from the browser."""
            if self.audio_subject is None:
                return JSONResponse(
                    status_code=400,
                    content={"success": False, "message": "Voice input not configured"},
                )

            try:
                data = await file.read()
                audio_np, sr = self._decode_audio(data)
                if audio_np is None:
                    return JSONResponse(
                        status_code=400,
                        content={"success": False, "message": "Unable to decode audio"},
                    )

                event = AudioEvent(
                    data=audio_np,
                    sample_rate=sr,
                    timestamp=time.time(),
                    channels=1 if audio_np.ndim == 1 else audio_np.shape[1],
                )

                # Push to reactive stream
                self.audio_subject.on_next(event)
                print(f"Received audio - {event.data.shape[0] / sr:.2f} s, {sr} Hz")
                return {"success": True}
            except Exception as e:
                print(f"Failed to process uploaded audio: {e}")
                return JSONResponse(status_code=500, content={"success": False, "message": str(e)})

        # Unitree API endpoints
        @self.app.get("/unitree/status")
        async def unitree_status():  # type: ignore[no-untyped-def]
            """Check the status of the Unitree API server"""
            return JSONResponse({"status": "online", "service": "unitree"})

        @self.app.post("/unitree/command")
        async def unitree_command(request: Request):  # type: ignore[no-untyped-def]
            """Process commands sent from the terminal frontend"""
            try:
                data = await request.json()
                command_text = data.get("command", "")

                # Emit the command through the query_subject
                self.query_subject.on_next(command_text)

                response = {
                    "success": True,
                    "command": command_text,
                    "result": f"Processed command: {command_text}",
                }

                return JSONResponse(response)
            except Exception as e:
                print(f"Error processing command: {e!s}")
                return JSONResponse(
                    status_code=500,
                    content={"success": False, "message": f"Error processing command: {e!s}"},
                )

        @self.app.get("/text_stream/{key}")
        async def text_stream(key: str):  # type: ignore[no-untyped-def]
            if key not in self.text_streams:
                raise HTTPException(status_code=404, detail=f"Text stream '{key}' not found")
            return EventSourceResponse(self.text_stream_generator(key))  # type: ignore[no-untyped-call]

        for key in self.streams:
            self.app.get(f"/video_feed/{key}")(self.create_video_feed_route(key))  # type: ignore[no-untyped-call]

    def run(self) -> None:
        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level="error",  # Reduce verbosity
        )
        self._server = uvicorn.Server(config)
        self._server.run()

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.should_exit = True


if __name__ == "__main__":
    server = FastAPIServer()
    server.run()
