#!/usr/bin/env python3
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
import base64

# For audio processing
import io
from pathlib import Path
from queue import Empty, Queue
import subprocess
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
from starlette.concurrency import run_in_threadpool
import uvicorn

from dimos.core.global_config import global_config
from dimos.stream.audio.base import AudioEvent
from dimos.web.edge_io import EdgeIO

# TODO: Resolve threading, start/stop stream functionality.


class FastAPIServer(EdgeIO):
    def __init__(  # type: ignore[no-untyped-def]
        self,
        dev_name: str = "FastAPI Server",
        edge_type: str = "Bidirectional",
        host: str | None = None,
        port: int = 5555,
        text_streams=None,
        audio_subject=None,
        **streams,
    ) -> None:
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
        self.host = host if host is not None else global_config.listen_host
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
        self._seat_guide_model = None
        self._seat_guide_yolo_detector = None
        self._seat_guide_model_lock = Lock()

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

        self.setup_routes()

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
    def _decode_audio(raw: bytes) -> tuple[np.ndarray, int]:
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
            # Starlette 0.38+: request is 1st positional arg, not in context dict.
            return self.templates.TemplateResponse(
                request,
                "index_fastapi.html",
                {
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

        @self.app.get("/seat-guide-camera", response_class=HTMLResponse)
        async def seat_guide_camera():  # type: ignore[no-untyped-def]
            """Browser-camera SeatGuide validation page."""
            return HTMLResponse(self._seat_guide_camera_page())

        @self.app.get("/seat-guide-speaker", response_class=HTMLResponse)
        async def seat_guide_speaker():  # type: ignore[no-untyped-def]
            """Phone speaker page for SeatGuide arrival notifications."""
            return HTMLResponse(self._seat_guide_speaker_page())

        @self.app.post("/seat_guide/detect_frame")
        async def seat_guide_detect_frame(request: Request):  # type: ignore[no-untyped-def]
            """Detect chairs, people, and empty seats from a browser camera frame."""
            try:
                payload = await request.json()
                image_data = str(payload.get("image", ""))
                if "," in image_data:
                    image_data = image_data.split(",", 1)[1]
                if not image_data:
                    return JSONResponse(
                        status_code=400,
                        content={"success": False, "message": "Missing image data"},
                    )
                encoded = base64.b64decode(image_data)
                frame = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    return JSONResponse(
                        status_code=400,
                        content={"success": False, "message": "Unable to decode image"},
                    )
                detector = str(payload.get("detector", "yolo")).strip().lower()
                result = await run_in_threadpool(
                    self._detect_seat_guide_frame,
                    frame,
                    detector,
                )
                return JSONResponse({"success": True, **result})
            except Exception as e:
                print(f"SeatGuide camera detection failed: {e}")
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

    def _detect_seat_guide_frame(
        self, frame: np.ndarray, detector: str = "yolo"
    ) -> dict[str, object]:
        """Run local SeatGuide chair/person detection on one browser camera frame."""
        if detector == "moondream":
            return self._detect_seat_guide_frame_moondream(frame)
        if detector != "yolo":
            raise ValueError(f"Unsupported SeatGuide detector: {detector}")
        return self._detect_seat_guide_frame_yolo(frame)

    def _detect_seat_guide_frame_yolo(self, frame: np.ndarray) -> dict[str, object]:
        """Run fast local YOLO chair/person detection on one browser camera frame."""
        import torch

        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
        from dimos.perception.detection.detectors.yolo import Yolo2DDetector

        height, width = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.from_numpy(
            rgb,
            format=ImageFormat.RGB,
            frame_id="browser_camera",
            ts=time.time(),
        )

        with self._seat_guide_model_lock:
            if self._seat_guide_yolo_detector is None:
                device = "cpu"
                if torch.backends.mps.is_available():
                    device = "mps"
                elif torch.cuda.is_available():
                    device = "cuda"
                self._seat_guide_yolo_detector = Yolo2DDetector(device=device)
            detections = self._seat_guide_yolo_detector.process_image(image)

        chair_detections = [
            detection
            for detection in detections.detections
            if detection.name.strip().lower() == "chair"
        ]
        person_detections = [
            detection
            for detection in detections.detections
            if detection.name.strip().lower() == "person"
        ]
        return self._seat_guide_detection_response(
            frame=frame,
            width=width,
            height=height,
            chair_boxes=[tuple(detection.bbox) for detection in chair_detections],
            person_boxes=[tuple(detection.bbox) for detection in person_detections],
            detector="yolo11n",
            description="YOLO realtime mode detects chairs and people without semantic captioning.",
        )

    def _detect_seat_guide_frame_moondream(self, frame: np.ndarray) -> dict[str, object]:
        """Run local Moondream chair/person detection on one browser camera frame."""
        import torch

        from dimos.models.vl.moondream import MoondreamVlModel
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        height, width = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.from_numpy(
            rgb,
            format=ImageFormat.RGB,
            frame_id="browser_camera",
            ts=time.time(),
        )

        with self._seat_guide_model_lock:
            if self._seat_guide_model is None:
                device = "cpu"
                if torch.backends.mps.is_available():
                    device = "mps"
                elif torch.cuda.is_available():
                    device = "cuda"
                self._seat_guide_model = MoondreamVlModel(
                    compile_model=False,
                    device=device,
                )
            model = self._seat_guide_model
            object_description = model.query(
                image,
                "In one short sentence, name the main visible objects in this image.",
            ).strip()
            chair_detections = model.query_detections(image, "chair", max_objects=10).detections
            person_detections = model.query_detections(image, "person", max_objects=10).detections
        return self._seat_guide_detection_response(
            frame=frame,
            width=width,
            height=height,
            chair_boxes=[tuple(detection.bbox) for detection in chair_detections],
            person_boxes=[tuple(detection.bbox) for detection in person_detections],
            detector="moondream2",
            description=object_description,
        )

    def _seat_guide_detection_response(
        self,
        *,
        frame: np.ndarray,
        width: int,
        height: int,
        chair_boxes: list[tuple[float, float, float, float]],
        person_boxes: list[tuple[float, float, float, float]],
        detector: str,
        description: str,
    ) -> dict[str, object]:
        people = person_boxes
        person_centers = [self._bbox_center(bbox) for bbox in people]

        seats: list[dict[str, object]] = []
        annotated = frame.copy()
        for index, bbox in enumerate(chair_boxes, start=1):
            occupied = any(
                self._bbox_contains_point(
                    self._expanded_bbox(bbox, width, height, fraction=0.15),
                    center,
                )
                for center in person_centers
            )
            color = (0, 0, 220) if occupied else (0, 180, 0)
            label = f"occupied chair {index}" if occupied else f"EMPTY SEAT {index}"
            self._draw_detection_box(annotated, bbox, label, color)
            seats.append(
                {
                    "id": f"seat_{index}",
                    "status": "occupied" if occupied else "empty",
                    "bbox": [round(value, 1) for value in bbox],
                }
            )

        for index, bbox in enumerate(people, start=1):
            self._draw_detection_box(annotated, bbox, f"person {index}", (220, 120, 0))

        empty_count = sum(1 for seat in seats if seat["status"] == "empty")
        ok, png = cv2.imencode(".png", annotated)
        if not ok:
            raise RuntimeError("Unable to encode annotated image")

        return {
            "detector": detector,
            "description": description,
            "chairs": len(seats),
            "people": len(people),
            "empty": empty_count,
            "seats": seats,
            "annotated_image": "data:image/png;base64,"
            + base64.b64encode(png.tobytes()).decode("ascii"),
        }

    @staticmethod
    def _bbox_center(bbox: tuple[float, float, float, float]) -> tuple[float, float]:
        x1, y1, x2, y2 = bbox
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0

    @staticmethod
    def _expanded_bbox(
        bbox: tuple[float, float, float, float],
        width: int,
        height: int,
        *,
        fraction: float,
    ) -> tuple[float, float, float, float]:
        x1, y1, x2, y2 = bbox
        pad_x = max(0.0, (x2 - x1) * fraction)
        pad_y = max(0.0, (y2 - y1) * fraction)
        return (
            max(0.0, x1 - pad_x),
            max(0.0, y1 - pad_y),
            min(float(width), x2 + pad_x),
            min(float(height), y2 + pad_y),
        )

    @staticmethod
    def _bbox_contains_point(
        bbox: tuple[float, float, float, float], point: tuple[float, float]
    ) -> bool:
        x1, y1, x2, y2 = bbox
        x, y = point
        return x1 <= x <= x2 and y1 <= y <= y2

    @staticmethod
    def _draw_detection_box(
        image: np.ndarray,
        bbox: tuple[float, float, float, float],
        label: str,
        color: tuple[int, int, int],
    ) -> None:
        x1, y1, x2, y2 = [round(value) for value in bbox]
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
        (text_w, text_h), baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            2,
        )
        top = max(0, y1 - text_h - baseline - 8)
        cv2.rectangle(image, (x1, top), (x1 + text_w + 8, top + text_h + baseline + 8), color, -1)
        cv2.putText(
            image,
            label,
            (x1 + 4, top + text_h + 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def _seat_guide_camera_page() -> str:
        return """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SeatGuide Camera Validation</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      color: #f5f7fb;
      background: #101316;
      font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    header {
      height: 56px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 20px;
      border-bottom: 1px solid #2a3036;
      background: #15191d;
    }
    h1 { margin: 0; font-size: 18px; font-weight: 650; letter-spacing: 0; }
    main {
      display: grid;
      grid-template-columns: minmax(360px, 1fr) minmax(360px, 1fr);
      gap: 16px;
      padding: 16px;
    }
    section {
      min-width: 0;
      border: 1px solid #2a3036;
      border-radius: 8px;
      background: #161b20;
      overflow: hidden;
    }
    .section-head {
      height: 44px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 14px;
      border-bottom: 1px solid #2a3036;
      color: #cfd6df;
      font-size: 14px;
      font-weight: 600;
    }
    video, img {
      display: block;
      width: 100%;
      aspect-ratio: 16 / 9;
      object-fit: contain;
      background: #050607;
    }
    .controls {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      align-items: center;
      padding: 12px;
      border-top: 1px solid #2a3036;
    }
    select {
      height: 36px;
      border: 1px solid #3b4550;
      border-radius: 6px;
      background: #111519;
      color: #f5f7fb;
      padding: 0 10px;
      font-size: 14px;
    }
    button {
      height: 36px;
      border: 1px solid #3b4550;
      border-radius: 6px;
      background: #e7edf5;
      color: #11161b;
      padding: 0 14px;
      font-size: 14px;
      font-weight: 650;
      cursor: pointer;
    }
    button.secondary {
      background: #1f252b;
      color: #f5f7fb;
    }
    button:disabled {
      opacity: 0.55;
      cursor: wait;
    }
    .status {
      padding: 12px;
      min-height: 140px;
      color: #d7dee7;
      background: #111519;
      border-top: 1px solid #2a3036;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 13px;
      line-height: 1.5;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
    }
    .metric {
      color: #9fb0c1;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 13px;
    }
    @media (max-width: 900px) {
      main { grid-template-columns: 1fr; }
      header { height: auto; gap: 8px; align-items: flex-start; flex-direction: column; padding: 12px 16px; }
    }
  </style>
</head>
<body>
  <header>
    <h1>SeatGuide Camera Validation</h1>
    <div class="metric" id="metric">camera=starting</div>
  </header>
  <main>
    <section>
      <div class="section-head"><span>Browser Camera</span><span id="frameSize">-</span></div>
      <video id="video" autoplay playsinline muted></video>
      <div class="controls">
        <select id="detector">
          <option value="yolo" selected>YOLO realtime</option>
          <option value="moondream">Moondream semantic</option>
        </select>
        <button id="detect">Detect Now</button>
        <button id="live" class="secondary">Start Live</button>
        <button id="stop" class="secondary">Stop Camera</button>
      </div>
      <div class="status" id="status">Requesting camera access...</div>
    </section>
    <section>
      <div class="section-head"><span>Detection Result</span><span id="summary">objects=waiting chairs=0 people=0 empty=0</span></div>
      <img id="result" alt="Annotated detection result">
      <div class="status" id="details">No detection run yet.</div>
    </section>
  </main>
  <canvas id="canvas" hidden></canvas>
  <script>
    const video = document.getElementById('video');
    const canvas = document.getElementById('canvas');
    const detect = document.getElementById('detect');
    const live = document.getElementById('live');
    const stopButton = document.getElementById('stop');
    const statusEl = document.getElementById('status');
    const details = document.getElementById('details');
    const metric = document.getElementById('metric');
    const summary = document.getElementById('summary');
    const frameSize = document.getElementById('frameSize');
    const result = document.getElementById('result');
    const detector = document.getElementById('detector');
    let stream = null;
    let liveEnabled = false;
    let detectionInFlight = false;
    let lastDetectionStartedAt = 0;

    function liveIntervalMs() {
      return detector.value === 'yolo' ? 450 : 3500;
    }

    function detectorLabel() {
      return detector.value === 'yolo' ? 'yolo11n' : 'moondream2';
    }

    async function startCamera() {
      stream = await navigator.mediaDevices.getUserMedia({
        video: { width: { ideal: 1280 }, height: { ideal: 720 } },
        audio: false
      });
      video.srcObject = stream;
      await new Promise(resolve => {
        if (video.readyState >= 2 && video.videoWidth > 0) resolve();
        else video.onloadedmetadata = resolve;
      });
      await video.play();
      frameSize.textContent = `${video.videoWidth}x${video.videoHeight}`;
      metric.textContent = `camera=ready model=${detectorLabel()} backend=/seat_guide/detect_frame`;
      statusEl.textContent = 'Camera ready. Point it at chairs, then click Detect Now or Start Live.';
    }

    async function runDetection(source) {
      if (detectionInFlight) return;
      detectionInFlight = true;
      detect.disabled = true;
      try {
        if (!video.videoWidth || !video.videoHeight) {
          throw new Error('Camera frame is not ready.');
        }
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
        const context = canvas.getContext('2d');
        context.drawImage(video, 0, 0, canvas.width, canvas.height);
        const image = canvas.toDataURL('image/jpeg', 0.92);
        lastDetectionStartedAt = performance.now();
        const selectedDetector = detector.value;
        statusEl.textContent = `${source}: captured ${canvas.width}x${canvas.height}. Running local ${detectorLabel()} detection...`;
        const response = await fetch('/seat_guide/detect_frame', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ image, detector: selectedDetector })
        });
        const payload = await response.json();
        if (!response.ok || !payload.success) {
          throw new Error(payload.message || 'Detection request failed.');
        }
        const elapsedMs = performance.now() - lastDetectionStartedAt;
        const elapsedSeconds = (elapsedMs / 1000).toFixed(2);
        const fps = elapsedMs > 0 ? (1000 / elapsedMs).toFixed(1) : '0.0';
        result.src = payload.annotated_image;
        summary.textContent = `model=${payload.detector} chairs=${payload.chairs} people=${payload.people} empty=${payload.empty}`;
        const seatDetails = payload.seats.length
          ? payload.seats.map(seat => `${seat.id}: ${seat.status} bbox=${seat.bbox.join(',')}`).join('\\n')
          : 'No chairs detected. Reframe the camera toward chairs and retry.';
        details.textContent = `Detector: ${payload.detector}\\nLatency: ${elapsedSeconds}s (${fps} fps equivalent)\\nObjects: ${payload.description || 'No object description returned.'}\\n\\n${seatDetails}`;
        metric.textContent = `camera=ready model=${payload.detector} latency=${elapsedSeconds}s fps=${fps} live=${liveEnabled ? 'on' : 'off'}`;
        statusEl.textContent = `${source}: detection complete in ${elapsedSeconds}s.`;
      } catch (error) {
        statusEl.textContent = `Detection failed: ${error.message}`;
      } finally {
        detect.disabled = false;
        detectionInFlight = false;
      }
    }

    detect.onclick = () => runDetection('manual');

    live.onclick = () => {
      liveEnabled = !liveEnabled;
      live.textContent = liveEnabled ? 'Stop Live' : 'Start Live';
      live.classList.toggle('secondary', !liveEnabled);
      metric.textContent = `camera=ready model=${detectorLabel()} live=${liveEnabled ? 'on' : 'off'}`;
      if (liveEnabled) runDetection('live');
    };

    function liveLoop() {
      if (liveEnabled && !detectionInFlight) runDetection('live');
      setTimeout(liveLoop, liveIntervalMs());
    }
    liveLoop();

    detector.onchange = () => {
      metric.textContent = `camera=${stream ? 'ready' : 'starting'} model=${detectorLabel()} live=${liveEnabled ? 'on' : 'off'}`;
    };

    stopButton.onclick = () => {
      liveEnabled = false;
      live.textContent = 'Start Live';
      live.classList.add('secondary');
      if (stream) stream.getTracks().forEach(track => track.stop());
      metric.textContent = 'camera=stopped';
      statusEl.textContent = 'Camera stopped.';
    };

    startCamera().catch(error => {
      metric.textContent = 'camera=error';
      statusEl.textContent = `Camera failed: ${error.message}`;
    });
  </script>
</body>
</html>"""

    @staticmethod
    def _seat_guide_speaker_page() -> str:
        return """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SeatGuide Phone Speaker</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      color: #f6f8fb;
      background: #111315;
      font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      width: min(680px, 100%);
      margin: 0 auto;
      padding: 24px 16px;
    }
    header {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
      padding-bottom: 18px;
      border-bottom: 1px solid #2c333a;
    }
    h1 {
      margin: 0;
      font-size: 24px;
      line-height: 1.15;
      font-weight: 700;
      letter-spacing: 0;
    }
    .state {
      min-width: 118px;
      padding: 8px 10px;
      border: 1px solid #3b454f;
      border-radius: 8px;
      color: #cbd5df;
      background: #171c21;
      text-align: center;
      font-size: 13px;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    }
    .actions {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin: 20px 0;
    }
    button {
      min-height: 48px;
      border: 1px solid #46525e;
      border-radius: 8px;
      background: #e8eef5;
      color: #11161b;
      padding: 0 14px;
      font-size: 16px;
      font-weight: 700;
      cursor: pointer;
    }
    button.secondary {
      background: #20262c;
      color: #f6f8fb;
    }
    .panel {
      border: 1px solid #2c333a;
      border-radius: 8px;
      background: #171c21;
      overflow: hidden;
    }
    .panel-head {
      padding: 12px 14px;
      border-bottom: 1px solid #2c333a;
      color: #cbd5df;
      font-size: 14px;
      font-weight: 650;
    }
    .messages {
      min-height: 260px;
      max-height: 58vh;
      overflow-y: auto;
      padding: 12px;
    }
    .message {
      padding: 12px;
      margin-bottom: 10px;
      border: 1px solid #333c45;
      border-radius: 8px;
      background: #11161a;
      color: #eef3f8;
      line-height: 1.45;
      overflow-wrap: anywhere;
    }
    .time {
      display: block;
      margin-bottom: 5px;
      color: #8e9baa;
      font-size: 12px;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    }
    @media (max-width: 520px) {
      header { flex-direction: column; }
      .state { width: 100%; text-align: left; }
      .actions { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>SeatGuide Phone Speaker</h1>
      </div>
      <div class="state" id="state">audio=locked</div>
    </header>
    <div class="actions">
      <button id="enable">Enable speaker</button>
      <button id="test" class="secondary">Play test</button>
    </div>
    <section class="panel">
      <div class="panel-head">Agent responses</div>
      <div class="messages" id="messages"></div>
    </section>
  </main>
  <script>
    const state = document.getElementById('state');
    const messages = document.getElementById('messages');
    const enable = document.getElementById('enable');
    const test = document.getElementById('test');
    let audioContext = null;
    let enabled = false;

    function isChinese(text) {
      return /[\\u3400-\\u9fff]/.test(text);
    }

    function setState(text) {
      state.textContent = text;
    }

    function appendMessage(text) {
      const item = document.createElement('div');
      item.className = 'message';
      const stamp = document.createElement('span');
      stamp.className = 'time';
      stamp.textContent = new Date().toLocaleTimeString();
      item.appendChild(stamp);
      item.appendChild(document.createTextNode(text));
      messages.prepend(item);
      while (messages.children.length > 20) {
        messages.removeChild(messages.lastChild);
      }
    }

    function beep() {
      if (!audioContext) return;
      const now = audioContext.currentTime;
      const oscillator = audioContext.createOscillator();
      const gain = audioContext.createGain();
      oscillator.frequency.value = 880;
      oscillator.type = 'sine';
      gain.gain.setValueAtTime(0.0001, now);
      gain.gain.exponentialRampToValueAtTime(0.85, now + 0.02);
      gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.8);
      oscillator.connect(gain).connect(audioContext.destination);
      oscillator.start(now);
      oscillator.stop(now + 0.85);
    }

    function speak(text) {
      appendMessage(text);
      if (!enabled) {
        setState('audio=locked');
        return;
      }
      beep();
      if (!('speechSynthesis' in window)) {
        setState('speech=missing');
        return;
      }
      window.speechSynthesis.cancel();
      const utterance = new SpeechSynthesisUtterance(text);
      utterance.lang = isChinese(text) ? 'zh-CN' : 'en-US';
      utterance.volume = 1.0;
      utterance.rate = 0.95;
      utterance.pitch = 1.0;
      utterance.onstart = () => setState('speech=playing');
      utterance.onend = () => setState('speech=ready');
      utterance.onerror = () => setState('speech=error');
      window.speechSynthesis.speak(utterance);
    }

    async function unlockAudio() {
      audioContext = audioContext || new (window.AudioContext || window.webkitAudioContext)();
      if (audioContext.state !== 'running') {
        await audioContext.resume();
      }
      enabled = true;
      setState('speech=ready');
      speak('SeatGuide speaker ready.');
    }

    enable.onclick = () => {
      unlockAudio().catch(error => setState(`audio=error ${error.message}`));
    };
    test.onclick = () => speak('SeatGuide speaker test.');

    const source = new EventSource('/text_stream/agent_responses');
    source.onopen = () => {
      if (!enabled) setState('stream=connected audio=locked');
    };
    source.onerror = () => setState('stream=retrying');
    source.onmessage = event => {
      if (event.data) speak(event.data);
    };
  </script>
</body>
</html>"""

    @staticmethod
    def _ensure_certs(certs_dir: Path) -> tuple[str, str]:
        """Return (cert_path, key_path), generating self-signed certs if needed.
        HTTPS is required by browsers for sensor APIs (DeviceOrientation)"""
        cert_path = certs_dir / "cert.pem"
        key_path = certs_dir / "key.pem"

        if cert_path.exists() and key_path.exists():
            return str(cert_path), str(key_path)

        certs_dir.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            [
                "openssl",
                "req",
                "-x509",
                "-newkey",
                "rsa:2048",
                "-keyout",
                str(key_path),
                "-out",
                str(cert_path),
                "-days",
                "365",
                "-nodes",
                "-subj",
                "/CN=localhost",
            ],
            capture_output=True,
        )
        if result.returncode != 0:
            raise RuntimeError(f"Failed to generate certificates: {result.stderr.decode()}")
        return str(cert_path), str(key_path)

    def run(self, ssl: bool = False, ssl_certs_dir: Path | str | None = None) -> None:
        ssl_certfile = None
        ssl_keyfile = None

        if ssl:
            if ssl_certs_dir is None:
                raise ValueError("ssl_certs_dir is required when ssl=True")
            ssl_certfile, ssl_keyfile = self._ensure_certs(Path(ssl_certs_dir))

        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level="error",  # Reduce verbosity
            ssl_certfile=ssl_certfile,
            ssl_keyfile=ssl_keyfile,
        )
        self._server = uvicorn.Server(config)
        self._server.run()

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.should_exit = True


if __name__ == "__main__":
    server = FastAPIServer()
    server.run()
