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

from __future__ import annotations

import io
import time
import logging
from pathlib import Path
from typing import Optional

import numpy as np
import ffmpeg
import soundfile as sf
import reactivex as rx
from fastapi import FastAPI, UploadFile, File
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from reactivex.subject import Subject
from reactivex import operators as ops

from dimos.web.edge_io import EdgeIO
from dimos.stream.audio.base import AudioEvent

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class VoiceWebInterface(EdgeIO):
    """A minimal FastAPI server that captures audio from the browser and exposes it
    as a ReactiveX ``Observable`` of :class:`dimos.stream.audio.base.AudioEvent`.

    The browser side records audio using the MediaRecorder API and sends it as a
    single *webm* blob to the ``/upload_audio`` endpoint when the user stops
    recording.  The server converts the blob to mono 16-kHz *wav* using
    *ffmpeg*, loads it into a NumPy array with *soundfile*, wraps it in an
    :class:`AudioEvent`, and finally pushes it to an internal
    ``Subject``.  Down-stream components such as the Whisper STT node can simply
    subscribe to :pyattr:`audio_stream` or call :pyfunc:`emit_audio` to receive
    the audio events.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5560):
        super().__init__(dev_name="Voice Web Interface", edge_type="Input")

        self.host = host
        self.port = port

        # Reactive stream for audio events
        self._audio_subject: Subject = Subject()
        # Shared observable so multiple subscribers receive the same events
        self.audio_stream = self._audio_subject.pipe(ops.share())

        # FastAPI app & CORS for local development
        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        self._setup_routes()

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def emit_audio(self):
        """Return the shared audio observable."""
        return self.audio_stream

    # ------------------------------------------------------------------
    # FastAPI routes
    # ------------------------------------------------------------------

    def _setup_routes(self):
        @self.app.get("/", response_class=HTMLResponse)
        async def index():  # noqa: D401 – simple page
            """Return minimal HTML that records and uploads audio."""
            return HTMLResponse(content=self._index_html(), status_code=200)

        @self.app.post("/upload_audio")
        async def upload_audio(file: UploadFile = File(...)):
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
                self._audio_subject.on_next(event)
                logger.info("Received audio – %.2f s, %d Hz", event.data.shape[0] / sr, sr)
                return {"success": True}
            except Exception as e:  # pragma: no cover – runtime safety
                logger.exception("Failed to process uploaded audio: %s", e)
                return JSONResponse(status_code=500, content={"success": False, "message": str(e)})

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _decode_audio(raw: bytes) -> tuple[Optional[np.ndarray], Optional[int]]:
        """Convert the *webm/opus* blob sent by the browser into mono 16-kHz PCM.

        Returns (audio, sample_rate) or (None, None) on failure.
        """
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
            logger.error("ffmpeg decoding failed: %s", exc)
            return None, None

    @staticmethod
    def _index_html() -> str:
        """Return HTML/JS for the voice interface."""
        return """
<!DOCTYPE html>
<html lang=\"en\">
<head>
    <meta charset=\"UTF-8\" />
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />
    <title>Voice Command Interface</title>
    <style>
        body { font-family: Arial, sans-serif; display:flex; flex-direction:column; align-items:center; padding:40px; }
        button { padding: 15px 25px; font-size: 18px; cursor: pointer; border: none; border-radius: 6px; }
        .rec { background:#dc3545; color:#fff; }
        .idle { background:#28a745; color:#fff; }
        #status { margin-top:20px; font-weight:bold; }
    </style>
</head>
<body>
    <h1>Voice Command Interface</h1>
    <button id=\"recordBtn\" class=\"idle\">🎤 Start Recording</button>
    <div id=\"status\"></div>

<script>
let mediaRecorder;
let chunks = [];
const btn = document.getElementById('recordBtn');
const statusDiv = document.getElementById('status');

btn.addEventListener('click', async () => {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
        mediaRecorder.stop();
        btn.textContent = '🎤 Start Recording';
        btn.className = 'idle';
    } else {
        if (!mediaRecorder) {
            const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
            mediaRecorder = new MediaRecorder(stream);
            mediaRecorder.ondataavailable = e => chunks.push(e.data);
            mediaRecorder.onstop = async () => {
                const blob = new Blob(chunks, { type: 'audio/webm' });
                chunks = [];
                statusDiv.textContent = 'Uploading…';
                const formData = new FormData();
                formData.append('file', blob, 'recording.webm');
                try {
                    const res = await fetch('/upload_audio', { method: 'POST', body: formData });
                    const json = await res.json();
                    statusDiv.textContent = json.success ? 'Uploaded!' : ('Error: ' + json.message);
                } catch(err){
                    statusDiv.textContent = 'Upload failed';
                }
            };
        }
        mediaRecorder.start();
        btn.textContent = '⏹️ Stop Recording';
        btn.className = 'rec';
        statusDiv.textContent = 'Recording…';
    }
});
</script>
</body>
</html>
"""

    # ------------------------------------------------------------------
    # Server runner
    # ------------------------------------------------------------------

    def run(self):
        """Run the FastAPI application using Uvicorn."""
        import uvicorn

        uvicorn.run(self.app, host=self.host, port=self.port)
