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
import cv2
from dimos.web.edge_io import EdgeIO
from fastapi import FastAPI, Request, Response, Form, HTTPException
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from sse_starlette.sse import EventSourceResponse
from fastapi.templating import Jinja2Templates
import uvicorn
from threading import Lock
from pathlib import Path
from queue import Queue, Empty
import asyncio

from reactivex.disposable import SingleAssignmentDisposable
from reactivex import operators as ops
import reactivex as rx
from fastapi.middleware.cors import CORSMiddleware

# TODO: Resolve threading, start/stop stream functionality.


class FastAPIServer(EdgeIO):

    

    def __init__(self,
                 dev_name="FastAPI Server",
                 edge_type="Bidirectional",
                 host="0.0.0.0",
                 port=5555,
                 text_streams=None,
                 **streams):
        print("Starting FastAPIServer initialization...")  # Debug print
        super().__init__(dev_name, edge_type)
        self.app = FastAPI()
        
        # Add CORS middleware with more permissive settings for development
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # More permissive for development
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
            expose_headers=["*"]
        )
        
        self.port = port
        self.host = host
        BASE_DIR = Path(__file__).resolve().parent
        self.templates = Jinja2Templates(directory=str(BASE_DIR / 'templates'))
        self.streams = streams
        self.active_streams = {}
        self.stream_locks = {key: Lock() for key in self.streams}
        self.stream_queues = {}
        self.stream_disposables = {}

        # Initialize text streams
        self.text_streams = text_streams or {}
        self.text_queues = {}
        self.text_disposables = {}
        self.text_clients = set()

        # Create a Subject for text queries
        self.query_subject = rx.subject.Subject()
        self.query_stream = self.query_subject.pipe(ops.share())

        for key in self.streams:
            if self.streams[key] is not None:
                self.active_streams[key] = self.streams[key].pipe(
                    ops.map(self.process_frame_fastapi), ops.share())

        # Set up text stream subscriptions
        for key, stream in self.text_streams.items():
            if stream is not None:
                self.text_queues[key] = Queue(maxsize=100)
                disposable = stream.subscribe(
                    lambda text, k=key: self.text_queues[k].put(text) 
                    if text is not None else None,
                    lambda e, k=key: self.text_queues[k].put(None),
                    lambda k=key: self.text_queues[k].put(None)
                )
                self.text_disposables[key] = disposable
                self.disposables.add(disposable)

        print("Setting up routes...")  # Debug print
        self.setup_routes()
        print("FastAPIServer initialization complete")  # Debug print

    def process_frame_fastapi(self, frame):
        """Convert frame to JPEG format for streaming."""
        _, buffer = cv2.imencode('.jpg', frame)
        return buffer.tobytes()

    def stream_generator(self, key):
        """Generate frames for a given video stream."""

        def generate():
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
                        lambda frame: frame_queue.put(frame)
                        if frame is not None else None,
                        lambda e: frame_queue.put(None),
                        lambda: frame_queue.put(None))

            try:
                while True:
                    try:
                        frame = frame_queue.get(timeout=1)
                        if frame is None:
                            break
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame +
                               b'\r\n')
                    except Empty:
                        # Instead of breaking, continue waiting for new frames
                        continue
            finally:
                if key in self.stream_disposables:
                    self.stream_disposables[key].dispose()

        return generate

    def create_video_feed_route(self, key):
        """Create a video feed route for a specific stream."""

        async def video_feed():
            return StreamingResponse(
                self.stream_generator(key)(),
                media_type="multipart/x-mixed-replace; boundary=frame")

        return video_feed

    async def text_stream_generator(self, key):
        """Generate SSE events for text stream."""
        client_id = id(object())
        self.text_clients.add(client_id)
        
        try:
            while True:
                if key in self.text_queues:
                    try:
                        text = self.text_queues[key].get(timeout=1)
                        if text is not None:
                            event_data = {
                                "event": "message",
                                "id": key,
                                "data": text
                            }
                            yield event_data
                    except Empty:
                        # Send a keep-alive comment
                        yield {
                            "event": "ping",
                            "data": ""
                        }
                await asyncio.sleep(0.1)
        finally:
            self.text_clients.remove(client_id)

    def setup_routes(self):
        """Set up FastAPI routes."""

        @self.app.get("/streams")
        async def get_streams():
            """Get list of available video streams"""
            return {"streams": list(self.streams.keys())}

        @self.app.get("/text_streams")
        async def get_text_streams():
            """Get list of available text streams"""
            return {"streams": list(self.text_streams.keys())}

        @self.app.get("/", response_class=HTMLResponse)
        async def index(request: Request):
            stream_keys = list(self.streams.keys())
            text_stream_keys = list(self.text_streams.keys())
            return self.templates.TemplateResponse("index_fastapi.html", {
                "request": request,
                "stream_keys": stream_keys,
                "text_stream_keys": text_stream_keys
            })
                    
        @self.app.post("/submit_query")
        async def submit_query(query: str = Form(...)):
            # Using Form directly as a dependency ensures proper form handling
            try:
                if query:
                    # Emit the query through our Subject
                    self.query_subject.on_next(query)
                    return JSONResponse({
                        "success": True,
                        "message": "Query received"
                    })
                return JSONResponse({
                    "success": False,
                    "message": "No query provided"
                })
            except Exception as e:
                # Ensure we always return valid JSON even on error
                return JSONResponse(status_code=500,
                                    content={
                                        "success": False,
                                        "message": f"Server error: {str(e)}"
                                    })
        # Unitree API endpoints
        @self.app.get("/unitree/status")
        async def unitree_status():
            """Check the status of the Unitree API server"""
            return JSONResponse({
                "status": "online", 
                "service": "unitree"
            })

        @self.app.post("/unitree/command")
        async def unitree_command(request: Request):
            """Process commands sent from the terminal frontend"""
            try:
                data = await request.json()
                command_text = data.get("command", "")
                
                # Emit the command through the query_subject
                self.query_subject.on_next(command_text)
                
                response = {
                    "success": True,
                    "command": command_text,
                    "result": f"Processed command: {command_text}"
                }
                
                return JSONResponse(response)
            except Exception as e:
                print(f"Error processing command: {str(e)}")  
                return JSONResponse(
                    status_code=500,
                    content={
                        "success": False,
                        "message": f"Error processing command: {str(e)}"
                    }
                )

        @self.app.get("/text_stream/{key}")
        async def text_stream(key: str):
            if key not in self.text_streams:
                raise HTTPException(status_code=404, detail=f"Text stream '{key}' not found")
            return EventSourceResponse(self.text_stream_generator(key))


        for key in self.streams:
            self.app.get(f"/video_feed/{key}")(
                self.create_video_feed_route(key))

    def run(self):
        """Run the FastAPI server."""
        uvicorn.run(self.app, host=self.host, port=self.port
                   )  # TODO: Translate structure to enable in-built workers' 
        
if __name__ == "__main__":
    server = FastAPIServer()
    server.run()