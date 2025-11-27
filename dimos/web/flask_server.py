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

from queue import Queue

import cv2
from flask import Flask, Response, render_template
from reactivex import operators as ops
from reactivex.disposable import SingleAssignmentDisposable

from dimos.web.edge_io import EdgeIO


class FlaskServer(EdgeIO):
    def __init__(  # type: ignore[no-untyped-def]
        self,
        dev_name: str = "Flask Server",
        edge_type: str = "Bidirectional",
        port: int = 5555,
        **streams,
    ) -> None:
        super().__init__(dev_name, edge_type)
        self.app = Flask(__name__)
        self.port = port
        self.streams = streams
        self.active_streams = {}

        # Initialize shared stream references with ref_count
        for key in self.streams:
            if self.streams[key] is not None:
                # Apply share and ref_count to manage subscriptions
                self.active_streams[key] = self.streams[key].pipe(
                    ops.map(self.process_frame_flask), ops.share()
                )

        self.setup_routes()

    def process_frame_flask(self, frame):  # type: ignore[no-untyped-def]
        """Convert frame to JPEG format for streaming."""
        _, buffer = cv2.imencode(".jpg", frame)
        return buffer.tobytes()

    def setup_routes(self) -> None:
        @self.app.route("/")
        def index():  # type: ignore[no-untyped-def]
            stream_keys = list(self.streams.keys())  # Get the keys from the streams dictionary
            return render_template("index_flask.html", stream_keys=stream_keys)

        # Function to create a streaming response
        def stream_generator(key):  # type: ignore[no-untyped-def]
            def generate():  # type: ignore[no-untyped-def]
                frame_queue = Queue()  # type: ignore[var-annotated]
                disposable = SingleAssignmentDisposable()

                # Subscribe to the shared, ref-counted stream
                if key in self.active_streams:
                    disposable.disposable = self.active_streams[key].subscribe(
                        lambda frame: frame_queue.put(frame) if frame is not None else None,
                        lambda e: frame_queue.put(None),
                        lambda: frame_queue.put(None),
                    )

                try:
                    while True:
                        frame = frame_queue.get()
                        if frame is None:
                            break
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                finally:
                    disposable.dispose()

            return generate

        def make_response_generator(key):  # type: ignore[no-untyped-def]
            def response_generator():  # type: ignore[no-untyped-def]
                return Response(
                    stream_generator(key)(),  # type: ignore[no-untyped-call]
                    mimetype="multipart/x-mixed-replace; boundary=frame",
                )

            return response_generator

        # Dynamically adding routes using add_url_rule
        for key in self.streams:
            endpoint = f"video_feed_{key}"
            self.app.add_url_rule(
                f"/video_feed/{key}",
                endpoint,
                view_func=make_response_generator(key),  # type: ignore[no-untyped-call]
            )

    def run(self, host: str = "0.0.0.0", port: int = 5555, threaded: bool = True) -> None:
        self.port = port
        self.app.run(host=host, port=self.port, debug=False, threaded=threaded)
