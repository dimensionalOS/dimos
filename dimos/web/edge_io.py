from flask import Flask, jsonify, request, Response, render_template
from ..types.media_provider import VideoProviderExample
from ..agents.agent import OpenAI_Agent

import cv2
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler, CurrentThreadScheduler
from reactivex.subject import BehaviorSubject
import numpy as np

from queue import Queue

class EdgeIO():
    def __init__(self, dev_name:str="NA", edge_type:str="Base"):
        self.dev_name = dev_name
        self.edge_type = edge_type
        self.disposables = CompositeDisposable()

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this agent."""
        self.disposables.dispose()

# TODO: Frame processing was moved to its own class. Fix this impl.
class FlaskServer(EdgeIO):
    def __init__(self, dev_name="Flask Server", edge_type="Bidirectional", port=5555, 
                 frame_obs=None, frame_edge_obs=None, frame_optical_obs=None):
        super().__init__(dev_name, edge_type)
        self.app = Flask(__name__)
        self.port = port
        self.frame_obs = frame_obs
        self.frame_edge_obs = frame_edge_obs
        self.frame_optical_obs = frame_optical_obs
        self.setup_routes()
        
    # TODO: Move these processing blocks to a processor block
    def process_frame_flask(self, frame):
        """Convert frame to JPEG format for streaming."""
        _, buffer = cv2.imencode('.jpg', frame)
        return buffer.tobytes()

    def setup_routes(self):
        # TODO: Fix
        # @self.app.route('/start', methods=['GET'])
        # def start_processing():
        #     """Endpoint to start video processing."""
        #     self.agent.subscribe_to_image_processing(self.frame_obs)
        #     return jsonify({"status": "Processing started"}), 200

        # TODO: Fix
        # @self.app.route('/stop', methods=['GET'])
        # def stop_processing():
        #     """Endpoint to stop video processing."""
        #     self.agent.dispose_all()
        #     return jsonify({"status": "Processing stopped"}), 200

        @self.app.route('/')
        def index():
            status_text = "The video stream is currently active."
            return render_template('index.html', status_text=status_text)

        @self.app.route('/video_feed')
        def video_feed():
            def generate():
                frame_queue = Queue()

                def on_next(frame):
                    frame_queue.put(frame)
                
                def on_error(e):
                    print(f"Error in streaming: {e}")
                    frame_queue.put(None)  # Use None to signal an error or completion.
                
                def on_completed():
                    print("Stream completed")
                    frame_queue.put(None)  # Signal completion to the generator.

                disposable_flask = self.frame_obs.subscribe(
                    on_next=lambda frame: self.flask_frame_subject.on_next(frame),
                    on_error=lambda e: print(f"Error: {e}"),
                    on_completed=lambda: self.flask_frame_subject.on_next(None),
                    # scheduler=scheduler
                )

                # Subscribe to the BehaviorSubject
                disposable = self.flask_frame_subject.pipe(
                    ops.map(self.process_frame_flask),
                ).subscribe(on_next, on_error, on_completed)
                
                self.disposables.add(disposable_flask)
                self.disposables.add(disposable)

                try:
                    while True:
                        frame = frame_queue.get()  # Wait for the next frame
                        if frame is None:  # Check if there's a signal to stop.
                            break
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                finally:
                    disposable_flask.dispose()
                    disposable.dispose()

            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/video_feed_edge')
        def video_feed_edge():
            def generate():
                frame_queue = Queue()

                def on_next(frame):
                    frame_queue.put(frame)
                
                def on_error(e):
                    print(f"Error in streaming: {e}")
                    frame_queue.put(None)  # Use None to signal an error or completion.
                
                def on_completed():
                    print("Stream completed")
                    frame_queue.put(None)  # Signal completion to the generator.

                

                disposable_flask = self.frame_edge_obs.subscribe(
                    on_next=lambda frame: self.flask_frame_subject.on_next(frame),
                    on_error=lambda e: print(f"Error: {e}"),
                    on_completed=lambda: self.flask_frame_subject.on_next(None),
                    # scheduler=scheduler
                )

                # Subscribe to the BehaviorSubject
                disposable = self.flask_frame_subject.pipe(
                    ops.subscribe_on(CurrentThreadScheduler()),
                    ops.map(self.process_frame_edge_detection),
                    ops.map(self.process_frame_flask),
                ).subscribe(on_next, on_error, on_completed)
                
                self.disposables.add(disposable_flask)
                self.disposables.add(disposable)

                try:
                    while True:
                        frame = frame_queue.get()  # Wait for the next frame
                        if frame is None:  # Check if there's a signal to stop.
                            break
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                finally:
                    disposable_flask.dispose()
                    disposable.dispose()

            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/video_feed_optical')
        def video_feed_optical():
            def generate():
                frame_queue = Queue()

                def on_next(frame):
                    frame_queue.put(frame)
                
                def on_error(e):
                    print(f"Error in streaming: {e}")
                    frame_queue.put(None)  # Use None to signal an error or completion.
                
                def on_completed():
                    print("Stream completed")
                    frame_queue.put(None)  # Signal completion to the generator.

                # Subscribe to the BehaviorSubject
                disposable = self.frame_optical_obs.subscribe(on_next, on_error, on_completed)

                try:
                    while True:
                        frame = frame_queue.get()  # Wait for the next frame
                        if frame is None:  # Check if there's a signal to stop.
                            continue
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                finally:
                    disposable.dispose()

            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def run(self, host='0.0.0.0', port=5555):
        self.port = port
        self.app.run(host=host, port=self.port, debug=True)

