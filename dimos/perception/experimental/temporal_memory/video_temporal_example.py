#!/usr/bin/env python3
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

"""
Example usage of TemporalMemory module with a VLM.

This example demonstrates how to:
1. Deploy a camera module
2. Deploy TemporalMemory with the camera
3. Query the temporal memory about entities and events
"""

from pathlib import Path
import sys
import threading
import time
from typing import Any

import cv2
from dotenv import load_dotenv
from flask import Flask, jsonify, request
import numpy as np
from numpy.typing import NDArray

from dimos import core
from dimos.core import Module, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.perception.experimental.temporal_memory import TemporalMemoryConfig
from dimos.perception.experimental.temporal_memory.temporal_memory_deploy import deploy
from dimos.stream.video_provider import VideoProvider

# Load environment variables
load_dotenv()

# Flask app for query endpoint
app = Flask(__name__)
_temporal_memory_ref = None


@app.route("/api/query", methods=["POST"])
def query_endpoint() -> Any:
    """Query endpoint for the running TemporalMemory."""
    global _temporal_memory_ref
    if _temporal_memory_ref is None:
        return jsonify({"error": "TemporalMemory not initialized"}), 503

    data = request.get_json()
    if not data or "question" not in data:
        return jsonify({"error": "Missing 'question' field"}), 400

    try:
        answer = _temporal_memory_ref.query(data["question"])
        return jsonify({"answer": answer, "question": data["question"]})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


def start_query_server() -> None:
    """Start Flask server in background thread."""
    app.run(host="127.0.0.1", port=8081, debug=False, threaded=True)


# Simple video file module
class VideoFileModule(Module):
    color_image: Out[Image] = None  # type: ignore[assignment]

    def __init__(self, video_path: str):
        super().__init__()
        self.video_provider = VideoProvider(dev_name="mp4", video_source=video_path)

    @rpc
    def start(self) -> None:
        def on_frame(frame: NDArray[Any]) -> None:
            img = Image.from_numpy(frame, format=ImageFormat.BGR)
            self.color_image.publish(img)

        self._disposables.add(
            self.video_provider.capture_video_as_observable(realtime=True).subscribe(on_frame)
        )

    @rpc
    def stop(self) -> None:
        """Stop the video provider."""
        super().stop()


def example_usage() -> None:
    """Example of how to use TemporalMemory with a video file."""
    global _temporal_memory_ref
    # Initialize variables to None for cleanup
    temporal_memory = None
    camera = None
    dimos = None

    try:
        # Create Dimos cluster
        dimos = core.start(1)

        # Get video path from command line or use default
        if len(sys.argv) > 1:
            video_path = sys.argv[1]
        else:
            video_path = "assets/simple_demo.mp4"

        if not Path(video_path).exists():
            print(f"Error: Video file not found: {video_path}")
            sys.exit(1)

        # Deploy video file module
        camera = dimos.deploy(VideoFileModule, video_path=video_path)  # type: ignore[attr-defined]
        camera.start()

        # Deploy temporal memory using the deploy function
        output_dir = Path("./temporal_memory_output")
        temporal_memory = deploy(
            dimos,
            camera,
            vlm=None,  # Will auto-create OpenAIVlModel if None
            config=TemporalMemoryConfig(
                fps=1.0,  # Process 1 frame per second
                window_s=2.0,  # Analyze 2-second windows
                stride_s=2.0,  # New window every 2 seconds
                summary_interval_s=10.0,  # Update rolling summary every 10 seconds
                max_frames_per_window=3,  # Max 3 frames per window
                output_dir=output_dir,
            ),
        )

        # Store reference for query endpoint
        _temporal_memory_ref = temporal_memory

        # Start query server in background
        server_thread = threading.Thread(target=start_query_server, daemon=True)
        server_thread.start()
        print("✅ Query server started on http://127.0.0.1:8081/api/query")

        print("TemporalMemory deployed and started!")
        print(f"Artifacts will be saved to: {output_dir}")

        # Calculate video duration and wait for full video to process

        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        video_duration = frame_count / fps if fps > 0 else 0
        cap.release()

        if video_duration > 0:
            print(
                f"Video duration: {video_duration:.1f} seconds ({frame_count:.0f} frames @ {fps:.1f} fps)"
            )
            print(f"Processing video... (this will take ~{video_duration:.1f} seconds)")
            # Wait for video duration + a small buffer for processing
            time.sleep(video_duration + 5)
        else:
            print("Could not determine video duration, waiting 30 seconds...")
            time.sleep(30)

        # Query the temporal memory
        questions = [
            "Are there any people in the scene?",
            "Describe the main activity happening now",
            "What has happened in the last few seconds?",
            "What entities are currently visible?",
        ]

        for question in questions:
            print(f"\nQuestion: {question}")
            answer = temporal_memory.query(question)
            print(f"Answer: {answer}")

        # Get current state
        state = temporal_memory.get_state()
        print("\n=== Current State ===")
        print(f"Entity count: {state['entity_count']}")
        print(f"Frame count: {state['frame_count']}")
        print(f"Rolling summary: {state['rolling_summary']}")
        print(f"Entities: {state['entities']}")

        # Get entity roster
        entities = temporal_memory.get_entity_roster()
        print("\n=== Entity Roster ===")
        for entity in entities:
            print(f"  {entity['id']}: {entity['descriptor']}")

        # Check graph database stats
        graph_stats = temporal_memory.get_graph_db_stats()
        print("\n=== Graph Database Stats ===")
        if "error" in graph_stats:
            print(f"Error: {graph_stats['error']}")
        else:
            print(f"Stats: {graph_stats['stats']}")
            print(f"\nEntities in DB ({len(graph_stats['entities'])}):")
            for entity in graph_stats["entities"]:
                print(f"  {entity['entity_id']} ({entity['entity_type']}): {entity['descriptor']}")
            print(f"\nRecent relations ({len(graph_stats['recent_relations'])}):")
            for rel in graph_stats["recent_relations"]:
                print(
                    f"  {rel['subject_id']} --{rel['relation_type']}--> {rel['object_id']} (confidence: {rel['confidence']:.2f})"
                )

        # Stop when done
        print("\nStopping TemporalMemory...")
        temporal_memory.stop()
        camera.stop()
        print("TemporalMemory stopped")

    finally:
        if temporal_memory is not None:
            temporal_memory.stop()
        if camera is not None:
            camera.stop()
        if dimos is not None:
            dimos.close_all()  # type: ignore[attr-defined]


if __name__ == "__main__":
    example_usage()
