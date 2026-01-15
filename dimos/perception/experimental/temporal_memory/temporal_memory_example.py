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

import os
from pathlib import Path

from dotenv import load_dotenv

from dimos import core
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam

from .temporal_memory import TemporalMemoryConfig
from .temporal_memory_deploy import deploy

# Load environment variables
load_dotenv()


def example_usage() -> None:
    """Example of how to use TemporalMemory."""
    # Initialize variables to None for cleanup
    temporal_memory = None
    camera = None
    dimos = None

    try:
        # Create Dimos cluster
        dimos = core.start(1)
        # Deploy camera module
        camera = dimos.deploy(CameraModule, hardware=lambda: Webcam(camera_index=0))  # type: ignore[attr-defined]
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

        print("TemporalMemory deployed and started!")
        print(f"Artifacts will be saved to: {output_dir}")

        # Let it run for a bit to build context
        print("Building temporal context... (wait ~15 seconds)")
        import time

        time.sleep(20)

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
        temporal_memory.stop()
        camera.stop()
        print("\nTemporalMemory stopped")

    finally:
        if temporal_memory is not None:
            temporal_memory.stop()
        if camera is not None:
            camera.stop()
        if dimos is not None:
            dimos.close_all()  # type: ignore[attr-defined]


if __name__ == "__main__":
    example_usage()
