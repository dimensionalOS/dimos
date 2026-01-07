#!/usr/bin/env python3
"""
Example usage of TemporalMemory module with a VLM.

This example demonstrates how to:
1. Deploy a camera module
2. Deploy TemporalMemory with the camera
3. Query the temporal memory about entities and events
"""

import os
from pathlib import Path

from dimos import core
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.models.vl.openai import OpenAIVlModel
from dimos.perception.temporal_memory import (
    TemporalMemoryConfig,
    _load_openai_api_key,
    deploy,
)

# Load environment variables - deploy() will also load if needed, but load here for early validation

if not _load_openai_api_key():
    print("WARNING: OPENAI_API_KEY not found. Set it in .env or default.env file.")


def example_usage():
    """Example of how to use TemporalMemory."""
    # Create Dimos cluster
    dimos = core.start(1)

    try:
        # Deploy camera module
        camera = dimos.deploy(CameraModule, hardware=lambda: Webcam(camera_index=0))
        camera.start()

        # Deploy temporal memory using the deploy function with GPT-4o mini VLM
        output_dir = Path("./temporal_memory_output")
        # Pass API key explicitly to ensure it's available when pickled to Dask workers
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            print("ERROR: OPENAI_API_KEY not found. Cannot create VLM.")
            return
        print(f"Using OpenAI API key (length: {len(api_key)})")
        vlm = OpenAIVlModel(api_key=api_key)
        temporal_memory = deploy(
            dimos,
            camera,
            vlm=vlm,
            config=TemporalMemoryConfig(
                fps=1.0,  # Process 1 frame per second
                window_s=2.0,  # Analyze 2-second windows
                stride_s=2.0,  # New window every 2 seconds
                summary_interval_s=10.0,  # Update rolling summary every 10 seconds
                max_frames_per_window=3,  # Max 3 frames per window
                output_dir=output_dir,
                use_structured_output=True,  # GPT-4o mini supports structured output
                use_multi_image=True,  # GPT-4o mini supports multi-image queries
            ),
        )

        print("TemporalMemory deployed and started!")
        print(f"Artifacts will be saved to: {output_dir}")

        # Let it run for a bit to build context
        print("Building temporal context... (wait ~15 seconds)")
        import time

        time.sleep(15)

        # Query the temporal memory
        questions = [
            "What entities are currently visible?",
            "What has happened in the last few seconds?",
            "Are there any people in the scene?",
            "Describe the main activity happening now",
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

        # Stop when done
        temporal_memory.stop()
        camera.stop()
        print("\nTemporalMemory stopped")
        
    finally:
        if 'temporal_memory' in locals():
            temporal_memory.stop()
        if 'camera_module' in locals():
            camera_module.stop()
        if 'dimos' in locals():
            dimos.close_all()


if __name__ == "__main__":
    example_usage()
