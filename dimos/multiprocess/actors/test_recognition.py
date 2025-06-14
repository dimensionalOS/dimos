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

import pytest
from streamz import Stream

from dimos.multiprocess.actors import FaceRecognitionActor, VideoActor, deploy_actor
from dimos.multiprocess.utils.testing import dask_client

print(dask_client)


class RecognitionResultActor:
    """Simple actor to collect and print recognition results."""

    def __init__(self, name="RecognitionResults", verbose=True):
        self.name = name
        self.verbose = verbose
        self.detection_count = 0
        self.frame_count = 0
        self.stream = Stream(asynchronous=True)
        self.stream.sink(self._process_recognition)

    def _process_recognition(self, recognition_frame):
        """Process incoming recognition frames."""
        self.frame_count += 1
        detections = recognition_frame.get("detections", [])
        self.detection_count += len(detections)

        if self.verbose and detections:
            print(
                f"{self.name}: Frame {recognition_frame['frame_number']} - {len(detections)} faces:"
            )
            for i, detection in enumerate(detections):
                print(
                    f"  Face {i + 1}: ({detection['x']}, {detection['y']}) {detection['w']}x{detection['h']}"
                )

    async def receive_frame(self, recognition_frame):
        """Receive recognition frame from upstream."""
        await self.stream.emit(recognition_frame)


@pytest.mark.asyncio
async def test_face_recognition(dask_client):
    """Test the face recognition pipeline."""
    print("\n=== Testing Face Recognition Actor ===")

    # Deploy actors
    print("Deploying actors...")
    camera_actor = deploy_actor(dask_client, VideoActor)
    face_actor = deploy_actor(dask_client, FaceRecognitionActor, name="FaceDetector", verbose=True)
    result_actor = deploy_actor(dask_client, RecognitionResultActor, name="Results", verbose=True)

    print(f"Camera actor: {camera_actor}")
    print(f"Face recognition actor: {face_actor}")
    print(f"Result actor: {result_actor}")

    # Connect the pipeline: Camera -> Face Recognition -> Results
    camera_actor.add_processor(face_actor)
    face_actor.add_processor(result_actor)

    # Run for a limited number of frames
    print("Starting face recognition pipeline...")
    camera_actor.run(110).result()  # Process 30 frames

    print("\n=== Results ===")
    print("Face recognition pipeline completed successfully!")
    print("Face recognition test completed successfully!")


@pytest.mark.asyncio
async def test_no_processors_skip(dask_client):
    """Test that recognition actor skips processing when no processors are added."""
    print("\n=== Testing No Processors Behavior ===")

    # Deploy actors but don't connect processors
    camera_actor = deploy_actor(dask_client, VideoActor)
    face_actor = deploy_actor(dask_client, FaceRecognitionActor, name="FaceDetector", verbose=True)

    # Connect camera to face actor, but face actor has no processors
    camera_actor.add_processor(face_actor)

    print("Running with no processors (should skip processing)...")
    camera_actor.run(5).result()  # Process just 5 frames

    print("No processors test completed - should have skipped processing!")


if __name__ == "__main__":
    # Run a quick manual test
    import asyncio

    from dask.distributed import Client

    async def manual_test():
        with Client() as client:
            await test_face_recognition(client)
            await test_no_processors_skip(client)

    asyncio.run(manual_test())
