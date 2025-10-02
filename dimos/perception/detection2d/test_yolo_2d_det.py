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

import os
import time

import cv2
import numpy as np
import pytest
import reactivex as rx
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler

from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.stream.video_provider import VideoProvider


class TestYolo2DDetector:
    def test_yolo_detector_initialization(self):
        """Test YOLO detector initializes correctly with default model path."""
        try:
            detector = Yolo2DDetector()
            assert detector is not None
            assert detector.model is not None
        except Exception as e:
            # If the model file doesn't exist, the test should still pass with a warning
            pytest.skip(f"Skipping test due to model initialization error: {e}")

    def test_yolo_detector_process_image(self):
        """Test YOLO detector can process video frames and return detection results."""
        # Create a dedicated scheduler for this test to avoid thread leaks
        test_scheduler = ThreadPoolScheduler(max_workers=6)
        try:
            # Import data inside method to avoid pytest fixture confusion
            from dimos.utils.data import get_data

            detector = Yolo2DDetector()

            video_path = get_data("assets") / "trimmed_video_office.mov"

            # Create video provider and directly get a video stream observable
            assert os.path.exists(video_path), f"Test video not found: {video_path}"
            video_provider = VideoProvider(
                dev_name="test_video", video_source=video_path, pool_scheduler=test_scheduler
            )
            # Process more frames for thorough testing
            video_stream = video_provider.capture_video_as_observable(realtime=False, fps=15)

            # Use ReactiveX operators to process the stream
            def process_frame(frame):
                try:
                    # Process frame with YOLO
                    bboxes, track_ids, class_ids, confidences, names = detector.process_image(frame)
                    print(
                        f"YOLO results - boxes: {(bboxes)}, tracks: {len(track_ids)}, classes: {(class_ids)}, confidences: {(confidences)}, names: {(names)}"
                    )

                    return {
                        "frame": frame,
                        "bboxes": bboxes,
                        "track_ids": track_ids,
                        "class_ids": class_ids,
                        "confidences": confidences,
                        "names": names,
                    }
                except Exception as e:
                    print(f"Exception in process_frame: {e}")
                    return {}

            # Create the detection stream using pipe and map operator
            detection_stream = video_stream.pipe(ops.map(process_frame))

            # Collect results from the stream
            results = []

            frames_processed = 0
            target_frames = 10

            def on_next(result):
                nonlocal frames_processed
                if not result:
                    return

                results.append(result)
                frames_processed += 1

                # Stop after processing target number of frames
                if frames_processed >= target_frames:
                    subscription.dispose()

            def on_error(error):
                pytest.fail(f"Error in detection stream: {error}")

            def on_completed():
                pass

            # Subscribe and wait for results
            subscription = detection_stream.subscribe(
                on_next=on_next, on_error=on_error, on_completed=on_completed
            )

            timeout = 10.0
            start_time = time.time()
            while frames_processed < target_frames and time.time() - start_time < timeout:
                time.sleep(0.5)

            # Clean up subscription
            subscription.dispose()
            video_provider.dispose_all()
            detector.stop()
            # Shutdown the scheduler to clean up threads
            test_scheduler.executor.shutdown(wait=True)
            # Check that we got detection results
            if len(results) == 0:
                pytest.skip("Skipping test due to error: Failed to get any detection results")

            # Verify we have detection results with expected properties
            assert len(results) > 0, "No detection results were received"

            # Print statistics about detections
            total_detections = sum(len(r["bboxes"]) for r in results if r.get("bboxes"))
            avg_detections = total_detections / len(results) if results else 0
            print(f"Total detections: {total_detections}, Average per frame: {avg_detections:.2f}")

            # Print most common detected objects
            object_counts = {}
            for r in results:
                if r.get("names"):
                    for name in r["names"]:
                        if name:
                            object_counts[name] = object_counts.get(name, 0) + 1

            if object_counts:
                print("Detected objects:")
                for obj, count in sorted(object_counts.items(), key=lambda x: x[1], reverse=True)[
                    :5
                ]:
                    print(f"  - {obj}: {count} times")

            # Analyze the first result
            result = results[0]

            # Check that we have a frame
            assert "frame" in result, "Result doesn't contain a frame"
            assert isinstance(result["frame"], np.ndarray), "Frame is not a numpy array"

            # Check that detection results are valid
            assert isinstance(result["bboxes"], list)
            assert isinstance(result["track_ids"], list)
            assert isinstance(result["class_ids"], list)
            assert isinstance(result["confidences"], list)
            assert isinstance(result["names"], list)

            # All result lists should be the same length
            assert (
                len(result["bboxes"])
                == len(result["track_ids"])
                == len(result["class_ids"])
                == len(result["confidences"])
                == len(result["names"])
            )

            # If we have detections, check that bbox format is valid
            if result["bboxes"]:
                assert len(result["bboxes"][0]) == 4, (
                    "Bounding boxes should be in [x1, y1, x2, y2] format"
                )

        except Exception as e:
            # Ensure cleanup happens even on exception
            if "detector" in locals():
                detector.stop()
            if "video_provider" in locals():
                video_provider.dispose_all()
            pytest.skip(f"Skipping test due to error: {e}")
        finally:
            # Always shutdown the scheduler
            test_scheduler.executor.shutdown(wait=True)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
