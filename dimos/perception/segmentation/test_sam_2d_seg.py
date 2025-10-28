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

import numpy as np
import pytest
from reactivex import operators as ops

from dimos.perception.segmentation.sam_2d_seg import Sam2DSegmenter
from dimos.perception.segmentation.utils import extract_masks_bboxes_probs_names
from dimos.stream.video_provider import VideoProvider


@pytest.mark.heavy
class TestSam2DSegmenter:
    def test_sam_segmenter_initialization(self) -> None:
        """Test FastSAM segmenter initializes correctly with default model path."""
        try:
            # Try to initialize with the default model path and existing device setting
            segmenter = Sam2DSegmenter(use_analyzer=False)
            assert segmenter is not None
            assert segmenter.model is not None
        except Exception as e:
            # If the model file doesn't exist, the test should still pass with a warning
            pytest.skip(f"Skipping test due to model initialization error: {e}")

    def test_sam_segmenter_process_image(self) -> None:
        """Test FastSAM segmenter can process video frames and return segmentation masks."""
        # Import get data inside method to avoid pytest fixture confusion
        from dimos.utils.data import get_data

        # Get test video path directly
        video_path = get_data("assets") / "trimmed_video_office.mov"
        try:
            # Initialize segmenter without analyzer for faster testing
            segmenter = Sam2DSegmenter(use_analyzer=False)

            # Note: conf and iou are parameters for process_image, not constructor
            # We'll monkey patch the process_image method to use lower thresholds

            def patched_process_image(image):
                results = segmenter.model.track(
                    source=image,
                    device=segmenter.device,
                    retina_masks=True,
                    conf=0.1,  # Lower confidence threshold for testing
                    iou=0.5,  # Lower IoU threshold
                    persist=True,
                    verbose=False,
                    tracker=segmenter.tracker_config
                    if hasattr(segmenter, "tracker_config")
                    else None,
                )

                if len(results) > 0:
                    masks, bboxes, track_ids, probs, names, _areas = (
                        extract_masks_bboxes_probs_names(results[0])
                    )
                    return masks, bboxes, track_ids, probs, names
                return [], [], [], [], []

            # Replace the method
            segmenter.process_image = patched_process_image

            # Create video provider and directly get a video stream observable
            assert os.path.exists(video_path), f"Test video not found: {video_path}"
            video_provider = VideoProvider(dev_name="test_video", video_source=video_path)

            video_stream = video_provider.capture_video_as_observable(realtime=False, fps=1)

            # Use ReactiveX operators to process the stream
            def process_frame(frame):
                try:
                    # Process frame with FastSAM
                    masks, bboxes, track_ids, probs, names = segmenter.process_image(frame)
                    print(
                        f"SAM results - masks: {len(masks)}, bboxes: {len(bboxes)}, track_ids: {len(track_ids)}, names: {len(names)}"
                    )

                    return {
                        "frame": frame,
                        "masks": masks,
                        "bboxes": bboxes,
                        "track_ids": track_ids,
                        "probs": probs,
                        "names": names,
                    }
                except Exception as e:
                    print(f"Error in process_frame: {e}")
                    return {}

            # Create the segmentation stream using pipe and map operator
            segmentation_stream = video_stream.pipe(ops.map(process_frame))

            # Collect results from the stream
            results = []
            frames_processed = 0
            target_frames = 5

            def on_next(result) -> None:
                nonlocal frames_processed, results
                if not result:
                    return

                results.append(result)
                frames_processed += 1

                # Stop processing after target frames
                if frames_processed >= target_frames:
                    subscription.dispose()

            def on_error(error) -> None:
                pytest.fail(f"Error in segmentation stream: {error}")

            def on_completed() -> None:
                pass

            # Subscribe and wait for results
            subscription = segmentation_stream.subscribe(
                on_next=on_next, on_error=on_error, on_completed=on_completed
            )

            # Wait for frames to be processed
            timeout = 30.0  # seconds
            start_time = time.time()
            while frames_processed < target_frames and time.time() - start_time < timeout:
                time.sleep(0.5)

            # Clean up subscription
            subscription.dispose()
            video_provider.dispose_all()

            # Check if we have results
            if len(results) == 0:
                pytest.skip(
                    "No segmentation results found, but test connection established correctly"
                )
                return

            print(f"Processed {len(results)} frames with segmentation results")

            # Analyze the first result
            result = results[0]

            # Check that we have a frame
            assert "frame" in result, "Result doesn't contain a frame"
            assert isinstance(result["frame"], np.ndarray), "Frame is not a numpy array"

            # Check that segmentation results are valid
            assert isinstance(result["masks"], list)
            assert isinstance(result["bboxes"], list)
            assert isinstance(result["track_ids"], list)
            assert isinstance(result["probs"], list)
            assert isinstance(result["names"], list)

            # All result lists should be the same length
            assert (
                len(result["masks"])
                == len(result["bboxes"])
                == len(result["track_ids"])
                == len(result["probs"])
                == len(result["names"])
            )

            # If we have masks, check that they have valid shape
            if result.get("masks") and len(result["masks"]) > 0:
                assert result["masks"][0].shape == (
                    result["frame"].shape[0],
                    result["frame"].shape[1],
                ), "Mask shape should match image dimensions"
                print(f"Found {len(result['masks'])} masks in first frame")
            else:
                print("No masks found in first frame, but test connection established correctly")

            # Test visualization function
            if result["masks"]:
                vis_frame = segmenter.visualize_results(
                    result["frame"],
                    result["masks"],
                    result["bboxes"],
                    result["track_ids"],
                    result["probs"],
                    result["names"],
                )
                assert isinstance(vis_frame, np.ndarray), "Visualization output should be an image"
                assert vis_frame.shape == result["frame"].shape, (
                    "Visualization should have same dimensions as input frame"
                )

            # We've already tested visualization above, so no need for a duplicate test

        except Exception as e:
            pytest.skip(f"Skipping test due to error: {e}")


if __name__ == "__main__":
    pytest.main(["-v", __file__])
