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

import cv2
import os
import argparse
from pathlib import Path

def extract_frames(video_path, output_dir, frame_rate):
    """
    Extract frames from a video file at a specified frame rate.

    Parameters:
    - video_path: Path to the input video file (.mov or .mp4).
    - output_dir: Directory where extracted frames will be saved.
    - frame_rate: Frame rate at which to extract frames (frames per second).
    """
    video_path = Path(video_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Open the video file
    cap = cv2.VideoCapture(str(video_path))

    # Get the original frame rate of the video
    original_frame_rate = cap.get(cv2.CAP_PROP_FPS)
    if original_frame_rate == 0:
        print(f"Could not retrieve frame rate for {video_path}")
        return

    # Calculate the interval between frames to capture
    frame_interval = int(round(original_frame_rate / frame_rate))
    if frame_interval == 0:
        frame_interval = 1

    frame_count = 0
    saved_frame_count = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Save the frame at the specified intervals
        if frame_count % frame_interval == 0:
            frame_filename = output_dir / f"frame_{saved_frame_count:04d}.jpg"
            cv2.imwrite(str(frame_filename), frame)
            saved_frame_count += 1

        frame_count += 1

    cap.release()
    print(f"Extracted {saved_frame_count} frames to {output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract frames from a video file.")
    parser.add_argument("video_path", type=str, help="Path to the input .mov or .mp4 video file.")
    parser.add_argument("--output_dir", type=str, default="frames", help="Directory to save extracted frames.")
    parser.add_argument("--frame_rate", type=float, default=1.0, help="Frame rate at which to extract frames (frames per second).")

    args = parser.parse_args()

    extract_frames(args.video_path, args.output_dir, args.frame_rate)
