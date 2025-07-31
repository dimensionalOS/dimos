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

"""Test Image generation from Minecraft RGB data."""

import pickle
import numpy as np
import cv2
from dimos.robot.sim.minecraft.connection import Minecraft


def test_image_from_pickle():
    """Test Image generation using pickled observation data."""
    # Load pickled observation
    with open("observation.pkl", "rb") as f:
        obs = pickle.load(f)

    # Create Minecraft instance
    mc = Minecraft()

    # Get video stream
    video_stream = mc.video_stream()

    # Get one image from the stream
    def get_first_image(img):
        get_first_image.result = img
        raise StopIteration()  # Stop after first image

    get_first_image.result = None

    try:
        subscription = video_stream.subscribe(get_first_image)
    except StopIteration:
        pass

    img_msg = get_first_image.result

    if img_msg:
        print("=== Image Generated ===")
        print(f"Frame ID: {img_msg.frame_id}")
        print(f"Format: {img_msg.format}")
        print(f"Shape: {img_msg.data.shape}")
        print(f"Dtype: {img_msg.data.dtype}")
        print(f"Min/Max: {img_msg.data.min()}/{img_msg.data.max()}")

        # Convert RGB to BGR for OpenCV display
        bgr_img = cv2.cvtColor(img_msg.data, cv2.COLOR_RGB2BGR)

        # Display the image
        cv2.imshow("Minecraft Camera View", bgr_img)
        print("\nPress any key to close the image window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Also save a copy
        cv2.imwrite("minecraft_view.png", bgr_img)
        print("Image saved as minecraft_view.png")


if __name__ == "__main__":
    test_image_from_pickle()
