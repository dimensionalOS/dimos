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

"""Test Zenoh image subscriber that displays camera frames."""

import zenoh
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from dimos.types.image import Image, ImageFormat
import time


class ZenohImageSubscriber:
    def __init__(self, topic="sensors/camera/image"):
        self.topic = topic
        self.latest_frame = None
        self.frame_count = 0
        self.start_time = time.time()

        # Set up matplotlib
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_title(f"Zenoh Camera Stream: {topic}")
        self.ax.axis("off")

        # Initialize with black image
        self.im = self.ax.imshow(np.zeros((480, 640, 3), dtype=np.uint8))

        print(f"Starting Zenoh subscriber for topic: {topic}")

        # Initialize Zenoh
        self.session = zenoh.open(zenoh.Config())
        self.subscriber = self.session.declare_subscriber(topic, self.on_frame)

        print("Zenoh subscriber ready. Waiting for frames...")
        print("Press Ctrl+C to stop")

    def on_frame(self, sample):
        """Callback for received Zenoh frames."""
        try:
            # Deserialize image (handles ZBytes automatically)
            img = Image.from_zenoh_binary(sample.payload)

            # Convert to RGB for matplotlib display
            if img.format == ImageFormat.BGR:
                display_img = img.convert_format(ImageFormat.RGB)
            else:
                display_img = img

            # Store latest frame
            self.latest_frame = display_img.data
            self.frame_count += 1

            # Print stats every 30 frames
            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                print(
                    f"Received {self.frame_count} frames, FPS: {fps:.1f}, "
                    f"Image: {img.width}x{img.height} {img.format}"
                )

        except Exception as e:
            print(f"Error processing frame: {e}")
            import traceback

            traceback.print_exc()

    def update_display(self):
        """Update the matplotlib display with the latest frame."""
        if self.latest_frame is not None:
            self.im.set_array(self.latest_frame)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def run(self):
        """Run the subscriber and display loop."""
        try:
            while True:
                self.update_display()
                plt.pause(0.033)  # ~30 FPS display update

        except KeyboardInterrupt:
            print("\nStopping subscriber...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        try:
            if hasattr(self, "subscriber"):
                self.subscriber.undeclare()
            if hasattr(self, "session"):
                self.session.close()
            plt.close("all")
            print("Cleanup completed.")
        except Exception as e:
            print(f"Error during cleanup: {e}")


def test_simple_display():
    """Simple test that just displays frames as they arrive."""
    print("Simple Zenoh image display test")

    session = zenoh.open(zenoh.Config())

    def on_frame(sample):
        try:
            # Deserialize image (handles ZBytes automatically)
            img = Image.from_zenoh_binary(sample.payload)
            print(f"Received frame: {img.width}x{img.height} {img.format} at {img.timestamp}")

            # Convert to RGB and display
            if img.format == ImageFormat.BGR:
                rgb_img = img.convert_format(ImageFormat.RGB)
            else:
                rgb_img = img

            # Show image
            plt.clf()
            plt.imshow(rgb_img.data)
            plt.title(f"Frame {img.timestamp.strftime('%H:%M:%S.%f')[:-3]}")
            plt.axis("off")
            plt.pause(0.001)

        except Exception as e:
            print(f"Error: {e}")

    # Subscribe
    sub = session.declare_subscriber("robot/camera", on_frame)

    print("Simple subscriber ready. Press Ctrl+C to stop...")

    try:
        plt.ion()
        plt.show()
        while True:
            plt.pause(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        sub.undeclare()
        session.close()
        plt.close("all")


def main():
    """Main function with options."""
    import argparse

    parser = argparse.ArgumentParser(description="Zenoh image subscriber test")
    parser.add_argument(
        "--topic", default="sensors/camera/image", help="Zenoh topic to subscribe to"
    )
    parser.add_argument("--simple", action="store_true", help="Use simple display mode")
    args = parser.parse_args()

    if args.simple:
        test_simple_display()
    else:
        subscriber = ZenohImageSubscriber(topic=args.topic)
        subscriber.run()


if __name__ == "__main__":
    main()
