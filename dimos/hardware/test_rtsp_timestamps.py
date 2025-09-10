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

"""Quick test to verify RTSP absolute timestamps are working correctly."""

import sys
import time
import subprocess

# Add system path for gi module if needed
if "/usr/lib/python3/dist-packages" not in sys.path:
    sys.path.insert(0, "/usr/lib/python3/dist-packages")

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)


def test_rtsp_timestamps(rtsp_url="rtsp://localhost:8554/video", duration=5):
    """Test RTSP stream and verify timestamps."""

    print(f"Testing RTSP stream: {rtsp_url}")
    print(f"Duration: {duration} seconds")
    print("-" * 50)

    # Create pipeline to receive RTSP stream
    pipeline_str = f"""
        rtspsrc location={rtsp_url} protocols=4 latency=0 !
        rtph264depay !
        h264parse !
        fakesink name=sink
    """

    pipeline = Gst.parse_launch(pipeline_str)
    sink = pipeline.get_by_name("sink")

    frame_count = [0]
    first_timestamp = [None]
    timestamps_absolute = [False]

    def on_handoff(sink, buffer, pad):
        """Check buffer timestamps."""
        if buffer.pts != Gst.CLOCK_TIME_NONE:
            timestamp = buffer.pts / 1e9
            frame_count[0] += 1

            if first_timestamp[0] is None:
                first_timestamp[0] = timestamp
                current_time = time.time()

                # Check if timestamp is absolute (Unix epoch)
                if timestamp > 1577836800:  # > Jan 1, 2020
                    timestamps_absolute[0] = True
                    drift = abs(timestamp - current_time)
                    print(f"✓ First frame has ABSOLUTE timestamp: {timestamp:.3f}")
                    print(f"  Current system time: {current_time:.3f}")
                    print(f"  Time drift: {drift:.3f} seconds")
                else:
                    print(f"✗ First frame has RELATIVE timestamp: {timestamp:.3f}")
                    print(f"  This is NOT an absolute Unix timestamp!")

            # Log every 10th frame
            if frame_count[0] % 10 == 0:
                if timestamps_absolute[0]:
                    current_time = time.time()
                    latency = current_time - timestamp
                    print(f"Frame {frame_count[0]}: ts={timestamp:.3f}, latency={latency:.3f}s")
                else:
                    delta = timestamp - first_timestamp[0]
                    print(
                        f"Frame {frame_count[0]}: relative_ts={timestamp:.3f}, delta={delta:.3f}s"
                    )

        return Gst.PadProbeReturn.OK

    # Connect to sink handoff signal
    sink.set_property("signal-handoffs", True)
    sink.connect("handoff", on_handoff)

    # Use system clock
    clock = Gst.SystemClock.obtain()
    pipeline.use_clock(clock)
    pipeline.set_base_time(0)

    # Start pipeline
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("Failed to start pipeline - is the RTSP server running?")
        return False

    # Run for specified duration
    bus = pipeline.get_bus()
    msg = bus.timed_pop_filtered(duration * Gst.SECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS)

    if msg:
        if msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            print(f"Error: {err}")
            return False

    # Clean up
    pipeline.set_state(Gst.State.NULL)

    print("-" * 50)
    print(f"Test complete: Received {frame_count[0]} frames")

    if timestamps_absolute[0]:
        print("✓ SUCCESS: RTSP stream is using ABSOLUTE timestamps!")
    else:
        print("✗ FAILURE: RTSP stream is using RELATIVE timestamps")
        print("  The timestamps are relative to stream start, not Unix epoch")

    return timestamps_absolute[0]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test RTSP timestamp handling")
    parser.add_argument("--url", default="rtsp://localhost:8554/video", help="RTSP URL to test")
    parser.add_argument("--duration", type=int, default=5, help="Test duration in seconds")

    args = parser.parse_args()

    print("=" * 50)
    print("RTSP Absolute Timestamp Test")
    print("=" * 50)
    print()
    print("Make sure the RTSP server is running:")
    print("  python3 rtsp_sender.py --device /dev/video0")
    print()

    success = test_rtsp_timestamps(args.url, args.duration)
    sys.exit(0 if success else 1)
