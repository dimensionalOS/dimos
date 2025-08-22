#!/usr/bin/env python3
"""Test video capture functionality."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

from dimos.robot.drone.video_capture import VideoCapture

def test_video_capture():
    print("Testing VideoCapture...")
    
    # Create video capture
    capture = VideoCapture(port=5600)
    
    # Track received frames
    frame_count = [0]
    
    def on_frame(img):
        frame_count[0] += 1
        if frame_count[0] == 1:
            print(f"First frame received! Shape: {img.data.shape}")
    
    # Subscribe to stream
    stream = capture.get_stream()
    sub = stream.subscribe(on_frame)
    
    # Start capture
    print("Starting video capture on UDP port 5600...")
    if not capture.start():
        print("Failed to start capture (this is expected without a video stream)")
        return
    
    print("Waiting for frames (5 seconds)...")
    time.sleep(5)
    
    print(f"Received {frame_count[0]} frames")
    
    # Cleanup
    print("Stopping capture...")
    capture.stop()
    sub.dispose()
    
    print("Test completed!")

if __name__ == "__main__":
    test_video_capture()