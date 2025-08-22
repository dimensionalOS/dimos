#!/usr/bin/env python3
"""Test real video stream from drone."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

import cv2
import numpy as np

def test_video_stream():
    print("Testing video stream from drone on UDP port 5600...")
    
    # Try different capture methods
    methods = [
        ("GStreamer H.264", (
            'udpsrc port=5600 ! '
            'application/x-rtp,encoding-name=H264,payload=96 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! '
            'videoconvert ! appsink'
        ), cv2.CAP_GSTREAMER),
        ("Simple UDP", 'udp://0.0.0.0:5600', cv2.CAP_ANY),
        ("UDP with format", 'udp://0.0.0.0:5600?overrun_nonfatal=1', cv2.CAP_FFMPEG),
    ]
    
    for name, pipeline, backend in methods:
        print(f"\n=== Trying {name} ===")
        print(f"Pipeline: {pipeline}")
        
        cap = cv2.VideoCapture(pipeline, backend)
        
        if not cap.isOpened():
            print(f"Failed to open with {name}")
            continue
        
        print(f"✓ Opened successfully with {name}")
        
        # Try to read frames
        frame_count = 0
        start_time = time.time()
        
        print("Reading frames for 5 seconds...")
        while time.time() - start_time < 5:
            ret, frame = cap.read()
            
            if ret and frame is not None:
                frame_count += 1
                if frame_count == 1:
                    print(f"First frame received! Shape: {frame.shape}, dtype: {frame.dtype}")
                    
                    # Save first frame for inspection
                    cv2.imwrite('/tmp/drone_frame.jpg', frame)
                    print("Saved first frame to /tmp/drone_frame.jpg")
                
                # Show frame info every 10 frames
                if frame_count % 10 == 0:
                    fps = frame_count / (time.time() - start_time)
                    print(f"Frames: {frame_count}, FPS: {fps:.1f}")
            else:
                time.sleep(0.001)
        
        cap.release()
        
        if frame_count > 0:
            print(f"✓ Success! Received {frame_count} frames")
            return True
        else:
            print(f"No frames received with {name}")
    
    print("\nAll methods failed to receive frames")
    return False

if __name__ == "__main__":
    test_video_stream()