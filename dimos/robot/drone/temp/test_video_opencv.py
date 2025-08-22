#!/usr/bin/env python3
"""Test video capture with OpenCV using GStreamer."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

import cv2
import numpy as np

def test_gstreamer_capture():
    print("Testing GStreamer video capture...")
    
    # Use the exact pipeline that worked
    pipeline = (
        'udpsrc port=5600 ! '
        'application/x-rtp,encoding-name=H264,payload=96 ! '
        'rtph264depay ! h264parse ! avdec_h264 ! '
        'videoconvert ! video/x-raw,format=BGR ! appsink drop=1'
    )
    
    print(f"Pipeline: {pipeline}")
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("Failed to open video capture")
        return False
    
    print("✓ Video capture opened successfully")
    
    frame_count = 0
    start_time = time.time()
    
    print("Capturing frames for 10 seconds...")
    while time.time() - start_time < 10:
        ret, frame = cap.read()
        
        if ret and frame is not None:
            frame_count += 1
            
            if frame_count == 1:
                print(f"First frame: shape={frame.shape}, dtype={frame.dtype}")
                # Save first frame
                cv2.imwrite('/tmp/drone_first_frame.jpg', frame)
                print("Saved first frame to /tmp/drone_first_frame.jpg")
            
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"Frames: {frame_count}, FPS: {fps:.1f}, Frame shape: {frame.shape}")
        else:
            # Small sleep if no frame
            time.sleep(0.001)
    
    cap.release()
    
    elapsed = time.time() - start_time
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"\n✓ Captured {frame_count} frames in {elapsed:.1f}s (avg {avg_fps:.1f} FPS)")
    
    return True

if __name__ == "__main__":
    test_gstreamer_capture()