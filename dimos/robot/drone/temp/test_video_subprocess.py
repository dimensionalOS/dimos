#!/usr/bin/env python3
"""Test video capture using subprocess with GStreamer."""

import sys
import os
import time
import subprocess
import threading
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

import cv2
import numpy as np

def test_subprocess_capture():
    print("Testing video capture with subprocess...")
    
    # GStreamer command that outputs raw video to stdout
    cmd = [
        'gst-launch-1.0',
        'udpsrc', 'port=5600', '!',
        'application/x-rtp,encoding-name=H264,payload=96', '!',
        'rtph264depay', '!', 
        'h264parse', '!',
        'avdec_h264', '!',
        'videoconvert', '!',
        'video/x-raw,format=BGR', '!',
        'filesink', 'location=/dev/stdout'
    ]
    
    print(f"Command: {' '.join(cmd)}")
    
    # Start the GStreamer process
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=0
    )
    
    print("GStreamer process started")
    
    # We need to figure out frame dimensions first
    # For DJI drones, common resolutions are 1920x1080 or 1280x720
    width = 1280
    height = 720
    channels = 3
    frame_size = width * height * channels
    
    frame_count = 0
    start_time = time.time()
    
    print(f"Reading frames (assuming {width}x{height})...")
    
    try:
        while time.time() - start_time < 10:
            # Read raw frame data
            raw_data = process.stdout.read(frame_size)
            
            if len(raw_data) == frame_size:
                # Convert to numpy array
                frame = np.frombuffer(raw_data, dtype=np.uint8)
                frame = frame.reshape((height, width, channels))
                
                frame_count += 1
                
                if frame_count == 1:
                    print(f"First frame received! Shape: {frame.shape}")
                    cv2.imwrite('/tmp/drone_subprocess_frame.jpg', frame)
                    print("Saved to /tmp/drone_subprocess_frame.jpg")
                
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed
                    print(f"Frames: {frame_count}, FPS: {fps:.1f}")
            
            if process.poll() is not None:
                print("GStreamer process ended")
                break
                
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        process.terminate()
        process.wait()
    
    elapsed = time.time() - start_time
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"\nCaptured {frame_count} frames in {elapsed:.1f}s (avg {avg_fps:.1f} FPS)")

def test_simple_capture():
    """Try a simpler OpenCV capture with different parameters."""
    print("\n=== Testing simple OpenCV capture ===")
    
    # Try with explicit parameters
    pipeline = (
        'udpsrc port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! '
        'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink'
    )
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if cap.isOpened():
        print("✓ Capture opened!")
        
        for i in range(10):
            ret, frame = cap.read()
            if ret:
                print(f"Frame {i}: {frame.shape}")
                if i == 0:
                    cv2.imwrite('/tmp/drone_simple_frame.jpg', frame)
            else:
                print(f"Failed to read frame {i}")
            time.sleep(0.1)
        
        cap.release()
    else:
        print("Failed to open capture")

if __name__ == "__main__":
    # test_subprocess_capture()
    test_simple_capture()