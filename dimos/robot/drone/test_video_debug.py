#!/usr/bin/env python3
"""Debug video stream to see what's wrong."""

import subprocess
import numpy as np
import time

def test_gstreamer_output():
    """Test raw GStreamer output."""
    cmd = [
        'gst-launch-1.0', '-q',
        'udpsrc', 'port=5600', '!',
        'application/x-rtp,encoding-name=H264,payload=96', '!',
        'rtph264depay', '!',
        'h264parse', '!',
        'avdec_h264', '!',
        'videoconvert', '!',
        'video/x-raw,format=RGB', '!',
        'fdsink'
    ]
    
    print("Starting GStreamer...")
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Read some data
    print("Reading data...")
    data = process.stdout.read(640*360*3*2)  # Read 2 frames worth
    print(f"Read {len(data)} bytes")
    
    # Try to decode as frames
    frame_size = 640 * 360 * 3
    if len(data) >= frame_size:
        frame_data = data[:frame_size]
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        try:
            frame = frame.reshape((360, 640, 3))
            print(f"Frame shape: {frame.shape}")
            print(f"Frame dtype: {frame.dtype}")
            print(f"Frame min/max: {frame.min()}/{frame.max()}")
            print(f"First pixel RGB: {frame[0,0,:]}")
            
            # Save frame for inspection
            import cv2
            cv2.imwrite("/tmp/test_frame_rgb.png", frame)
            cv2.imwrite("/tmp/test_frame_bgr.png", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            print("Saved test frames to /tmp/")
            
        except Exception as e:
            print(f"Failed to reshape: {e}")
    
    process.terminate()

if __name__ == "__main__":
    test_gstreamer_output()