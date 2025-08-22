#!/usr/bin/env python3
"""Test video capture using SDP file."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

import cv2
import numpy as np

def test_sdp_capture():
    print("Testing video capture with SDP file...")
    
    # Path to SDP file
    sdp_file = os.path.join(os.path.dirname(__file__), 'stream.sdp')
    print(f"Using SDP file: {sdp_file}")
    
    # Try OpenCV with the SDP file
    cap = cv2.VideoCapture(sdp_file, cv2.CAP_FFMPEG)
    
    if not cap.isOpened():
        print("Failed to open video capture with SDP")
        return False
    
    print("✓ Video capture opened successfully!")
    
    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"Video properties: {width}x{height} @ {fps} FPS")
    
    frame_count = 0
    start_time = time.time()
    
    print("Capturing frames for 10 seconds...")
    while time.time() - start_time < 10:
        ret, frame = cap.read()
        
        if ret and frame is not None:
            frame_count += 1
            
            if frame_count == 1:
                print(f"First frame received! Shape: {frame.shape}, dtype: {frame.dtype}")
                cv2.imwrite('/tmp/drone_sdp_frame.jpg', frame)
                print("Saved first frame to /tmp/drone_sdp_frame.jpg")
                
                # Check frame statistics
                print(f"  Min pixel: {frame.min()}, Max pixel: {frame.max()}")
                print(f"  Mean: {frame.mean():.1f}")
            
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                actual_fps = frame_count / elapsed
                print(f"Frames: {frame_count}, FPS: {actual_fps:.1f}")
        else:
            # Small delay if no frame
            time.sleep(0.001)
    
    cap.release()
    
    elapsed = time.time() - start_time
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    
    print(f"\n✓ Success! Captured {frame_count} frames in {elapsed:.1f}s")
    print(f"  Average FPS: {avg_fps:.1f}")
    
    return True

if __name__ == "__main__":
    success = test_sdp_capture()
    if not success:
        print("\nTrying alternative approach with ffplay...")
        # Show the ffplay command that would work
        sdp_file = os.path.join(os.path.dirname(__file__), 'stream.sdp')
        print(f"Run this command to test video:")
        print(f"  ffplay -protocol_whitelist file,udp,rtp -i {sdp_file}")