#!/usr/bin/env python3
"""Test video capture with SDP using different methods."""

import sys
import os
import time
import subprocess
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

import cv2
import numpy as np

def test_opencv_with_options():
    """Try OpenCV with protocol options."""
    print("Testing OpenCV with protocol options...")
    
    sdp_file = os.path.join(os.path.dirname(__file__), 'stream.sdp')
    
    # Try with protocol whitelist as environment variable
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,udp,rtp'
    
    cap = cv2.VideoCapture(sdp_file, cv2.CAP_FFMPEG)
    
    if cap.isOpened():
        print("✓ Opened with protocol whitelist!")
        ret, frame = cap.read()
        if ret:
            print(f"Got frame: {frame.shape}")
            cv2.imwrite('/tmp/drone_frame_opencv.jpg', frame)
        cap.release()
        return True
    
    print("Failed with OpenCV")
    return False

def test_ffmpeg_subprocess():
    """Use ffmpeg subprocess to decode video."""
    print("\nTesting ffmpeg subprocess with SDP...")
    
    sdp_file = os.path.join(os.path.dirname(__file__), 'stream.sdp')
    
    # FFmpeg command with SDP
    cmd = [
        'ffmpeg',
        '-protocol_whitelist', 'file,udp,rtp',
        '-i', sdp_file,
        '-f', 'rawvideo',
        '-pix_fmt', 'bgr24',  # BGR for OpenCV
        '-vcodec', 'rawvideo',
        '-an',  # No audio
        '-'
    ]
    
    print(f"Command: {' '.join(cmd)}")
    
    # Start ffmpeg process
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=10**8
    )
    
    # Read stderr to get video info
    print("Waiting for video info...")
    for _ in range(50):  # Read up to 50 lines
        line = process.stderr.readline()
        if line:
            line_str = line.decode('utf-8', errors='ignore')
            if 'Stream' in line_str and 'Video' in line_str:
                print(f"Video info: {line_str.strip()}")
                # Extract dimensions
                import re
                match = re.search(r'(\d{3,4})x(\d{3,4})', line_str)
                if match:
                    width = int(match.group(1))
                    height = int(match.group(2))
                    print(f"Detected dimensions: {width}x{height}")
                    break
    else:
        # Default dimensions if not detected
        width, height = 1920, 1080
        print(f"Using default dimensions: {width}x{height}")
    
    # Read frames
    frame_size = width * height * 3
    frame_count = 0
    start_time = time.time()
    
    print("Reading frames for 10 seconds...")
    while time.time() - start_time < 10:
        raw_frame = process.stdout.read(frame_size)
        
        if len(raw_frame) == frame_size:
            # Convert to numpy array
            frame = np.frombuffer(raw_frame, dtype=np.uint8)
            frame = frame.reshape((height, width, 3))
            
            frame_count += 1
            
            if frame_count == 1:
                print(f"First frame! Shape: {frame.shape}")
                cv2.imwrite('/tmp/drone_ffmpeg_sdp.jpg', frame)
                print("Saved to /tmp/drone_ffmpeg_sdp.jpg")
            
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"Frames: {frame_count}, FPS: {fps:.1f}")
        
        if process.poll() is not None:
            print("FFmpeg process ended")
            break
    
    process.terminate()
    process.wait()
    
    if frame_count > 0:
        print(f"✓ Success! Captured {frame_count} frames")
        return True
    else:
        print("No frames captured")
        return False

def test_direct_udp():
    """Try direct UDP capture without SDP."""
    print("\nTesting direct UDP capture...")
    
    # Direct RTP over UDP
    url = 'udp://127.0.0.1:5600'
    
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    
    if cap.isOpened():
        print("✓ Opened direct UDP!")
        ret, frame = cap.read()
        if ret:
            print(f"Got frame: {frame.shape}")
            cv2.imwrite('/tmp/drone_direct_udp.jpg', frame)
        cap.release()
        return True
    
    print("Failed with direct UDP")
    return False

if __name__ == "__main__":
    # Test different methods
    success = False
    
    success = test_opencv_with_options() or success
    success = test_ffmpeg_subprocess() or success  
    success = test_direct_udp() or success
    
    if success:
        print("\n✓ At least one method worked!")
    else:
        print("\n✗ All methods failed")
        print("\nYou can manually test with:")
        sdp_file = os.path.join(os.path.dirname(__file__), 'stream.sdp')
        print(f"  ffplay -protocol_whitelist file,udp,rtp -i {sdp_file}")
        print(f"  vlc {sdp_file}")