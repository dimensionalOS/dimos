#!/usr/bin/env python3
"""Test video capture using ffmpeg subprocess."""

import sys
import os
import time
import subprocess
import threading
import queue
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

import cv2
import numpy as np
from dimos.msgs.sensor_msgs import Image

class FFmpegVideoCapture:
    def __init__(self, port=5600):
        self.port = port
        self.process = None
        self.frame_queue = queue.Queue(maxsize=10)
        self.running = False
        
    def start(self):
        """Start ffmpeg capture process."""
        # FFmpeg command to capture UDP stream
        cmd = [
            'ffmpeg',
            '-i', f'udp://0.0.0.0:{self.port}',
            '-f', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-vcodec', 'rawvideo',
            '-'
        ]
        
        print(f"Starting ffmpeg: {' '.join(cmd)}")
        
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=10**8
        )
        
        self.running = True
        
        # Start thread to read stderr (for getting video info)
        self.stderr_thread = threading.Thread(target=self._read_stderr, daemon=True)
        self.stderr_thread.start()
        
        # Start thread to read frames
        self.frame_thread = threading.Thread(target=self._read_frames, daemon=True)
        self.frame_thread.start()
        
        return True
    
    def _read_stderr(self):
        """Read stderr to get video dimensions."""
        while self.running:
            line = self.process.stderr.readline()
            if line:
                line_str = line.decode('utf-8', errors='ignore')
                # Look for video stream info
                if 'Video:' in line_str or 'Stream' in line_str:
                    print(f"FFmpeg info: {line_str.strip()}")
                    # Extract dimensions if available
                    import re
                    match = re.search(r'(\d+)x(\d+)', line_str)
                    if match:
                        self.width = int(match.group(1))
                        self.height = int(match.group(2))
                        print(f"Detected video dimensions: {self.width}x{self.height}")
    
    def _read_frames(self):
        """Read frames from ffmpeg stdout."""
        # Default dimensions (will be updated from stderr)
        self.width = 1920
        self.height = 1080
        
        # Wait a bit for dimensions to be detected
        time.sleep(2)
        
        frame_size = self.width * self.height * 3
        frame_count = 0
        
        while self.running:
            raw_frame = self.process.stdout.read(frame_size)
            
            if len(raw_frame) == frame_size:
                # Convert to numpy array
                frame = np.frombuffer(raw_frame, dtype=np.uint8)
                frame = frame.reshape((self.height, self.width, 3))
                
                # Add to queue (drop old frames if full)
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except:
                        pass
                
                self.frame_queue.put(frame)
                frame_count += 1
                
                if frame_count % 30 == 0:
                    print(f"Captured {frame_count} frames")
    
    def get_frame(self):
        """Get latest frame."""
        try:
            return self.frame_queue.get(timeout=0.1)
        except:
            return None
    
    def stop(self):
        """Stop capture."""
        self.running = False
        if self.process:
            self.process.terminate()
            self.process.wait()

def test_ffmpeg():
    print("Testing FFmpeg video capture...")
    
    capture = FFmpegVideoCapture(5600)
    
    if not capture.start():
        print("Failed to start capture")
        return
    
    print("Waiting for frames...")
    time.sleep(3)
    
    frame_count = 0
    start_time = time.time()
    
    while time.time() - start_time < 10:
        frame = capture.get_frame()
        
        if frame is not None:
            frame_count += 1
            
            if frame_count == 1:
                print(f"First frame! Shape: {frame.shape}")
                # Convert RGB to BGR for OpenCV
                bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite('/tmp/drone_ffmpeg_frame.jpg', bgr_frame)
                print("Saved to /tmp/drone_ffmpeg_frame.jpg")
            
            if frame_count % 10 == 0:
                print(f"Frame {frame_count}")
        
        time.sleep(0.03)  # ~30 FPS
    
    capture.stop()
    print(f"Captured {frame_count} frames")

if __name__ == "__main__":
    test_ffmpeg()