#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.

"""Video streaming for drone using subprocess with gst-launch."""

import subprocess
import threading
import time
import numpy as np
from typing import Optional
from reactivex import Subject

from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class DroneVideoStream:
    """Capture drone video using gst-launch subprocess."""
    
    def __init__(self, port: int = 5600):
        self.port = port
        self._video_subject = Subject()
        self._process = None
        self._running = False
        
    def start(self) -> bool:
        """Start video capture using gst-launch."""
        try:
            # Use BGR format like Unitree (Foxglove expects BGR)
            cmd = [
                'gst-launch-1.0', '-q',  # Quiet mode
                'udpsrc', f'port={self.port}', '!',
                'application/x-rtp,encoding-name=H264,payload=96', '!',
                'rtph264depay', '!',
                'h264parse', '!',
                'avdec_h264', '!',
                'videoconvert', '!',
                'video/x-raw,format=BGR', '!',
                'fdsink'
            ]
            
            logger.info(f"Starting video capture on UDP port {self.port}")
            
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,  # Capture stderr for debugging
                bufsize=0
            )
            
            self._running = True
            
            # Start capture thread
            self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._capture_thread.start()
            
            # Start error monitoring thread
            self._error_thread = threading.Thread(target=self._error_monitor, daemon=True)
            self._error_thread.start()
            
            logger.info("Video stream started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start video stream: {e}")
            return False
    
    def _capture_loop(self):
        """Read raw video frames from gst-launch stdout."""
        buffer = b''
        frame_count = 0
        bytes_read = 0
        frame_size = None
        width, height = None, None
        
        logger.info("Starting video capture loop")
        
        while self._running:
            try:
                # Read available data (non-blocking)
                chunk = self._process.stdout.read(65536)  # Read in chunks
                if not chunk:
                    if frame_count == 0 and bytes_read == 0:
                        logger.debug("Waiting for video data...")
                    time.sleep(0.001)
                    continue
                    
                buffer += chunk
                bytes_read += len(chunk)
                
                # Auto-detect frame size if not known
                if frame_size is None and len(buffer) > 100000:
                    # Common resolutions for drones
                    for w, h in [(640, 360), (640, 480), (1280, 720), (1920, 1080)]:
                        test_size = w * h * 3
                        if len(buffer) >= test_size:
                            # Try this size
                            try:
                                test_data = buffer[:test_size]
                                test_frame = np.frombuffer(test_data, dtype=np.uint8)
                                test_frame = test_frame.reshape((h, w, 3))
                                # If reshape works, use this size
                                width, height = w, h
                                frame_size = test_size
                                logger.info(f"Detected video size: {width}x{height} ({frame_size} bytes per frame)")
                                break
                            except:
                                continue
                
                # Process complete frames from buffer
                while frame_size and len(buffer) >= frame_size:
                    # Extract one frame
                    frame_data = buffer[:frame_size]
                    buffer = buffer[frame_size:]
                    
                    # Convert to numpy array
                    try:
                        # Create proper numpy array
                        frame = np.frombuffer(frame_data, dtype=np.uint8).copy()
                        frame = frame.reshape((height, width, 3))
                        
                        # Ensure contiguous C-order array
                        if not frame.flags['C_CONTIGUOUS']:
                            frame = np.ascontiguousarray(frame)
                        
                        # Create Image message with BGR format (default)
                        img_msg = Image.from_numpy(frame)  # Default is BGR
                        
                        # Publish
                        self._video_subject.on_next(img_msg)
                        
                        frame_count += 1
                        if frame_count == 1:
                            logger.info("First frame published!")
                        elif frame_count % 100 == 0:
                            logger.info(f"Streamed {frame_count} frames")
                    except Exception as e:
                        logger.debug(f"Frame decode error: {e}")
                        # Skip bad frame but keep going
                        pass
                        
            except Exception as e:
                if self._running:
                    logger.error(f"Error in capture loop: {e}")
                time.sleep(0.1)
    
    def _error_monitor(self):
        """Monitor GStreamer stderr for errors."""
        while self._running and self._process:
            try:
                line = self._process.stderr.readline()
                if line:
                    logger.debug(f"GStreamer: {line.decode('utf-8').strip()}")
            except:
                pass
    
    def stop(self):
        """Stop video stream."""
        self._running = False
        
        if self._process:
            self._process.terminate()
            self._process.wait(timeout=2)
            self._process = None
            
        logger.info("Video stream stopped")
    
    def get_stream(self):
        """Get the video stream observable."""
        return self._video_subject