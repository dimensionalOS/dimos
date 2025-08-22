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

"""Video capture from UDP stream using GStreamer."""

import threading
import time
from typing import Optional

import cv2
import numpy as np
from reactivex import Subject

from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class VideoCapture:
    """Capture video from UDP stream using GStreamer."""
    
    def __init__(self, port: int = 5600):
        """Initialize video capture.
        
        Args:
            port: UDP port for video stream
        """
        self.port = port
        self._video_subject = Subject()
        self._capture_thread = None
        self._stop_event = threading.Event()
        self._cap = None
    
    def start(self) -> bool:
        """Start video capture."""
        try:
            # GStreamer pipeline for H.264 UDP capture
            pipeline = (
                f'udpsrc port={self.port} ! '
                'application/x-rtp,encoding-name=H264,payload=96 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! '
                'videoconvert ! video/x-raw,format=RGB ! appsink'
            )
            
            logger.info(f"Starting video capture on UDP port {self.port}")
            logger.debug(f"GStreamer pipeline: {pipeline}")
            
            # Try to open with GStreamer backend
            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not self._cap.isOpened():
                # Fallback to testing with a simple UDP capture
                logger.warning("GStreamer capture failed, trying simple UDP")
                self._cap = cv2.VideoCapture(f'udp://0.0.0.0:{self.port}')
                
                if not self._cap.isOpened():
                    logger.error("Failed to open video capture")
                    return False
            
            # Start capture thread
            self._stop_event.clear()
            self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._capture_thread.start()
            
            logger.info("Video capture started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start video capture: {e}")
            return False
    
    def _capture_loop(self):
        """Main capture loop running in thread."""
        logger.info("Video capture loop started")
        frame_count = 0
        last_log_time = time.time()
        
        while not self._stop_event.is_set():
            try:
                ret, frame = self._cap.read()
                
                if ret and frame is not None:
                    # Convert BGR to RGB if needed
                    if len(frame.shape) == 3 and frame.shape[2] == 3:
                        # OpenCV returns BGR, convert to RGB
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    else:
                        frame_rgb = frame
                    
                    # Create Image message
                    img_msg = Image.from_numpy(frame_rgb)
                    
                    # Publish to stream
                    self._video_subject.on_next(img_msg)
                    
                    frame_count += 1
                    
                    # Log stats every 5 seconds
                    current_time = time.time()
                    if current_time - last_log_time > 5.0:
                        fps = frame_count / (current_time - last_log_time)
                        logger.info(f"Video capture: {fps:.1f} FPS, shape={frame.shape}")
                        frame_count = 0
                        last_log_time = current_time
                else:
                    # No frame available, wait a bit
                    time.sleep(0.001)
                    
            except Exception as e:
                logger.error(f"Error in capture loop: {e}")
                time.sleep(0.1)
        
        logger.info("Video capture loop stopped")
    
    def stop(self):
        """Stop video capture."""
        logger.info("Stopping video capture")
        
        # Signal thread to stop
        self._stop_event.set()
        
        # Wait for thread to finish
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        
        # Release capture
        if self._cap:
            self._cap.release()
            self._cap = None
        
        logger.info("Video capture stopped")
    
    def get_stream(self):
        """Get the video stream observable.
        
        Returns:
            Observable stream of Image messages
        """
        return self._video_subject
    
    def is_running(self) -> bool:
        """Check if capture is running.
        
        Returns:
            True if capture thread is active
        """
        return self._capture_thread and self._capture_thread.is_alive()