#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.

"""Video streaming using GStreamer Python bindings."""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import threading
import time
from reactivex import Subject

from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)

Gst.init(None)


class DroneVideoStream:
    """Capture drone video using GStreamer Python bindings."""
    
    def __init__(self, port: int = 5600):
        self.port = port
        self._video_subject = Subject()
        self.pipeline = None
        self._running = False
        self.width = 640
        self.height = 360
        
    def start(self) -> bool:
        """Start video capture using GStreamer."""
        try:
            # Build pipeline string
            pipeline_str = f"""
                udpsrc port={self.port} !
                application/x-rtp,encoding-name=H264,payload=96 !
                rtph264depay !
                h264parse !
                avdec_h264 !
                videoconvert !
                video/x-raw,format=RGB,width={self.width},height={self.height} !
                appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true
            """
            
            logger.info(f"Starting GStreamer pipeline on port {self.port}")
            self.pipeline = Gst.parse_launch(pipeline_str)
            
            # Get appsink
            appsink = self.pipeline.get_by_name('sink')
            appsink.connect('new-sample', self._on_new_sample)
            
            # Start pipeline
            self.pipeline.set_state(Gst.State.PLAYING)
            self._running = True
            
            # Start main loop in thread
            self._loop = GLib.MainLoop()
            self._loop_thread = threading.Thread(target=self._loop.run, daemon=True)
            self._loop_thread.start()
            
            logger.info("Video stream started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start video stream: {e}")
            return False
    
    def _on_new_sample(self, sink):
        """Handle new video frame."""
        try:
            sample = sink.emit('pull-sample')
            if not sample:
                return Gst.FlowReturn.OK
            
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            
            # Get frame info from caps
            struct = caps.get_structure(0)
            width = struct.get_value('width')
            height = struct.get_value('height')
            
            # Extract frame data
            result, map_info = buffer.map(Gst.MapFlags.READ)
            if not result:
                return Gst.FlowReturn.OK
            
            # Convert to numpy array
            frame_data = np.frombuffer(map_info.data, dtype=np.uint8)
            
            # Reshape to RGB image
            if len(frame_data) == width * height * 3:
                frame = frame_data.reshape((height, width, 3))
                
                # Create Image message with correct format (RGB from GStreamer)
                img_msg = Image.from_numpy(frame, format=ImageFormat.RGB)
                
                # Publish
                self._video_subject.on_next(img_msg)
            else:
                logger.warning(f"Frame size mismatch: expected {width*height*3}, got {len(frame_data)}")
            
            buffer.unmap(map_info)
            
        except Exception as e:
            logger.error(f"Error processing frame: {e}")
        
        return Gst.FlowReturn.OK
    
    def stop(self):
        """Stop video stream."""
        self._running = False
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            
        if hasattr(self, '_loop'):
            self._loop.quit()
            
        logger.info("Video stream stopped")
    
    def get_stream(self):
        """Get the video stream observable."""
        return self._video_subject