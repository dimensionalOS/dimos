#!/usr/bin/env python3
"""
working_realsense_simple.py - Simple working RealSense setup
"""

import pyrealsense2 as rs
import numpy as np
import time

print("Simple RealSense D435 Test\n")

# Reset device first (since that worked)
ctx = rs.context()
devices = ctx.query_devices()
if devices:
    print("Resetting device...")
    devices[0].hardware_reset()
    time.sleep(3)  # Give it time to reset

# Now start fresh
pipeline = rs.pipeline()
config = rs.config()

# Use a config we KNOW works from your output: 640x480 @ 6fps
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 6)

print("Starting pipeline (640x480 @ 6fps)...")
try:
    pipeline.start(config)
    
    # IMPORTANT: Skip warmup frames
    print("Warming up...")
    for i in range(30):
        try:
            pipeline.wait_for_frames(timeout_ms=500)
        except:
            pass  # Warmup frames often fail
    
    print("\nGetting real frames:")
    align = rs.align(rs.stream.color)
    
    for i in range(10):
        frames = pipeline.wait_for_frames(timeout_ms=10000)
        aligned = align.process(frames)
        
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        
        if depth_frame and color_frame:
            depth = np.asanyarray(depth_frame.get_data())
            rgb = np.asanyarray(color_frame.get_data())
            
            valid = np.sum(depth > 0)
            print(f"Frame {i+1}: RGB shape={rgb.shape}, valid depth pixels={valid}")
        
        time.sleep(0.2)  # Don't overwhelm at 6fps
    
    print("\n✓ SUCCESS!")
    
finally:
    pipeline.stop()