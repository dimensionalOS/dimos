from isaacsim import SimulationApp
# Initialize the Isaac Sim application in headless mode
simulation_app = SimulationApp({"headless": True})

import os
import omni.usd
import omni.replicator.core as rep
import subprocess
import cv2
import numpy as np
from pxr import UsdGeom, Sdf
import time

# Specify the input USDA file
USDA_FILE_PATH = "/dimos/assets/TestSim3.usda"

# FFmpeg configuration
width, height = 1920, 1080
fps = 65

command = [
    'ffmpeg',
    '-y',
    '-loglevel', 'debug',
    '-f', 'rawvideo',
    '-vcodec', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', f"{width}x{height}",
    '-r', str(fps),
    '-i', '-',
    '-an',  # No audio
    '-c:v', 'h264_nvenc',
    '-preset', 'fast',
    '-f', 'rtsp',
    'rtsp://mediamtx:8554/stream',
    '-rtsp_transport', 'tcp'
]

# Open FFmpeg process
proc = subprocess.Popen(command, stdin=subprocess.PIPE)

# Open the specified USDA file
omni.usd.get_context().open_stage(USDA_FILE_PATH)
stage = omni.usd.get_context().get_stage()

# Check if the stage loaded correctly
if not stage:
    print(f"Failed to load stage: {USDA_FILE_PATH}")
    simulation_app.close()
    exit()

# Update the camera path to use the head camera
camera_path = "/World/alfred_parent_prim/alfred_base_descr/chest_cam_rgb_camera_frame/chest_cam"
# Get the camera prim
camera_prim = stage.GetPrimAtPath(camera_path)
if not camera_prim:
    print(f"Failed to find camera at path: {camera_path}")
    simulation_app.close()
    exit()

# Add a delay to allow the scene to load
print("Waiting 5 seconds for scene to initialize...")
time.sleep(5)

# Create a render product for the camera
render_product = rep.create.render_product(camera_path, resolution=(width, height))
print("[Setup] Successfully created render product")

# Attach an RGB annotator to the render product
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach(render_product)
print("[Setup] Successfully attached RGB annotator")

# Start the streamer
#streamer.start()
#print("[Setup] Successfully started video streamer")

try:
    print("[Stream] Starting camera stream loop...")
    frame_count = 0
    start_time = time.time()
    last_fps_print = time.time()
    
    while True:
        # Time the full frame processing
        frame_start = time.time()
        
        # Step the simulation to generate a new frame
        rep.orchestrator.step()
        
        # Get RGB data and convert to BGR
        start = time.time()
        frame = rgb_annotator.get_data()
        get_data_time = time.time() - start
        print(f"[Stream] Getting frame data took {get_data_time*1000:.2f}ms")
        
        start = time.time()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        cvt_time = time.time() - start
        print(f"[Stream] Color conversion took {cvt_time*1000:.2f}ms")
        
        # Ensure frame is contiguous
        # if not frame.flags['C_CONTIGUOUS']:
        #     frame = np.ascontiguousarray(frame)
            
        # Write frame directly to FFmpeg
        proc.stdin.write(frame.tobytes())
        proc.stdin.flush()
        
        # Calculate and print total frame time
        frame_time = time.time() - frame_start
        print(f"[Stream] Total frame processing took {frame_time*1000:.2f}ms")
        
        frame_count += 1
        if frame_count % 100 == 0:
            current_time = time.time()
            elapsed_time = current_time - start_time
            fps = frame_count / elapsed_time
            print(f"[Stream] Processed {frame_count} frames | Current FPS: {fps:.2f}")
        
except KeyboardInterrupt:
    print("\n[Stream] Received keyboard interrupt, stopping stream...")
finally:
    # Clean up
    print("[Cleanup] Stopping FFmpeg process...")
    proc.stdin.close()
    proc.wait()
    print("[Cleanup] Closing simulation...")
    simulation_app.close()
    print("[Cleanup] Successfully cleaned up resources")
