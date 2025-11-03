from isaacsim import SimulationApp
# Initialize the Isaac Sim application in headless mode
simulation_app = SimulationApp({"headless": True})

import os
import omni.usd
import omni.replicator.core as rep
from PIL import Image
from pxr import UsdGeom, Sdf
import time
from streaming.nvenc_streamer import NVENCStreamer

# Specify the input USDA file
USDA_FILE_PATH = "/dimos/assets/TestSim3.usda"

# Initialize the video streamer
streamer = NVENCStreamer(
    width=1920,
    height=1080,
    fps=30
    )

# Open the specified USDA file
omni.usd.get_context().open_stage(USDA_FILE_PATH)
stage = omni.usd.get_context().get_stage()

# Check if the stage loaded correctly
if not stage:
    print(f"Failed to load stage: {USDA_FILE_PATH}")
    simulation_app.close()
    exit()

# Update the camera path to use the head camera
camera_path = "/World/alfred_parent_prim/alfred_base_descr/head_cam_rgb_camera_frame/head_cam"

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
render_product = rep.create.render_product(camera_path, resolution=(1920, 1080))
print("[Setup] Successfully created render product")

# Attach an RGB annotator to the render product
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach(render_product)
print("[Setup] Successfully attached RGB annotator")

# Start the streamer
streamer.start()
print("[Setup] Successfully started video streamer")

try:
    print("[Stream] Starting camera stream loop...")
    frame_count = 0
    start_time = time.time()
    
    while True:
        # Step the simulation to generate a new frame
        rep.orchestrator.step()
        
        # Get RGB data and stream it
        rgb_data = rgb_annotator.get_data()
        streamer.push_frame(rgb_data)
        
        frame_count += 1
        if frame_count % 100 == 0:
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time
            print(f"[Stream] Processed {frame_count} frames | Current FPS: {fps:.2f}")
        
except KeyboardInterrupt:
    print("\n[Stream] Received keyboard interrupt, stopping stream...")
finally:
    # Clean up
    print("[Cleanup] Stopping video streamer...")
    streamer.stop()
    print("[Cleanup] Closing simulation...")
    simulation_app.close()
    print("[Cleanup] Successfully cleaned up resources")
