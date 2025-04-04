import os
from dimos.simulation.isaac import IsaacSimulator
from dimos.simulation.isaac import IsaacStream

def main():
    # Initialize simulator
    sim = IsaacSimulator(headless=True)
    
    # Create stream with custom settings
    stream = IsaacStream(
        simulator=sim,
        width=1920,
        height=1080,
        fps=60,
        camera_path="/World/alfred_parent_prim/alfred_base_descr/chest_cam_rgb_camera_frame/chest_cam",
        annotator_type='rgb',
        transport='tcp',
        rtsp_url="rtsp://mediamtx:8554/stream",
        usd_path=f"{os.getcwd()}/assets/TestSim3.usda"
    )
    
    # Start streaming
    stream.stream()

if __name__ == "__main__":
    main()
