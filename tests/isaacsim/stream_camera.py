from dimos.simulation.simulator import Simulator
from dimos.simulation.stream import SimulationStream

def main():
    # Initialize simulator
    sim = Simulator(headless=True)
    
    # Create stream with custom settings
    stream = SimulationStream(
        simulator=sim,
        width=1920,
        height=1080,
        fps=60,
        camera_path="/World/alfred_parent_prim/alfred_base_descr/chest_cam_rgb_camera_frame/chest_cam",
        annotator='rgb',
        transport='tcp',
        rtsp_url="rtsp://mediamtx:8554/stream",
        usd_path="./assets/TestSim3.usda"
    )
    
    # Start streaming
    stream.stream()

if __name__ == "__main__":
    main()
