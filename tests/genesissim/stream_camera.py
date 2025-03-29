import os
from dimos.simulation.genesis import GenesisSimulator, GenesisStream

def main():
    # Add multiple entities at once
    entities = [
        {
            'type': 'primitive',
            'params': {'shape': 'plane'}
        },
        {
            'type': 'mjcf',
            'path': 'xml/franka_emika_panda/panda.xml'
        }
    ]
    # Initialize simulator
    sim = GenesisSimulator(
        headless=True,
        entities=entities
    )

    # You can also add entity individually
    sim.add_entity('primitive', shape='box', size=[0.5, 0.5, 0.5], pos=[0, 1, 0.5])
    
    # Create stream with custom settings
    stream = GenesisStream(
        simulator=sim,
        width=1280,  # Genesis default resolution
        height=960,
        fps=60,
        camera_path="/camera",  # Genesis uses simpler camera paths
        annotator_type='rgb',  # Can be 'rgb' or 'normals'
        transport='tcp',
        rtsp_url="rtsp://mediamtx:8554/stream"
    )
    
    # Start streaming
    try:
        stream.stream()
    except KeyboardInterrupt:
        print("\n[Stream] Received keyboard interrupt, stopping stream...")
    finally:
        try:
            stream.cleanup()
        finally:
            sim.close()

if __name__ == "__main__":
    main() 