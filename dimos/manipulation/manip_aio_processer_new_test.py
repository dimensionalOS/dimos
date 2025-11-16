# test_pipeline_with_grasps.py

import numpy as np
import cv2
import threading
import time
import reactivex as rx
from manip_aio_pipeline import ManipulationPipeline

def test_pipeline_with_grasps():
    print("Starting Pipeline Test WITH Grasp Generation...")
    
    # Camera configurations
    camera_configs = [
    {
        "camera_id": 0,
        "intrinsics": [600.0, 600.0, 320.0, 240.0],
        "extrinsics": np.array([  # Camera 0's ArUco calibration
            [0.999,   0.012, -0.034,  0.152],
            [-0.011,  0.998,  0.045, -0.082],
            [0.035,  -0.044,  0.997,  0.723],
            [0,       0,      0,      1]
        ])
    },
    {
        "camera_id": 1,
        "intrinsics": [600.0, 600.0, 320.0, 240.0],
        "extrinsics": np.array([  # Camera 1's ArUco calibration
            [0.997,  -0.023,  0.071,  0.453],
            [0.025,   0.999, -0.018,  0.105],
            [-0.070,  0.021,  0.997,  0.695],
            [0,       0,      0,      1]
        ])
    }
]
    
    # Initialize pipeline WITH grasp generation
    pipeline = ManipulationPipeline(
        camera_configs=camera_configs,
        min_confidence=0.5,
        enable_grasp_generation=True,  # ENABLED!
        grasp_server_url="ws://localhost:8765",  # Make sure server is running
        max_grasps_per_object=5
    )
    
    print("Pipeline initialized with GRASP GENERATION enabled")
    
    # Create camera stream (same as before)
    def create_camera_stream(cam_id):
        def subscribe(observer):
            cap = cv2.VideoCapture(cam_id)
            
            def emit_frames():
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w = rgb.shape[:2]
                    
                    # For testing: create fake depth
                    # Replace with real depth camera!
                    depth = np.ones((h, w), dtype=np.float32) * 1.0
                    
                    observer.on_next({"rgb": rgb, "depth": depth})
                    time.sleep(0.033)
            
            thread = threading.Thread(target=emit_frames, daemon=True)
            thread.start()
            
        return rx.create(subscribe)
    
    # Create streams
    camera_streams = [create_camera_stream(0)]
    output_streams = pipeline.create_streams(camera_streams)
    
    # Windows for visualization
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Grasp Overlay", cv2.WINDOW_NORMAL)
    
    # SUBSCRIBE TO OBJECTS WITH GRASPS!
    def on_objects_with_grasps(objects):
        print("\n" + "="*60)
        print(f"OBJECTS WITH GRASPS: {len(objects)} objects")
        print("="*60)
        
        for i, obj in enumerate(objects):
            print(f"\nObject {i}: {obj.get('class_name', 'unknown')}")
            print(f"  Object ID: {obj.get('temp_id', 'N/A')}")
            
            # SHOW GRASP INFORMATION!
            if 'grasps' in obj and obj['grasps']:
                print(f"  Number of grasps: {len(obj['grasps'])}")
                
                for j, grasp in enumerate(obj['grasps'][:3]):  # Show top 3 grasps
                    print(f"\n  Grasp {j+1}:")
                    print(f"    Score: {grasp['score']:.3f}")
                    print(f"    Position: [{grasp['translation'][0]:.3f}, "
                          f"{grasp['translation'][1]:.3f}, {grasp['translation'][2]:.3f}]")
                    print(f"    Width: {grasp['width']*1000:.1f}mm")
                    print(f"    Grasp ID: {grasp['id']}")
            else:
                print("  No grasps generated (insufficient points or failed)")
    
    # Subscribe to grasp overlay visualization
    def on_grasp_overlay(overlay):
        if overlay is not None:
            viz_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
            cv2.imshow("Grasp Overlay", viz_bgr)
            
            # Save a snapshot
            cv2.imwrite("grasp_overlay.png", viz_bgr)
            print("Saved grasp visualization to grasp_overlay.png")
    
    # Subscribe to all outputs
    subscriptions = []
    
    # Main outputs to watch
    if "objects_with_grasps" in output_streams:
        sub = output_streams["objects_with_grasps"].subscribe(on_objects_with_grasps)
        subscriptions.append(sub)
        print("✓ Subscribed to objects_with_grasps stream")
    
    if "grasp_overlay" in output_streams:
        sub = output_streams["grasp_overlay"].subscribe(on_grasp_overlay)
        subscriptions.append(sub)
        print("✓ Subscribed to grasp_overlay stream")
    
    if "detection_viz" in output_streams:
        sub = output_streams["detection_viz"].subscribe(
            lambda viz: cv2.imshow("Detection", cv2.cvtColor(viz, cv2.COLOR_RGB2BGR)) if viz else None
        )
        subscriptions.append(sub)
    
    print("\n" + "="*60)
    print("PIPELINE TEST WITH GRASPS RUNNING!")
    print("="*60)
    print("\nOUTPUTS YOU SHOULD SEE:")
    print("1. Detection window - objects with bounding boxes")
    print("2. Grasp Overlay window - objects with grasp poses visualized")
    print("3. Console - detailed grasp information per object:")
    print("   - Grasp scores (quality)")
    print("   - 3D positions (end-effector targets)")
    print("   - Gripper widths")
    print("\nMAKE SURE:")
    print("- Grasp server is running (ws://localhost:8765)")
    print("- Objects are visible to camera")
    print("- Using real depth camera for best results")
    print("\nPress 'q' to quit")
    print("="*60 + "\n")
    
    # Main loop
    try:
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nStopping...")
    
    # Cleanup
    for sub in subscriptions:
        sub.dispose()
    pipeline.cleanup()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_pipeline_with_grasps()
{
  "object_id": "obj_0_timestamp",
  "class_name": "mug",
  "grasps": [
    {
      "id": "obj_0_grasp_0",
      "score": 0.95,
      "translation": [0.3, 0.1, 0.2],
      "rotation_matrix": [[1,0,0], [0,1,0], [0,0,1]],
      "width": 0.05
    }
  ]
}