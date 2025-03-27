import tests.test_header

import os
import time

from dimos.robot.unitree.unitree_go2 import UnitreeGo2, WebRTCConnectionMethod
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl

def get_env_var(var_name, default=None, required=False):
    """Get environment variable with validation."""
    value = os.getenv(var_name, default)
    if value == '':
        value = default
    if required and not value:
        raise ValueError(f"{var_name} environment variable is required")
    return value


if __name__ == "__main__":
    # Get configuration from environment variables
    robot_ip = get_env_var("ROBOT_IP", "192.168.9.140")
    connection_method = get_env_var("CONNECTION_METHOD", "LocalSTA")
    serial_number = get_env_var("SERIAL_NUMBER", None)
    output_dir = get_env_var("ROS_OUTPUT_DIR",
                             os.path.join(os.getcwd(), "assets/output/ros"))

    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    print(f"Ensuring output directory exists: {output_dir}")

    use_ros = True
    use_webrtc = False
    # Convert connection method string to enum
    connection_method = getattr(WebRTCConnectionMethod, connection_method)

    print("Initializing UnitreeGo2...")
    print(f"Configuration:")
    print(f"  IP: {robot_ip}")
    print(f"  Connection Method: {connection_method}")
    print(f"  Serial Number: {serial_number if serial_number else 'Not provided'}")
    print(f"  Output Directory: {output_dir}")

    if use_ros:
        ros_control = UnitreeROSControl(node_name="unitree_go2", use_raw=True)
    else:
        ros_control = None

    robot = UnitreeGo2(ip=robot_ip,
                       connection_method=connection_method,
                       serial_number=serial_number,
                       output_dir=output_dir,
                       ros_control=ros_control,
                       use_ros=use_ros,
                       use_webrtc=use_webrtc)
    time.sleep(5)
    try:

        # Start perception
        print("\nStarting perception system...")

        # Get the processed stream
        processed_stream = robot.get_ros_video_stream(fps=30)

        # Create frame counter for unique filenames
        frame_count = 0

        # Create a subscriber to handle the frames
        def handle_frame(frame):
            global frame_count
            frame_count += 1

            try:
                # Save frame to output directory if desired for debugging frame streaming
                # MAKE SURE TO CHANGE OUTPUT DIR depending on if running in ROS or local
                #frame_path = os.path.join(output_dir, f"frame_{frame_count:04d}.jpg")
                #success = cv2.imwrite(frame_path, frame)
                #print(f"Frame #{frame_count} {'saved successfully' if success else 'failed to save'} to {frame_path}")
                pass

            except Exception as e:
                print(f"Error in handle_frame: {e}")
                import traceback
                print(traceback.format_exc())

        def handle_error(error):
            print(f"Error in stream: {error}")

        def handle_completion():
            print("Stream completed")

        # Subscribe to the stream
        print("Creating subscription...")
        try:
            subscription = processed_stream.subscribe(
                on_next=handle_frame,
                on_error=lambda e: print(f"Subscription error: {e}"),
                on_completed=lambda: print("Subscription completed"))
            print("Subscription created successfully")
        except Exception as e:
            print(f"Error creating subscription: {e}")

        time.sleep(5)

        # First put the robot in a good starting state
        print("Running recovery stand...")
        robot.webrtc_req(api_id=1006)  # RecoveryStand

        # Queue 20 WebRTC requests back-to-back
        print("\n🤖 QUEUEING WEBRTC COMMANDS BACK-TO-BACK FOR TESTING UnitreeGo2🤖\n")

        # Dance 1
        robot.webrtc_req(api_id=1033)  
        print("Queued: WiggleHips (1033)")

        robot.reverse(distance=0.2, speed=0.5)
        print("Queued: Reverse 0.5m at 0.5m/s")

        # Wiggle Hips
        robot.webrtc_req(api_id=1033)  
        print("Queued: WiggleHips (1033)")

        robot.move(distance=0.2, speed=0.5)
        print("Queued: Move forward 1.0m at 0.5m/s")

        robot.webrtc_req(api_id=1017) 
        print("Queued: Stretch (1017)")

        robot.move(distance=0.2, speed=0.5)
        print("Queued: Move forward 1.0m at 0.5m/s")

        robot.webrtc_req(api_id=1017)  
        print("Queued: Stretch (1017)")

        robot.reverse(distance=0.2, speed=0.5)
        print("Queued: Reverse 0.5m at 0.5m/s")

        robot.webrtc_req(api_id=1017)  
        print("Queued: Stretch (1017)")\
        
        robot.spin(degrees=-90.0, speed=45.0)
        print("Queued: Spin right 90 degrees at 45 degrees/s")

        robot.spin(degrees=90.0, speed=45.0)    
        print("Queued: Spin left 90 degrees at 45 degrees/s")

        # To prevent termination
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping perception...")
        if 'subscription' in locals():
            subscription.dispose()
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Cleanup
        print("Cleaning up resources...")
        if 'subscription' in locals():
            subscription.dispose()
        del robot
        print("Cleanup complete.")
